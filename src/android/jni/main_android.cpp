
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>

#include <android/log.h>
#include <android/native_window_jni.h>

#include "common/common_paths.h"
#include "common/file_util.h"
#include "common/logging/filter.h"
#include "common/logging/log.h"
#include "core/core.h"
#include "core/frontend/applets/default_applets.h"
#include "core/settings.h"
#include "video_core/video_core.h"

#include "config/main_settings.h"
#include "egl_android.h"
#include "input_manager.h"
#include "jni_common.h"
#include "logcat_backend.h"

static ANativeWindow* s_surface = nullptr;

static std::atomic<bool> s_stop_run;
static std::atomic<bool> s_is_running;
static std::mutex s_running_mutex;
static std::condition_variable s_running_cv;
static std::unique_ptr<EGLAndroid> s_render_window;
static std::map<std::string, std::unique_ptr<Loader::AppLoader>> s_app_loaders;

void BootGame(const std::string& path) {
    s_render_window = std::make_unique<EGLAndroid>();
    s_render_window->Initialize(s_surface, 1.0f);

    Core::System& system{Core::System::GetInstance()};
    const Core::System::ResultStatus result{system.Load(*s_render_window, path)};
    if (result != Core::System::ResultStatus::Success) {
        switch (result) {
        case Core::System::ResultStatus::ErrorGetLoader:
            LOG_CRITICAL(Frontend, "Failed to obtain loader for {}!", path);
            LOG_CRITICAL(Frontend, "Invalid ROM Format, Your ROM format is not supported.");
            break;

        case Core::System::ResultStatus::ErrorSystemMode:
            LOG_CRITICAL(Frontend, "Failed to load ROM!");
            LOG_CRITICAL(Frontend, "ROM Corrupted, Your ROM is corrupted.");
            break;

        case Core::System::ResultStatus::ErrorLoader_ErrorEncrypted:
            LOG_CRITICAL(Frontend, "ROM Encrypted, Your ROM is encrypted.");
            break;

        case Core::System::ResultStatus::ErrorLoader_ErrorInvalidFormat:
            LOG_CRITICAL(Frontend, "Invalid ROM Format, Your ROM format is not supported.");
            break;

        case Core::System::ResultStatus::ErrorVideoCore:
            LOG_CRITICAL(
                Frontend,
                "Video Core Error, Ensure that you have the latest graphics drivers for your GPU.");
            break;

        case Core::System::ResultStatus::ErrorVideoCore_ErrorGenericDrivers:
            LOG_CRITICAL(Frontend,
                         "Video Core Error, You are running default Windows drivers for your GPU.");
            break;

        case Core::System::ResultStatus::ErrorVideoCore_ErrorBelowGL33:
            LOG_CRITICAL(Frontend, "Video Core Error, Opengl below GL33");
            break;

        default:
            LOG_CRITICAL(Frontend, "Error while loading ROM! An unknown error occured.");
            break;
        }
        return;
    }

    s_stop_run = false;
    s_is_running = true;
    s_render_window->MakeCurrent();
    while (!s_stop_run) {
        if (s_is_running) {
            Core::System::ResultStatus result = system.RunLoop();
            if (result == Core::System::ResultStatus::ShutdownRequested) {
                // End emulation execution
                break;
            } else if (result != Core::System::ResultStatus::Success) {
                LOG_CRITICAL(Frontend, "Error {}: {}", static_cast<u32>(result),
                             system.GetStatusDetails());
            }
        } else {
            std::unique_lock lock{s_running_mutex};
            s_running_cv.wait(lock, [] { return s_is_running || s_stop_run; });
        }
    }

    // Shutdown the core emulation
    system.Shutdown();
    s_render_window.reset();
    s_is_running = false;
}

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_SetUserPath(JNIEnv* env, jobject obj,
                                                                    jstring jPath) {
    // set log backend
    Log::Filter log_filter(Log::Level::Info);
    log_filter.ParseFilterString(Settings::values.log_filter);
    Log::SetGlobalFilter(log_filter);
    Log::AddBackend(std::make_unique<Log::LogcatBackend>());

    // init user path
    std::string path = GetJString(jPath);
    if (path[path.size() - 1] != '/')
        path.push_back('/');
    FileUtil::SetUserPath(path);

    // create user directory
    FileUtil::CreateFullPath(FileUtil::GetUserPath(FileUtil::UserPath::ConfigDir));
    FileUtil::CreateFullPath(FileUtil::GetUserPath(FileUtil::UserPath::LogDir));
    if (!FileUtil::Exists(FileUtil::GetUserPath(FileUtil::UserPath::ConfigDir) + "config-mmj.ini")) {
        Config::SaveDefault();
    }

    // Register frontend applets
    Frontend::RegisterDefaultApplets();

    // profile
    InputManager::GetInstance().InitProfile();
}

JNIEXPORT jboolean JNICALL Java_org_citra_emu_NativeLibrary_IsRunning(JNIEnv* env, jobject obj) {
    return s_is_running;
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_SurfaceChanged(JNIEnv* env, jobject obj,
                                                                       jobject surf) {
    s_surface = ANativeWindow_fromSurface(env, surf);
    if (s_render_window) {
        s_render_window->UpdateSurface(s_surface);
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_SurfaceDestroyed(JNIEnv* env, jobject obj) {
    ANativeWindow_release(s_surface);
    s_surface = nullptr;
    if (s_render_window) {
        s_render_window->UpdateSurface(s_surface);
    }
}

JNIEXPORT jboolean JNICALL Java_org_citra_emu_NativeLibrary_InputEvent(JNIEnv* env, jobject obj,
                                                                       jstring jDevice, jint button,
                                                                       jfloat value) {
    return InputManager::GetInstance().InputEvent(GetJString(jDevice), button, value);
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_TouchEvent(JNIEnv* env, jobject obj,
                                                                   jint action, jfloat x,
                                                                   jfloat y) {
    if (s_render_window) {
        switch (action) {
        case 0:
            s_render_window->TouchPressed(x, y);
            break;
        case 1:
            s_render_window->TouchMoved(x, y);
            break;
        case 2:
            s_render_window->TouchReleased();
            break;
        }
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_Run(JNIEnv* env, jobject obj,
                                                            jstring jFile) {
    // load
    Config::Load();
    // citra doesn't support arm jit
    Settings::values.use_cpu_jit = Config::Get(Config::USE_CPU_JIT);
    Settings::values.is_new_3ds = Config::Get(Config::IS_NEW_3DS);
    Settings::values.use_virtual_sd = Config::Get(Config::USE_VIRTUAL_SD);
    Settings::values.region_value = Config::Get(Config::SYSTEM_REGION);
    // use opengl es on android
    Settings::values.use_gles = Config::Get(Config::USE_GLES);
    Settings::values.use_hw_renderer = Config::Get(Config::USE_HW_RENDERER);
    Settings::values.use_hw_shader = Config::Get(Config::USE_HW_SHADER);
    Settings::values.use_shader_jit = Config::Get(Config::USE_SHADER_JIT);
    Settings::values.shaders_accurate_mul = Config::Get(Config::SHADERS_ACCURATE_MUL);
    Settings::values.shaders_accurate_gs = Config::Get(Config::SHADERS_ACCURATE_GS);
    Settings::values.use_frame_limit = Config::Get(Config::USE_FRAME_LIMIT);
    Settings::values.frame_limit = Config::Get(Config::FRAME_LIMIT);
    Settings::values.resolution_factor = Config::Get(Config::RESOLUTION_FACTOR);
    Settings::values.layout_option = Config::Get(Config::LAYOUT_OPTION);
    // audio
    Settings::values.enable_dsp_lle = Config::Get(Config::ENABLE_DSP_LLE);
    Settings::values.enable_dsp_lle_multithread = Config::Get(Config::DSP_LLE_MULTITHREAD);
    Settings::values.volume = Config::Get(Config::AUDIO_VOLUME);
    Settings::values.sink_id = Config::Get(Config::AUDIO_ENGINE);
    Settings::values.audio_device_id = Config::Get(Config::AUDIO_DEVICE);
    Settings::values.enable_audio_stretching = Config::Get(Config::AUDIO_STRETCHING);
    //
    Settings::values.init_clock = Settings::InitClock::SystemTime;
    Settings::values.init_time = 946681277;
    Settings::values.core_ticks_hack = 0;
    Settings::Apply();

    // settings
    Settings::LogSettings();
    // run
    BootGame(GetJString(jFile));
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_SaveScreenShot(JNIEnv* env, jobject obj) {
    u16 res_scale = VideoCore::GetResolutionScaleFactor();
    const Layout::FramebufferLayout layout{Layout::FrameLayoutFromResolutionScale(res_scale)};
    std::vector<u32> pixels(layout.width * layout.height);
    VideoCore::RequestScreenshot(
        pixels.data(),
        [=] {
            auto path = FileUtil::GetUserPath(FileUtil::UserPath::UserDir) + "screenshot.png";
            SaveImageToFile(path, layout.width, layout.height, pixels);
        },
        layout);
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_ResumeEmulation(JNIEnv* env, jobject obj) {
    s_is_running = true;
    s_running_cv.notify_all();
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_PauseEmulation(JNIEnv* env, jobject obj) {
    s_is_running = false;
    s_running_cv.notify_all();
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_StopEmulation(JNIEnv* env, jobject obj) {
    s_stop_run = true;
    s_running_cv.notify_all();
}

JNIEXPORT jintArray JNICALL Java_org_citra_emu_NativeLibrary_getRunningSettings(JNIEnv* env,
                                                                                jobject obj) {
    int i = 0;
    int settings[2];

    // get settings
    settings[i++] = sqrt(Settings::values.core_ticks_hack / 3);
    settings[i++] = Settings::values.layout_option == Settings::LayoutOption::SingleScreen;

    jintArray array = env->NewIntArray(i);
    env->SetIntArrayRegion(array, 0, i, settings);
    return array;
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_setRunningSettings(JNIEnv* env, jobject obj,
                                                                           jintArray array) {
    int i = 0;
    jint* settings = env->GetIntArrayElements(array, nullptr);

    // FMV Hack
    u32 ticks = settings[i++];
    Settings::values.core_ticks_hack = ticks * ticks * 3;

    // Change Layout
    bool single_screen = settings[i++] == 1;
    if (single_screen) {
        if (Settings::values.layout_option != Settings::LayoutOption::SingleScreen) {
            Settings::values.layout_option = Settings::LayoutOption::SingleScreen;
            s_render_window->UpdateLayout();
        }
    } else {
        if (Settings::values.layout_option == Settings::LayoutOption::SingleScreen) {
            Settings::values.layout_option = Config::Get(Config::LAYOUT_OPTION);
            s_render_window->UpdateLayout();
        }
    }

    env->ReleaseIntArrayElements(array, settings, 0);
}

JNIEXPORT jstring JNICALL Java_org_citra_emu_NativeLibrary_GetAppTitle(JNIEnv* env, jobject obj,
                                                                       jstring jPath) {
    std::string path = GetJString(jPath);
    auto iter = s_app_loaders.find(path);
    if (iter != s_app_loaders.end()) {
        std::string title;
        iter->second->ReadTitle(title);
        return ToJString(title);
    }

    auto result = s_app_loaders.emplace(path, Loader::GetLoader(path));
    if (result.second) {
        std::string title;
        result.first->second->ReadTitle(title);
        return ToJString(title);
    }

    return ToJString(std::string());
}

#ifdef __cplusplus
}
#endif
