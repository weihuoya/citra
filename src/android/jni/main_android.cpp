
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>

#include <android/log.h>
#include <android/native_window_jni.h>
#include <sys/system_properties.h>

#include "common/common_paths.h"
#include "common/file_util.h"
#include "core/core.h"
#include "core/cheats/cheats.h"
#include "core/frontend/applets/default_applets.h"
#include "core/hle/service/am/am.h"
#include "core/hle/service/cfg/cfg.h"
#include "core/hle/service/nfc/nfc.h"
#include "core/hle/service/ptm/ptm.h"
#include "core/loader/smdh.h"
#include "core/settings.h"
#include "video_core/video_core.h"

#include "camera/still_image_camera.h"
#include "config/main_settings.h"
#include "egl_android.h"
#include "input_manager.h"
#include "jni_common.h"
#include "keyboard.h"
#include "png_handler.h"
#include "mem_region.h"
#include "ndk_motion.h"

static ANativeWindow* s_surface = nullptr;

static std::atomic<bool> s_stop_running;
static std::atomic<bool> s_is_running;
static std::mutex s_running_mutex;
static std::condition_variable s_running_cv;
static std::unique_ptr<EGLAndroid> s_render_window;
static std::shared_ptr<AndroidKeyboard> s_keyboard;
static std::map<std::string, std::unique_ptr<Loader::AppLoader>> s_app_loaders;

void BootGame(const std::string& path) {
    s_render_window = std::make_unique<EGLAndroid>(Settings::values.use_present_thread);
    s_render_window->Initialize(s_surface);

    Core::System& system{Core::System::GetInstance()};

    // system config
    // std::shared_ptr<Service::CFG::Module> cfg = Service::CFG::GetModule(system);
    // cfg->SetSystemLanguage(Service::CFG::SystemLanguage::LANGUAGE_EN);
    // cfg->SetSoundOutputMode(Service::CFG::SoundOutputMode::SOUND_SURROUND);
    // cfg->SetCountryCode(49); // USA
    // cfg->UpdateConfigNANDSavegame();

    Core::System::ResultStatus result = system.Load(*s_render_window, path);
    if (result != Core::System::ResultStatus::Success) {
        switch (result) {
        case Core::System::ResultStatus::ErrorGetLoader:
            LOG_CRITICAL(Frontend, "Failed to obtain loader for {}!", path);
            ShowMessageDialog(0, "Invalid ROM Format, Your ROM format is not supported.");
            break;

        case Core::System::ResultStatus::ErrorSystemMode:
            LOG_CRITICAL(Frontend, "Failed to load ROM!");
            ShowMessageDialog(0, "ROM Corrupted, Your ROM is corrupted.");
            break;

        case Core::System::ResultStatus::ErrorLoader_ErrorEncrypted:
            ShowMessageDialog(0, "ROM Encrypted, Your ROM is encrypted.");
            break;

        case Core::System::ResultStatus::ErrorLoader_ErrorInvalidFormat:
            ShowMessageDialog(0, "Invalid ROM Format, Your ROM format is not supported.");
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
            ShowMessageDialog(0, "Error while loading ROM! An unknown error occured.");
            break;
        }
        return;
    }

    s_stop_running = false;
    s_is_running = true;
    while (!s_stop_running) {
        if (s_is_running) {
            result = system.RunLoop();
            if (result == Core::System::ResultStatus::ShutdownRequested) {
                // End emulation execution
                break;
            } else if (result != Core::System::ResultStatus::Success) {
                ShowMessageDialog(0, fmt::format("Error {}: {}", static_cast<u32>(result),
                                                 system.GetStatusDetails()));
                break;
            }
        } else {
            // Ensure no audio bleeds out while game is paused
            const float volume = Settings::values.volume;
            Settings::values.volume = 0;

            std::unique_lock lock{s_running_mutex};
            s_running_cv.wait(lock, [] { return s_is_running || s_stop_running; });
            s_render_window->PollEvents();
            Settings::values.volume = volume;
        }
    }

    // Shutdown the core emulation
    s_render_window->DoneCurrent();
    system.Shutdown();
    s_render_window.reset();
    s_is_running = false;
    resetSearchResults();
    NotifyGameShudown();
}

static Loader::AppLoader* GetAppLoader(const std::string& path) {
    auto iter = s_app_loaders.find(path);
    if (iter != s_app_loaders.end()) {
        return iter->second.get();
    }

    auto result = s_app_loaders.emplace(path, Loader::GetLoader(path));
    return result.first->second.get();
}

static bool GetSMDHData(Loader::AppLoader* loader, Loader::SMDH* smdh) {
    std::vector<u8> smdh_data;
    [&loader, &smdh_data]() -> void {
        u64 program_id = 0;
        loader->ReadProgramId(program_id);
        loader->ReadIcon(smdh_data);
        if (program_id < 0x00040000'00000000 || program_id > 0x00040000'FFFFFFFF)
            return;

        std::string update_path = Service::AM::GetTitleContentPath(
            Service::FS::MediaType::SDMC, program_id + 0x0000000E'00000000);

        if (!FileUtil::Exists(update_path))
            return;

        std::unique_ptr<Loader::AppLoader> update_loader = Loader::GetLoader(update_path);
        if (!update_loader)
            return;

        smdh_data.clear();
        update_loader->ReadIcon(smdh_data);
    }();

    if (Loader::IsValidSMDH(smdh_data)) {
        memcpy(smdh, smdh_data.data(), sizeof(Loader::SMDH));
        return true;
    }

    return false;
}

static std::vector<u16> GetIconData(Loader::AppLoader* loader) {
    Loader::SMDH smdh;
    if (GetSMDHData(loader, &smdh)) {
        return smdh.GetIcon(true);
    }
    return std::vector<u16>();
}

static std::vector<Loader::SMDH::GameRegion> GetGameRegions(Loader::AppLoader* loader) {
    Loader::SMDH smdh;
    if (GetSMDHData(loader, &smdh)) {
        return smdh.GetRegions();
    }
    return {};
}

static int GetPropertyInteger(const char * name, int defaultValue) {
    int result = defaultValue;
#ifdef __ANDROID__
    char valueText[PROP_VALUE_MAX] = {0};
    if (__system_property_get(name, valueText) != 0) {
        result = atoi(valueText);
    }
#else
    (void) name;
#endif
    return result;
}

static int GetSdkVersion() {
    static int sCachedSdkVersion = -1;
#ifdef __ANDROID__
    if (sCachedSdkVersion == -1) {
        sCachedSdkVersion = GetPropertyInteger("ro.build.version.sdk", -1);
    }
#endif
    return sCachedSdkVersion;
}

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_InstallCIA(JNIEnv* env, jclass obj,
                                                                   jobjectArray jPaths) {
    const std::vector<std::string> paths = JStringArrayToVector(jPaths);
    Service::AM::InstallStatus status;
    for (const auto& path : paths) {
        status = Service::AM::InstallCIA(
            path, std::bind(UpdateProgress, path, std::placeholders::_1, std::placeholders::_2));
        if (Service::AM::InstallStatus::Success != status) {
            UpdateProgress(path, static_cast<u32>(status), 0);
        } else {
            UpdateProgress(path, 0, 0);
        }
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_SetUserPath(JNIEnv* env, jclass obj,
                                                                    jstring jPath) {
    // init user path
    std::string path = GetJString(jPath);
    if (path[path.size() - 1] != '/')
        path.push_back('/');
    FileUtil::SetUserPath(path);

    // create user directory
    FileUtil::CreateFullPath(FileUtil::GetUserPath(FileUtil::UserPath::AmiiboDir));
    FileUtil::CreateFullPath(FileUtil::GetUserPath(FileUtil::UserPath::CacheDir));
    FileUtil::CreateFullPath(FileUtil::GetUserPath(FileUtil::UserPath::ConfigDir));
    FileUtil::CreateFullPath(FileUtil::GetUserPath(FileUtil::UserPath::LogDir));
    if (!FileUtil::Exists(FileUtil::GetUserPath(FileUtil::UserPath::ConfigDir) +
                          "config-mmj.ini")) {
        Config::SaveDefault();
    }

    // Register frontend applets
    Frontend::RegisterDefaultApplets();
    s_keyboard = std::make_shared<AndroidKeyboard>();
    Core::System::GetInstance().RegisterSoftwareKeyboard(s_keyboard);
    Core::System::GetInstance().RegisterImageInterface(std::make_shared<PNGHandler>());
    Camera::RegisterFactory("image", std::make_unique<Camera::StillImageCameraFactory>());
    if (GetSdkVersion() >= 26) {
        Input::RegisterFactory<Input::MotionDevice>("motion_emu", std::make_shared<NDKMotionFactory>());
    }

    //
    Core::System& system{Core::System::GetInstance()};
    system.nfc_scanning_callback = [](bool isScanning) -> void {
        HandleNFCScanning(isScanning);
    };
}

JNIEXPORT jboolean JNICALL Java_org_citra_emu_NativeLibrary_IsRunning(JNIEnv* env, jclass obj) {
    return s_is_running;
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_SurfaceChanged(JNIEnv* env, jclass obj,
                                                                       jobject surf) {
    s_surface = ANativeWindow_fromSurface(env, surf);
    if (s_render_window) {
        s_render_window->UpdateSurface(s_surface);
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_SurfaceDestroyed(JNIEnv* env, jclass obj) {
    ANativeWindow_release(s_surface);
    s_surface = nullptr;
    if (s_render_window) {
        s_render_window->UpdateSurface(s_surface);
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_DoFrame(JNIEnv* env, jclass obj) {
    if(!s_is_running || s_stop_running) {
        return;
    }
    s_render_window->TryPresenting();
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_HandleImage(JNIEnv* env, jclass obj,
                                                                    jintArray jPixels, jint width,
                                                                    jint height) {
    jint* pixels = nullptr;
    if (jPixels != nullptr) {
        pixels = env->GetIntArrayElements(jPixels, nullptr);
    }
    g_image_loading_callback(reinterpret_cast<u32*>(pixels), width, height);
    if (jPixels != nullptr) {
        env->ReleaseIntArrayElements(jPixels, pixels, 0);
    }
    g_image_loading_callback = nullptr;
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_InputEvent(JNIEnv* env, jclass obj,
                                                                   jint button, jfloat value) {
    InputManager::GetInstance().InputEvent(button, value);
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_TouchEvent(JNIEnv* env, jclass obj,
                                                                   jint action, jint x, jint y) {
    const int TOUCH_PRESSED = 1;
    const int TOUCH_MOVED = 2;
    const int TOUCH_RELEASED = 4;
    const int BEGIN_TILT = 8;
    const int TILT = 16;
    const int END_TILT = 32;

    if (s_render_window) {
        if (action & TOUCH_PRESSED) {
            s_render_window->TouchPressed(x, y);
        } else if (action & TOUCH_MOVED) {
            s_render_window->TouchMoved(x, y);
        } else if (action & TOUCH_RELEASED) {
            s_render_window->TouchReleased();
        }

        if (action & BEGIN_TILT) {
            InputManager::GetInstance().BeginTilt(-x / 3, -y / 3);
        } else if (action & TILT) {
            InputManager::GetInstance().Tilt(-x / 3, -y / 3);
        } else if (action & END_TILT) {
            InputManager::GetInstance().EndTilt();
        }
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_KeyboardEvent(JNIEnv* env, jclass obj,
                                                                      jint action, jstring jtext) {
    if (s_keyboard)
        s_keyboard->Accept(action, GetJString(jtext));
}

JNIEXPORT jboolean JNICALL Java_org_citra_emu_NativeLibrary_KeyEvent(JNIEnv* env, jclass obj,
                                                                     jint button, jint action) {
    return InputManager::GetInstance().KeyEvent(button, action);
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_MoveEvent(JNIEnv* env, jclass obj,
                                                                  jint axis, jfloat value) {
    InputManager::GetInstance().KeyEvent(axis, value);
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_Run(JNIEnv* env, jclass obj,
                                                            jstring jFile) {
    // reload config
    Config::Clear();
    Config::Load();
    // system
    Settings::values.use_cpu_jit = Config::Get(Config::USE_CPU_JIT);
    Settings::values.is_new_3ds = Config::Get(Config::IS_NEW_3DS);
    Settings::values.use_virtual_sd = Config::Get(Config::USE_VIRTUAL_SD);
    Settings::values.region_value = Config::Get(Config::SYSTEM_REGION);
    // renderer
    Settings::values.use_gles = Config::Get(Config::USE_GLES);
    Settings::values.show_fps = Config::Get(Config::SHOW_FPS);
    Settings::values.use_hw_renderer = Config::Get(Config::USE_HW_RENDERER);
    Settings::values.use_hw_shader = Config::Get(Config::USE_HW_SHADER);
    Settings::values.use_shader_jit = Config::Get(Config::USE_SHADER_JIT);
    Settings::values.shaders_accurate_mul = Config::Get(Config::SHADERS_ACCURATE_MUL);
    Settings::values.use_frame_limit = Config::Get(Config::USE_FRAME_LIMIT);
    Settings::values.frame_limit = Config::Get(Config::FRAME_LIMIT);
    Settings::values.resolution_factor = Config::Get(Config::RESOLUTION_FACTOR);
    Settings::values.factor_3d = Config::Get(Config::FACTOR_3D);
    Settings::values.texture_load_hack = Config::Get(Config::TEXTURE_LOAD_HACK);
    Settings::values.custom_textures = Config::Get(Config::CUSTOM_TEXTURES);
    Settings::values.preload_textures = Config::Get(Config::PRELOAD_TEXTURES);
    Settings::values.layout_option = Config::Get(Config::LAYOUT_OPTION);
    Settings::values.pp_shader_name = Config::Get(Config::POST_PROCESSING_SHADER);
    // audio
    Settings::values.enable_dsp_lle = Config::Get(Config::ENABLE_DSP_LLE);
    Settings::values.enable_dsp_lle_multithread = Config::Get(Config::DSP_LLE_MULTITHREAD);
    Settings::values.volume = Config::Get(Config::AUDIO_VOLUME);
    Settings::values.sink_id = Config::Get(Config::AUDIO_ENGINE);
    Settings::values.audio_device_id = Config::Get(Config::AUDIO_DEVICE);
    Settings::values.enable_audio_stretching = Config::Get(Config::AUDIO_STRETCHING);
    // mic
    Settings::values.mic_input_type = Config::Get(Config::MIC_INPUT_TYPE);
    Settings::values.mic_input_device = Config::Get(Config::MIC_INPUT_DEVICE);
    // debug
    Settings::values.allow_shadow = Config::Get(Config::ALLOW_SHADOW);
    Settings::values.use_present_thread = Config::Get(Config::USE_PRESENT_THREAD);
    u8 shaderType = Config::Get(Config::SHADER_TYPE);
    if (shaderType == 0) {
        Settings::values.use_separable_shader = false;
        Settings::values.use_shader_cache = false;
    } else if (shaderType == 1) {
        Settings::values.use_separable_shader = false;
        Settings::values.use_shader_cache = true;
    } else {
        Settings::values.use_separable_shader = true;
        Settings::values.use_shader_cache = false;
    }
    // custom layout
    Settings::values.custom_layout = Config::Get(Config::USE_CUSTOM_LAYOUT);
    Settings::values.custom_top_left = Config::Get(Config::CUSTOM_TOP_LEFT);
    Settings::values.custom_top_top = Config::Get(Config::CUSTOM_TOP_TOP);
    Settings::values.custom_top_right = Config::Get(Config::CUSTOM_TOP_RIGHT);
    Settings::values.custom_top_bottom = Config::Get(Config::CUSTOM_TOP_BOTTOM);
    Settings::values.custom_bottom_left = Config::Get(Config::CUSTOM_BOTTOM_LEFT);
    Settings::values.custom_bottom_top = Config::Get(Config::CUSTOM_BOTTOM_TOP);
    Settings::values.custom_bottom_right = Config::Get(Config::CUSTOM_BOTTOM_RIGHT);
    Settings::values.custom_bottom_bottom = Config::Get(Config::CUSTOM_BOTTOM_BOTTOM);
    //
    Settings::values.init_clock = Settings::InitClock::SystemTime;
    Settings::values.init_time = 946681277;
    Settings::values.core_ticks_hack = 0;
    Settings::values.skip_slow_draw = false;
    Settings::values.display_transfer_hack = false;
    Settings::values.stream_buffer_hack = !Settings::values.use_present_thread;
    Settings::Apply();

    // profile
    InputManager::GetInstance().InitProfile();

    // camera
    Settings::values.camera_name[Service::CAM::OuterRightCamera] = Config::Get(Config::CAMERA_DEVICE);
    Settings::values.camera_name[Service::CAM::InnerCamera] = Config::Get(Config::CAMERA_DEVICE);
    Settings::values.camera_name[Service::CAM::OuterLeftCamera] = Config::Get(Config::CAMERA_DEVICE);

    // play coin
    Service::PTM::Module::SetPlayCoins(99);

    // language
    std::shared_ptr<Service::CFG::Module> cfg = std::make_shared<Service::CFG::Module>();
    cfg->SetSystemLanguage(Config::Get(Config::SYSTEM_LANGUAGE));
    cfg->UpdateConfigNANDSavegame();

    // run
    BootGame(GetJString(jFile));
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_ResumeEmulation(JNIEnv* env, jclass obj) {
    s_is_running = true;
    s_running_cv.notify_all();
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_PauseEmulation(JNIEnv* env, jclass obj) {
    s_is_running = false;
    s_running_cv.notify_all();
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_StopEmulation(JNIEnv* env, jclass obj) {
    s_stop_running = true;
    s_render_window->StopPresenting();
    s_running_cv.notify_all();
}

JNIEXPORT jintArray JNICALL Java_org_citra_emu_NativeLibrary_getRunningSettings(JNIEnv* env,
                                                                                jclass obj) {
    int i = 0;
    int settings[5];

    // get settings
    settings[i++] = Settings::values.core_ticks_hack > 0;
    settings[i++] = Settings::values.skip_slow_draw;
    settings[i++] = Settings::values.texture_load_hack;
    settings[i++] = std::min(std::max(Settings::values.resolution_factor - 1, 0), 3);
    settings[i++] = static_cast<int>(Settings::values.layout_option);

    jintArray array = env->NewIntArray(i);
    env->SetIntArrayRegion(array, 0, i, settings);
    return array;
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_setRunningSettings(JNIEnv* env, jclass obj,
                                                                           jintArray array) {
    int i = 0;
    jint* settings = env->GetIntArrayElements(array, nullptr);

    // FMV Hack
    if (settings[i++]) {
        if (Settings::values.use_cpu_jit) {
            Settings::values.core_ticks_hack = 16000;
        } else {
            Settings::values.core_ticks_hack = 0xFFFF;
        }
    } else {
        Settings::values.core_ticks_hack = 0;
    }

    // Skip Slow Draw
    Settings::values.skip_slow_draw = settings[i++] > 0;

    //
    Settings::values.texture_load_hack = settings[i++] > 0;

    // Scale Factor
    Settings::values.resolution_factor = settings[i++] + 1;

    // Change Layout
    auto screen_layout = static_cast<Settings::LayoutOption>(settings[i++]);
    if (Settings::values.layout_option != screen_layout) {
        Settings::values.layout_option = screen_layout;
        s_render_window->UpdateLayout();
    }

    env->ReleaseIntArrayElements(array, settings, 0);
}

JNIEXPORT jintArray JNICALL Java_org_citra_emu_NativeLibrary_searchMemory(JNIEnv* env, jclass obj,
                                                                           jint start_addr,
                                                                           jint stop_addr,
                                                                           jint value_type,
                                                                           jint search_type,
                                                                           jint scan_type,
                                                                           jint value) {
    return searchMemoryRegion(start_addr, stop_addr, value_type, search_type, scan_type, value);
}

JNIEXPORT jintArray JNICALL Java_org_citra_emu_NativeLibrary_getSearchResults(JNIEnv* env, jclass obj) {
    return getSearchResults();
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_resetSearchResults(JNIEnv* env, jclass obj) {
    resetSearchResults();
}

JNIEXPORT jintArray JNICALL Java_org_citra_emu_NativeLibrary_loadPageTable(JNIEnv* env, jclass obj) {
    u32 start_addr = 0;
    u32 mem_size = 0;
    std::vector<u32> pages;
    Core::System& system{Core::System::GetInstance()};
    auto pagetable = system.Memory().GetCurrentPageTable();

    for (u32 i = 0; i < pagetable->pointers.size(); ++i) {
        auto p = pagetable->pointers[i];
        if (p != nullptr) {
            u32 addr = i << Memory::PAGE_BITS;
            if ((start_addr + mem_size) != addr) {
                if (mem_size > 0) {
                    pages.push_back(start_addr);
                    pages.push_back(mem_size);
                    mem_size = 0;
                }
                start_addr = addr;
            }
            mem_size += Memory::PAGE_SIZE;
        }
    }

    jintArray array = env->NewIntArray(pages.size());
    env->SetIntArrayRegion(array, 0, pages.size(), reinterpret_cast<const jint*>(pages.data()));
    return array;
}

JNIEXPORT jbyteArray JNICALL Java_org_citra_emu_NativeLibrary_loadPage(JNIEnv* env, jclass obj, jint index) {
    Core::System& system{Core::System::GetInstance()};
    auto p = system.Memory().GetCurrentPageTable()->pointers[index];
    if (p != nullptr) {
        jbyteArray array = env->NewByteArray(Memory::PAGE_SIZE);
        env->SetByteArrayRegion(array, 0, Memory::PAGE_SIZE, reinterpret_cast<const jbyte *>(p));
        return array;
    } else {
        return nullptr;
    }
}

JNIEXPORT jint JNICALL Java_org_citra_emu_NativeLibrary_readMemory(JNIEnv* env, jclass obj, jint jAddr, jint valueType) {
    u32 addr = static_cast<u32>(jAddr);
    u32 index = addr >> Memory::PAGE_BITS;
    u32 offset = addr & Memory::PAGE_MASK;
    Core::System& system{Core::System::GetInstance()};
    auto p = system.Memory().GetCurrentPageTable()->pointers[index];
    if (p != nullptr) {
        if (valueType == 0) {
            return *reinterpret_cast<u32*>(p + offset);
        } else if (valueType == 1) {
            return *reinterpret_cast<u16*>(p + offset);
        } else if (valueType == 2) {
            return *reinterpret_cast<u8*>(p + offset);
        }
    }
    return 0;
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_writeMemory(JNIEnv* env, jclass obj, jint jAddr, jint valueType, jint value) {
    u32 addr = static_cast<u32>(jAddr);
    u32 index = addr >> Memory::PAGE_BITS;
    u32 offset = addr & Memory::PAGE_MASK;
    Core::System& system{Core::System::GetInstance()};
    auto p = system.Memory().GetCurrentPageTable()->pointers[index];
    if (p != nullptr) {
        if (valueType == 0) {
            *reinterpret_cast<u32*>(p + offset) = static_cast<u32>(value);
        } else if (valueType == 1) {
            *reinterpret_cast<u16*>(p + offset) = static_cast<u16>(value);
        } else if (valueType == 2) {
            *reinterpret_cast<u8*>(p + offset) = static_cast<u8>(value);
        }
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_reloadCheatCode(JNIEnv* env, jclass obj) {
    Core::System& system{Core::System::GetInstance()};
    system.CheatEngine().ReloadCheatFile();
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_loadAmiibo(JNIEnv* env, jclass obj,
                                                                   jstring jPath) {
    std::string path = GetJString(jPath);

    Core::System& system{Core::System::GetInstance()};
    Service::SM::ServiceManager& sm = system.ServiceManager();
    auto nfc = sm.GetService<Service::NFC::Module::Interface>("nfc:u");
    if (nfc == nullptr) {
        return;
    }

    FileUtil::IOFile amiibo_file(path, "rb");
    Service::NFC::AmiiboData amiibo_data{};
    std::size_t size = amiibo_file.ReadBytes(&amiibo_data, sizeof(amiibo_data));
    if (size == sizeof(amiibo_data)) {
        nfc->LoadAmiibo(amiibo_data);
    }
    amiibo_file.Close();
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_ResetCamera(JNIEnv* env, jclass obj) {
    Settings::values.camera_name[Service::CAM::OuterRightCamera] = "image";
    Settings::values.camera_name[Service::CAM::InnerCamera] = "image";
    Settings::values.camera_name[Service::CAM::OuterLeftCamera] = "image";
    Settings::Apply();
}

JNIEXPORT jstring JNICALL Java_org_citra_emu_NativeLibrary_GetAppId(JNIEnv* env, jclass obj,
                                                                    jstring jPath) {
    Loader::AppLoader* app_loader = GetAppLoader(GetJString(jPath));
    u64 programId;
    app_loader->ReadProgramId(programId);
    return ToJString(fmt::format("{:016X}", programId));
}

JNIEXPORT jstring JNICALL Java_org_citra_emu_NativeLibrary_GetAppTitle(JNIEnv* env, jclass obj,
                                                                       jstring jPath) {
    Loader::AppLoader* app_loader = GetAppLoader(GetJString(jPath));
    std::string title;
    app_loader->ReadTitle(title);
    return ToJString(title);
}

JNIEXPORT jintArray JNICALL Java_org_citra_emu_NativeLibrary_GetAppIcon(JNIEnv* env, jclass obj,
                                                                        jstring jPath) {
    Loader::AppLoader* app_loader = GetAppLoader(GetJString(jPath));
    std::vector<u16> icon = GetIconData(app_loader);
    return ToJIntArray(reinterpret_cast<u32*>(icon.data()), icon.size() / 2);
}

JNIEXPORT jint JNICALL Java_org_citra_emu_NativeLibrary_GetAppRegion(JNIEnv* env, jclass obj,
                                                                     jstring jPath) {
    Loader::AppLoader* app_loader = GetAppLoader(GetJString(jPath));
    std::vector<Loader::SMDH::GameRegion> regions = GetGameRegions(app_loader);
    if (regions.empty()) {
        regions.push_back(Loader::SMDH::GameRegion::Japan);
    }
    return static_cast<jint>(regions[0]);
}

JNIEXPORT jboolean JNICALL Java_org_citra_emu_NativeLibrary_IsAppExecutable(JNIEnv* env, jclass obj,
                                                                            jstring jPath) {
    bool executable = false;
    Loader::AppLoader* app_loader = GetAppLoader(GetJString(jPath));
    if (app_loader) {
        app_loader->IsExecutable(executable);
    }
    return executable;
}

#ifdef __cplusplus
}
#endif
