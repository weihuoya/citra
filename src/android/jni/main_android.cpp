#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>

#ifdef ANDROID
#include <android/log.h>
#endif

#include <android/native_window_jni.h>

#include "common/common_paths.h"
#include "common/file_util.h"
#include "core/cheats/cheats.h"
#include "core/core.h"
#include "core/frontend/applets/default_applets.h"
#include "core/frontend/framebuffer_layout.h"
#include "core/frontend/mic.h"
#include "core/hle/service/am/am.h"
#include "core/hle/service/cfg/cfg.h"
#include "core/hle/service/hid/hid.h"
#include "core/hle/service/nfc/nfc.h"
#include "core/hle/service/ptm/ptm.h"
#include "core/loader/smdh.h"
#include "core/settings.h"
#include "video_core/video_core.h"

#include "camera/ndk_camera.h"
#include "camera/still_image_camera.h"
#include "config/main_settings.h"
#include "egl_android.h"
#include "input_manager.h"
#include "jni_common.h"
#include "keyboard.h"
#include "mem_region.h"
#include "mic.h"
#include "multiplayer.h"
#include "png_handler.h"
#include "saf_handler.h"

static ANativeWindow* s_surface = nullptr;

static std::atomic<bool> s_update_hid;
static std::atomic<bool> s_stop_running;
static std::atomic<bool> s_is_running;
static std::mutex s_running_mutex;
static std::condition_variable s_running_cv;
static std::unique_ptr<EGLAndroid> s_render_window;
static std::shared_ptr<AndroidKeyboard> s_keyboard;

namespace {

#ifdef ANDROID

using PerfClock = std::chrono::steady_clock;
constexpr u64 PERF_LOG_INTERVAL_NS = 5'000'000'000ULL;

u64 PerfNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               PerfClock::now().time_since_epoch())
        .count();
}

double PerfNsToMs(u64 ns) {
    return static_cast<double>(ns) / 1'000'000.0;
}

void UpdateAtomicMax(std::atomic<u64>& target, u64 value) {
    u64 current = target.load(std::memory_order_relaxed);
    while (current < value &&
           !target.compare_exchange_weak(current, value, std::memory_order_relaxed,
                                         std::memory_order_relaxed)) {
    }
}

struct DoFramePerfStats {
    std::atomic<u64> last_log_ns{0};
    std::atomic<u64> calls{0};
    std::atomic<u64> no_window{0};
    std::atomic<u64> stopped{0};
    std::atomic<u64> direct_path{0};
    std::atomic<u64> paused{0};
    std::atomic<u64> active_present{0};
    std::atomic<u64> active_present_total_ns{0};
    std::atomic<u64> active_present_max_ns{0};
};

DoFramePerfStats g_do_frame_perf_stats;

void MaybeLogDoFramePerf(u64 now_ns) {
    u64 last_log_ns = g_do_frame_perf_stats.last_log_ns.load(std::memory_order_relaxed);
    if (last_log_ns != 0 && now_ns - last_log_ns < PERF_LOG_INTERVAL_NS) {
        return;
    }
    if (!g_do_frame_perf_stats.last_log_ns.compare_exchange_strong(last_log_ns, now_ns,
                                                                   std::memory_order_relaxed,
                                                                   std::memory_order_relaxed)) {
        return;
    }
    if (last_log_ns == 0) {
        return;
    }

    const auto calls = g_do_frame_perf_stats.calls.exchange(0, std::memory_order_relaxed);
    const auto no_window = g_do_frame_perf_stats.no_window.exchange(0, std::memory_order_relaxed);
    const auto stopped = g_do_frame_perf_stats.stopped.exchange(0, std::memory_order_relaxed);
    const auto direct_path =
        g_do_frame_perf_stats.direct_path.exchange(0, std::memory_order_relaxed);
    const auto paused = g_do_frame_perf_stats.paused.exchange(0, std::memory_order_relaxed);
    const auto active_present =
        g_do_frame_perf_stats.active_present.exchange(0, std::memory_order_relaxed);
    const auto active_present_total_ns =
        g_do_frame_perf_stats.active_present_total_ns.exchange(0, std::memory_order_relaxed);
    const auto active_present_max_ns =
        g_do_frame_perf_stats.active_present_max_ns.exchange(0, std::memory_order_relaxed);

    if (calls == 0) {
        return;
    }

    const double avg_present_ms =
        active_present == 0 ? 0.0 : PerfNsToMs(active_present_total_ns) / active_present;

    __android_log_print(ANDROID_LOG_INFO, "citra",
                        "[Perf][DoFrame] window_ms=%.1f calls=%llu no_window=%llu stopped=%llu "
                        "direct_off=%llu paused=%llu active=%llu avg_present_ms=%.3f "
                        "max_present_ms=%.3f",
                        PerfNsToMs(now_ns - last_log_ns), static_cast<unsigned long long>(calls),
                        static_cast<unsigned long long>(no_window),
                        static_cast<unsigned long long>(stopped),
                        static_cast<unsigned long long>(direct_path),
                        static_cast<unsigned long long>(paused),
                        static_cast<unsigned long long>(active_present), avg_present_ms,
                        PerfNsToMs(active_present_max_ns));
}

void RecordDoFrameNoWindow() {
    const auto now_ns = PerfNowNs();
    g_do_frame_perf_stats.calls.fetch_add(1, std::memory_order_relaxed);
    g_do_frame_perf_stats.no_window.fetch_add(1, std::memory_order_relaxed);
    MaybeLogDoFramePerf(now_ns);
}

void RecordDoFrameStopped() {
    const auto now_ns = PerfNowNs();
    g_do_frame_perf_stats.calls.fetch_add(1, std::memory_order_relaxed);
    g_do_frame_perf_stats.stopped.fetch_add(1, std::memory_order_relaxed);
    MaybeLogDoFramePerf(now_ns);
}

void RecordDoFrameDirectPath() {
    const auto now_ns = PerfNowNs();
    g_do_frame_perf_stats.calls.fetch_add(1, std::memory_order_relaxed);
    g_do_frame_perf_stats.direct_path.fetch_add(1, std::memory_order_relaxed);
    MaybeLogDoFramePerf(now_ns);
}

void RecordDoFramePaused() {
    const auto now_ns = PerfNowNs();
    g_do_frame_perf_stats.calls.fetch_add(1, std::memory_order_relaxed);
    g_do_frame_perf_stats.paused.fetch_add(1, std::memory_order_relaxed);
    MaybeLogDoFramePerf(now_ns);
}

void RecordDoFrameActivePresent(u64 elapsed_ns) {
    const auto now_ns = PerfNowNs();
    g_do_frame_perf_stats.calls.fetch_add(1, std::memory_order_relaxed);
    g_do_frame_perf_stats.active_present.fetch_add(1, std::memory_order_relaxed);
    g_do_frame_perf_stats.active_present_total_ns.fetch_add(elapsed_ns,
                                                            std::memory_order_relaxed);
    UpdateAtomicMax(g_do_frame_perf_stats.active_present_max_ns, elapsed_ns);
    MaybeLogDoFramePerf(now_ns);
}

void LogAndroidRunConfig(const std::string& file_path) {
    __android_log_print(
        ANDROID_LOG_INFO, "citra",
        "[Perf][RunConfig] file=%s present_thread=%d fence_sync=%d stream_buffer_hack=%d "
        "use_gles=%d resolution_factor=%d frame_limit=%d accurate_mul=%d shadow_rendering=%d",
        file_path.c_str(), static_cast<int>(Settings::values.use_present_thread),
        static_cast<int>(Settings::values.use_fence_sync),
        static_cast<int>(Settings::values.stream_buffer_hack),
        static_cast<int>(Settings::values.use_gles),
        static_cast<int>(Settings::values.resolution_factor),
        static_cast<int>(Settings::values.frame_limit),
        static_cast<int>(Settings::values.shaders_accurate_mul),
        static_cast<int>(Settings::values.shadow_rendering));
}

#else

void RecordDoFrameNoWindow() {}
void RecordDoFrameStopped() {}
void RecordDoFrameDirectPath() {}
void RecordDoFramePaused() {}
void RecordDoFrameActivePresent(u64) {}
void LogAndroidRunConfig(const std::string&) {}

#endif

} // namespace

static std::string GetAndroidAudioOutputSink(u8 output_type) {
    switch (output_type) {
    case 1:
        return "null";
    case 2:
        return "cubeb";
    default:
        return "auto";
    }
}

static Settings::MicInputType GetAndroidMicInputType(u8 input_type) {
    switch (input_type) {
    case 2:
        return Settings::MicInputType::Static;
    case 3:
        return Settings::MicInputType::Real;
    case 1:
    default:
        return Settings::MicInputType::None;
    }
}

struct GameInfo {
    u64 id;
    u64 timestamp;
    std::string strid;
    std::string name;
    std::vector<u16> icon;
    std::vector<Loader::SMDH::GameRegion> regions;
    bool executable;
};
static std::map<std::string, GameInfo> s_app_dict;

void BootGame(const std::string& path) {
    NativeLibrary::UpdateProgress("BootGame", 0, 1);

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
            NativeLibrary::ShowMessageDialog(
                0, "Invalid ROM Format, Your ROM format is not supported.");
            break;

        case Core::System::ResultStatus::ErrorSystemMode:
            LOG_CRITICAL(Frontend, "Failed to load ROM!");
            NativeLibrary::ShowMessageDialog(0, "ROM Corrupted, Your ROM is corrupted.");
            break;

        case Core::System::ResultStatus::ErrorLoader_ErrorEncrypted:
            NativeLibrary::ShowMessageDialog(0, "ROM Encrypted, Your ROM is encrypted.");
            break;

        case Core::System::ResultStatus::ErrorLoader_ErrorInvalidFormat:
            NativeLibrary::ShowMessageDialog(
                0, "Invalid ROM Format, Your ROM format is not supported.");
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
            NativeLibrary::ShowMessageDialog(0,
                                             "Error while loading ROM! An unknown error occured.");
            break;
        }
        s_render_window.reset();
        return;
    }

    auto hid = system.ServiceManager()
                   .GetService<Service::HID::Module::Interface>("hid:USER")
                   ->GetModule();
    NativeLibrary::UpdateProgress("BootGame", 1, 1);

    s_update_hid = false;
    s_stop_running = false;
    s_is_running = true;
    while (!s_stop_running) {
        if (s_is_running) {
            result = system.RunLoop();
            if (result == Core::System::ResultStatus::ShutdownRequested) {
                // End emulation execution
                s_render_window->UpdateSurface(nullptr);
                break;
            } else if (result != Core::System::ResultStatus::Success) {
                s_stop_running = true;
                NativeLibrary::ShowMessageDialog(0, fmt::format("Error {}: {}",
                                                                static_cast<u32>(result),
                                                                system.GetStatusDetails()));
                break;
            }
            if (s_update_hid) {
                hid->UpdatePad();
                s_update_hid = false;
            }
        } else {
            // Ensure no audio bleeds out while game is paused
            const float volume = Settings::values.audio_volume;
            Settings::values.audio_volume = 0;

            std::unique_lock lock{s_running_mutex};
            s_running_cv.wait(lock, [] { return s_is_running || s_stop_running; });
            s_render_window->PollEvents();
            Settings::values.audio_volume = volume;
        }
    }

    // Shutdown the core emulation
    s_render_window->DoneCurrent();
    hid.reset();
    system.Shutdown();
    s_render_window.reset();
    s_is_running = false;
    resetSearchResults();
    Config::Save();
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

        std::vector<u8> dlc_data;
        update_loader->ReadIcon(dlc_data);
        if (Loader::IsValidSMDH(dlc_data)) {
            std::swap(smdh_data, dlc_data);
        }
    }();

    if (Loader::IsValidSMDH(smdh_data)) {
        memcpy(smdh, smdh_data.data(), sizeof(Loader::SMDH));
        return true;
    }

    return false;
}

static const GameInfo& GetGameInfo(const std::string& path) {
    u64 timestamp;
    if (IsSafPath(path)) {
        timestamp = NativeLibrary::SafLastModified(path);
    } else {
        timestamp = FileUtil::GetFileModificationTimestamp(path);
    }
    auto [iter, is_new] = s_app_dict.emplace(path, GameInfo{});
    auto& game = iter->second;
    if (!is_new) {
        if (timestamp == game.timestamp) {
            return game;
        }
    }

    auto app_loader = Loader::GetLoader(path);
    if (app_loader == nullptr) {
        return game;
    }

    app_loader->ReadProgramId(game.id);
    app_loader->ReadTitle(game.name);
    app_loader->IsExecutable(game.executable);
    game.strid = fmt::format("{:016X}", game.id);
    game.timestamp = timestamp;

    Loader::SMDH smdh;
    if (GetSMDHData(app_loader.get(), &smdh)) {
        game.icon = smdh.GetIcon(true);
        game.regions = smdh.GetRegions();
    }

    if (game.regions.empty()) {
        game.regions.push_back(Loader::SMDH::GameRegion::Japan);
    }

    return game;
}

static void UpdateDisplayRotation() {
    // display rotation
    NativeLibrary::current_display_rotation = NativeLibrary::GetDisplayRotation();

    // custom layout
    if (NativeLibrary::IsPortrait()) {
        Settings::values.layout_option = Config::Get(Config::LAYOUT_OPTION);
        Settings::values.large_screen_proportion = Config::Get(Config::LARGE_SCREEN_PROPORTION);
        Settings::values.large_screen_auto_fit = Config::Get(Config::LARGE_SCREEN_AUTO_FIT);
        Settings::values.large_screen_secondary_left =
            Config::Get(Config::LARGE_SCREEN_SECONDARY_LEFT);
        Settings::values.large_screen_secondary_top =
            Config::Get(Config::LARGE_SCREEN_SECONDARY_TOP);
        Settings::values.hybrid_side_column_left = Config::Get(Config::HYBRID_SIDE_COLUMN_LEFT);
        Settings::values.hybrid_secondary_top = Config::Get(Config::HYBRID_SECONDARY_TOP);
        Settings::values.hybrid_fit = Config::Get(Config::HYBRID_FIT);
        Settings::values.layout_margin_left = Config::Get(Config::LAYOUT_MARGIN_LEFT);
        Settings::values.layout_margin_top = Config::Get(Config::LAYOUT_MARGIN_TOP);
        Settings::values.layout_margin_right = Config::Get(Config::LAYOUT_MARGIN_RIGHT);
        Settings::values.layout_margin_bottom = Config::Get(Config::LAYOUT_MARGIN_BOTTOM);
        Settings::values.custom_top_left = Config::Get(Config::PORTRAIT_TOP_LEFT);
        Settings::values.custom_top_top = Config::Get(Config::PORTRAIT_TOP_TOP);
        Settings::values.custom_top_right = Config::Get(Config::PORTRAIT_TOP_RIGHT);
        Settings::values.custom_top_bottom = Config::Get(Config::PORTRAIT_TOP_BOTTOM);
        Settings::values.custom_bottom_left = Config::Get(Config::PORTRAIT_BOTTOM_LEFT);
        Settings::values.custom_bottom_top = Config::Get(Config::PORTRAIT_BOTTOM_TOP);
        Settings::values.custom_bottom_right = Config::Get(Config::PORTRAIT_BOTTOM_RIGHT);
        Settings::values.custom_bottom_bottom = Config::Get(Config::PORTRAIT_BOTTOM_BOTTOM);
        Settings::values.custom_layout = Config::Get(Config::PORTRAIT_CUSTOM_LAYOUT);
        Settings::values.swap_screen = Config::Get(Config::PORTRAIT_SWAP_SCREEN);
    } else {
        Settings::values.layout_option = Config::Get(Config::LANDSCAPE_LAYOUT_OPTION);
        Settings::values.large_screen_proportion =
            Config::Get(Config::LANDSCAPE_LARGE_SCREEN_PROPORTION);
        Settings::values.large_screen_auto_fit =
            Config::Get(Config::LANDSCAPE_LARGE_SCREEN_AUTO_FIT);
        Settings::values.large_screen_secondary_left =
            Config::Get(Config::LANDSCAPE_LARGE_SCREEN_SECONDARY_LEFT);
        Settings::values.large_screen_secondary_top =
            Config::Get(Config::LANDSCAPE_LARGE_SCREEN_SECONDARY_TOP);
        Settings::values.hybrid_side_column_left =
            Config::Get(Config::LANDSCAPE_HYBRID_SIDE_COLUMN_LEFT);
        Settings::values.hybrid_secondary_top =
            Config::Get(Config::LANDSCAPE_HYBRID_SECONDARY_TOP);
        Settings::values.hybrid_fit = Config::Get(Config::LANDSCAPE_HYBRID_FIT);
        Settings::values.layout_margin_left = Config::Get(Config::LANDSCAPE_LAYOUT_MARGIN_LEFT);
        Settings::values.layout_margin_top = Config::Get(Config::LANDSCAPE_LAYOUT_MARGIN_TOP);
        Settings::values.layout_margin_right = Config::Get(Config::LANDSCAPE_LAYOUT_MARGIN_RIGHT);
        Settings::values.layout_margin_bottom = Config::Get(Config::LANDSCAPE_LAYOUT_MARGIN_BOTTOM);
        Settings::values.custom_top_left = Config::Get(Config::LANDSCAPE_TOP_LEFT);
        Settings::values.custom_top_top = Config::Get(Config::LANDSCAPE_TOP_TOP);
        Settings::values.custom_top_right = Config::Get(Config::LANDSCAPE_TOP_RIGHT);
        Settings::values.custom_top_bottom = Config::Get(Config::LANDSCAPE_TOP_BOTTOM);
        Settings::values.custom_bottom_left = Config::Get(Config::LANDSCAPE_BOTTOM_LEFT);
        Settings::values.custom_bottom_top = Config::Get(Config::LANDSCAPE_BOTTOM_TOP);
        Settings::values.custom_bottom_right = Config::Get(Config::LANDSCAPE_BOTTOM_RIGHT);
        Settings::values.custom_bottom_bottom = Config::Get(Config::LANDSCAPE_BOTTOM_BOTTOM);
        Settings::values.custom_layout = Config::Get(Config::LANDSCAPE_CUSTOM_LAYOUT);
        Settings::values.swap_screen = Config::Get(Config::LANDSCAPE_SWAP_SCREEN);
    }
}

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_InstallCIA(JNIEnv* env, jclass obj,
                                                                   jobjectArray jPaths) {
    const std::vector<std::string> paths = JniHelper::Unwrap<std::vector<std::string>>(jPaths);
    Service::AM::InstallStatus status;
    for (const auto& path : paths) {
        status =
            Service::AM::InstallCIA(path, std::bind(NativeLibrary::UpdateProgress, path,
                                                    std::placeholders::_1, std::placeholders::_2));
        if (Service::AM::InstallStatus::Success != status) {
            NativeLibrary::UpdateProgress(path, static_cast<u32>(status), 0);
        } else {
            NativeLibrary::UpdateProgress(path, 0, 0);
        }
    }

    // clear all cached game infos
    s_app_dict.clear();
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_SetUserPath(JNIEnv* env, jclass obj,
                                                                    jstring jPath) {
    // saf
    FileUtil::RegisterIOFactory(std::make_unique<AndroidIOFactory>());

    // init user path
    std::string path = JniHelper::Unwrap(jPath);
    if (path[path.size() - 1] != '/')
        path.push_back('/');
    FileUtil::SetUserPath(path);

    // create user directory
    FileUtil::CreateFullPath(FileUtil::GetUserPath(FileUtil::UserPath::AmiiboDir));
    FileUtil::CreateFullPath(FileUtil::GetUserPath(FileUtil::UserPath::CacheDir));
    FileUtil::CreateFullPath(FileUtil::GetUserPath(FileUtil::UserPath::ConfigDir));
    FileUtil::CreateFullPath(FileUtil::GetUserPath(FileUtil::UserPath::LogDir));
    if (!FileUtil::Exists(FileUtil::GetUserPath(FileUtil::UserPath::ConfigDir) + "config-mmj.ini")) {
        Config::SaveDefault();
    }

    // Register frontend applets
    Frontend::RegisterDefaultApplets();
    s_keyboard = std::make_shared<AndroidKeyboard>();
    Core::System::GetInstance().RegisterSoftwareKeyboard(s_keyboard);
    Core::System::GetInstance().RegisterImageInterface(std::make_shared<PNGHandler>());
    Camera::RegisterFactory("image", std::make_unique<StillImageCameraFactory>());
    Camera::RegisterFactory("camera", std::make_unique<NDKCameraFactory>());

    // Register real Mic factory
    Frontend::Mic::RegisterRealMicFactory(std::make_unique<AndroidMicFactory>());

    //
    Core::System& system{Core::System::GetInstance()};
    system.nfc_scanning_callback = [](bool isScanning) -> void {
        NativeLibrary::HandleNFCScanning(isScanning);
    };
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_SetStatesPath(JNIEnv* env, jclass obj,
                                                                      jstring jPath) {
    const std::string path = JniHelper::Unwrap(jPath);
    FileUtil::SetUserPathOverride(FileUtil::UserPath::StatesDir, path);
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_SetSDMCPath(JNIEnv* env, jclass obj,
                                                                    jstring jPath) {
    const std::string path = JniHelper::Unwrap(jPath);
    FileUtil::SetUserPathOverride(FileUtil::UserPath::SDMCDir, path);
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
    UpdateDisplayRotation();
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_SurfaceDestroyed(JNIEnv* env, jclass obj) {
    ANativeWindow_release(s_surface);
    s_surface = nullptr;
    if (s_render_window) {
        s_render_window->UpdateSurface(s_surface);
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_WindowChanged(JNIEnv* env, jclass obj) {
    if (s_render_window) {
        UpdateDisplayRotation();
        s_render_window->UpdateWindow();
    }
}

JNIEXPORT jboolean JNICALL Java_org_citra_emu_NativeLibrary_DoFrame(JNIEnv* env, jclass obj) {
    // Keep the Choreographer loop alive until the render window exists. Before
    // NativeLibrary.Run loads config and BootGame creates EGLAndroid,
    // use_present_thread is still at its default value and would otherwise stop
    // the callback loop before shared-context presentation ever starts. This
    // must also win over the previous session's stop flag during an in-process
    // game restart, because the new Java callback can run before the new
    // NativeLibrary.Run thread clears s_stop_running.
    if (!s_render_window) {
        RecordDoFrameNoWindow();
        return JNI_TRUE;
    }
    if (s_stop_running) {
        RecordDoFrameStopped();
        return JNI_FALSE;
    }
    if (!Settings::values.use_present_thread) {
        RecordDoFrameDirectPath();
        return JNI_FALSE;
    }
    // Keep the Choreographer loop alive during boot and pause. The shared-context
    // presentation path still needs future callbacks even before emulation enters
    // the steady-state RunLoop.
    if (s_is_running && s_render_window) {
        const auto start_ns = PerfNowNs();
        s_render_window->TryPresenting();
        RecordDoFrameActivePresent(PerfNowNs() - start_ns);
        return JNI_TRUE;
    } else {
        RecordDoFramePaused();
        return JNI_TRUE;
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_HandleImage(JNIEnv* env, jclass obj,
                                                                    jintArray jpixels, jint width,
                                                                    jint height) {
    jint* pixels = env->GetIntArrayElements(jpixels, nullptr);
    NativeLibrary::ImageLoadedCallback(reinterpret_cast<u32*>(pixels), width, height);
    env->ReleaseIntArrayElements(jpixels, pixels, 0);
    env->DeleteLocalRef(jpixels);
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_SetBackgroundImage(JNIEnv* env, jclass obj,
                                                                           jintArray jpixels,
                                                                           jint width,
                                                                           jint height) {
    jint* pixels = env->GetIntArrayElements(jpixels, nullptr);
    VideoCore::SetBackgroundImage(reinterpret_cast<u32*>(pixels), width, height);
    env->ReleaseIntArrayElements(jpixels, pixels, 0);
    env->DeleteLocalRef(jpixels);
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_InputEvent(JNIEnv* env, jclass obj,
                                                                   jint button, jfloat value) {
    if (InputManager::GetInstance().InputEvent(button, value)) {
        s_update_hid = true;
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_TouchEvent(JNIEnv* env, jclass obj,
                                                                   jint action, jint x, jint y) {
    const int TOUCH_PRESSED = 1;
    const int TOUCH_MOVED = 2;
    const int TOUCH_RELEASED = 4;

    if (s_render_window) {
        if (action & TOUCH_PRESSED) {
            s_render_window->TouchPressed(x, y);
        } else if (action & TOUCH_MOVED) {
            s_render_window->TouchMoved(x, y);
        } else if (action & TOUCH_RELEASED) {
            s_render_window->TouchReleased();
        }
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_KeyboardEvent(JNIEnv* env, jclass obj,
                                                                      jint action, jstring jtext) {
    if (s_keyboard)
        s_keyboard->Accept(action, JniHelper::Unwrap(jtext));
}

JNIEXPORT jboolean JNICALL Java_org_citra_emu_NativeLibrary_KeyEvent(JNIEnv* env, jclass obj,
                                                                     jint button, jint action) {
    if (InputManager::GetInstance().KeyEvent(button, action)) {
        s_update_hid = true;
        return true;
    }
    return false;
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_MoveEvent(JNIEnv* env, jclass obj,
                                                                  jint axis, jfloat value) {
    if (InputManager::GetInstance().KeyEvent(axis, value)) {
        s_update_hid = true;
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_Run(JNIEnv* env, jclass obj,
                                                            jstring jFile) {
    NativeLibrary::Initialize(env);
    const std::string file_path = JniHelper::Unwrap(jFile);
    // Starting a fresh game inside the same process must clear the previous
    // session's stop flag before the first Choreographer callback runs, or the
    // callback loop will kill itself again before BootGame recreates EGL state.
    s_stop_running = false;
    s_is_running = false;
    // reload config
    Config::Clear();
    Config::Load();
    // system
    Settings::values.use_cpu_jit = Config::Get(Config::USE_CPU_JIT);
    Settings::values.is_new_3ds = Config::Get(Config::IS_NEW_3DS);
    Settings::values.use_virtual_sd = Config::Get(Config::USE_VIRTUAL_SD);
    Settings::values.region_value = Config::Get(Config::SYSTEM_REGION);
    Settings::values.shared_font_type = Config::Get(Config::SHARED_FONT_TYPE);
    // renderer
    Settings::values.use_direct_display = false;
    Settings::values.accurate_max_min = false;
    Settings::values.accurate_rcp_rsq = false;
    Settings::values.skip_load_buffer = false;
    Settings::values.merge_framebuffer = false;
    Settings::values.hw_gs_mode = Config::Get(Config::HW_GS_MODE);
    Settings::values.use_gles = Config::Get(Config::USE_GLES);
    Settings::values.show_fps = Config::Get(Config::SHOW_FPS);
    Settings::values.use_hw_renderer = true;
    Settings::values.use_hw_shader = Config::Get(Config::USE_HW_SHADER);
    Settings::values.use_shader_jit = Config::Get(Config::USE_SHADER_JIT);
    Settings::values.shaders_accurate_mul = Config::Get(Config::SHADERS_ACCURATE_MUL);
    Settings::values.use_frame_limit = Config::Get(Config::USE_FRAME_LIMIT);
    Settings::values.frame_limit = Config::Get(Config::FRAME_LIMIT);
    Settings::values.resolution_factor = Config::Get(Config::RESOLUTION_FACTOR);
    Settings::values.factor_3d = Config::Get(Config::FACTOR_3D);
    Settings::values.custom_textures = Config::Get(Config::CUSTOM_TEXTURES);
    Settings::values.pp_shader_name = Config::Get(Config::POST_PROCESSING_SHADER);
    Settings::values.remote_shader_host = Config::Get(Config::REMOTE_SHADER_HOST);
    // audio
    Settings::values.enable_dsp_lle = Config::Get(Config::ENABLE_DSP_LLE);
    Settings::values.enable_dsp_lle_multithread = Config::Get(Config::DSP_LLE_MULTITHREAD);
    Settings::values.audio_volume = Config::Get(Config::AUDIO_VOLUME);
    Settings::values.sink_id = GetAndroidAudioOutputSink(Config::Get(Config::AUDIO_OUTPUT_TYPE));
    Settings::values.audio_device_id = Config::Get(Config::AUDIO_OUTPUT_DEVICE);
    Settings::values.enable_audio_stretching = Config::Get(Config::AUDIO_STRETCHING);
    // mic
    Settings::values.mic_input_type = GetAndroidMicInputType(Config::Get(Config::AUDIO_INPUT_TYPE));
    Settings::values.mic_input_device = Config::Get(Config::AUDIO_INPUT_DEVICE);
    // debug
    Settings::values.shadow_rendering = Config::Get(Config::SHADOW_RENDERING);
    Settings::values.use_present_thread = Config::Get(Config::USE_PRESENT_THREAD);
    Settings::values.core_downcount_hack = Config::Get(Config::CPU_USAGE_LIMIT);
    u8 shaderType = Config::Get(Config::SHADER_TYPE);
    if (shaderType == 0) {
        Settings::values.use_separable_shader = false;
        Settings::values.use_shader_cache = false;
    } else if (shaderType == 1) {
        Settings::values.use_separable_shader = false;
        Settings::values.use_shader_cache = true;
    } else {
        Settings::values.use_separable_shader = true;
        Settings::values.use_shader_cache = true;
    }
    Settings::SetLLEModules(Config::Get(Config::LLE_MODULES));
    // custom layout
    Settings::values.swap_screen = false;
    if (NativeLibrary::IsPortrait()) {
        Settings::values.custom_layout = Config::Get(Config::PORTRAIT_CUSTOM_LAYOUT);
    } else {
        Settings::values.custom_layout = Config::Get(Config::LANDSCAPE_CUSTOM_LAYOUT);
    }
    UpdateDisplayRotation();
    //
    Settings::values.init_clock = Settings::InitClock::SystemTime;
    Settings::values.init_time = 946681277;
    Settings::values.core_ticks_hack = 0;
    Settings::values.skip_slow_draw = false;
    Settings::values.display_transfer_hack = false;
    Settings::values.skip_cpu_write = false;
    Settings::values.skip_texture_copy = false;
    Settings::values.disable_clip_coef = false;
    Settings::values.y2r_perform_hack = false;
    Settings::values.y2r_event_delay = false;
    Settings::values.force_texture_filter = 0;
    Settings::values.stream_buffer_hack = !Settings::values.use_present_thread;
    Settings::values.use_fence_sync = Config::Get(Config::USE_FENCE_SYNC);

    LogAndroidRunConfig(file_path);

    // profile
    InputManager::GetInstance().Init();

    // camera
    Settings::values.camera_name[Service::CAM::OuterRightCamera] = Config::Get(Config::CAMERA_DEVICE);
    Settings::values.camera_name[Service::CAM::InnerCamera] = Config::Get(Config::CAMERA_DEVICE);
    Settings::values.camera_name[Service::CAM::OuterLeftCamera] = Config::Get(Config::CAMERA_DEVICE);
    Settings::values.camera_config[Service::CAM::OuterRightCamera] = "0";
    Settings::values.camera_config[Service::CAM::InnerCamera] = "1";
    Settings::values.camera_config[Service::CAM::OuterLeftCamera] = "2";

    // play coin
    Service::PTM::Module::SetPlayCoins(99);

    // language
    std::shared_ptr<Service::CFG::Module> cfg = std::make_shared<Service::CFG::Module>();
    cfg->SetSystemLanguage(Config::Get(Config::SYSTEM_LANGUAGE));
    cfg->UpdateConfigNANDSavegame();

    // run
    BootGame(file_path);

    // shotdown
    InputManager::GetInstance().Shutdown();
    NativeLibrary::Shutdown(env);
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_ResumeEmulation(JNIEnv* env, jclass obj) {
    if (!s_stop_running) {
        s_is_running = true;
        s_running_cv.notify_all();
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_PauseEmulation(JNIEnv* env, jclass obj) {
    s_is_running = false;
    s_running_cv.notify_all();
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_nativeStopEmulation(JNIEnv* env,
                                                                            jclass obj) {
    s_stop_running = true;
    if (s_render_window) {
        s_render_window->StopPresenting();
    }
    s_running_cv.notify_all();
}

JNIEXPORT jintArray JNICALL Java_org_citra_emu_NativeLibrary_getRunningSettings(JNIEnv* env,
                                                                                jclass obj) {
    int i = 0;
    int settings[26];

    // get settings
    settings[i++] = Settings::values.core_ticks_hack > 0;
    settings[i++] = Settings::values.skip_slow_draw;
    settings[i++] = Settings::values.skip_cpu_write;
    settings[i++] = Settings::values.skip_texture_copy;
    settings[i++] = Settings::values.force_texture_filter;
    settings[i++] = Settings::values.hw_gs_mode;
    settings[i++] = Settings::values.shadow_rendering;
    settings[i++] = Settings::values.async_shader_compile;
    settings[i++] = Settings::values.use_compatible_mode;
    settings[i++] = std::min(std::max(Settings::values.resolution_factor - 1, 0), 3);
    settings[i++] = static_cast<int>(Settings::values.layout_option);
    settings[i++] = Settings::values.swap_screen;
    settings[i++] = Settings::values.large_screen_auto_fit;
    settings[i++] = Settings::values.large_screen_proportion;
    settings[i++] = Settings::values.large_screen_secondary_left;
    settings[i++] = Settings::values.large_screen_secondary_top;
    settings[i++] = static_cast<int>(Settings::values.hybrid_fit);
    settings[i++] = Settings::values.hybrid_side_column_left;
    settings[i++] = Settings::values.hybrid_secondary_top;
    settings[i++] = Settings::values.layout_margin_left;
    settings[i++] = Settings::values.layout_margin_top;
    settings[i++] = Settings::values.layout_margin_right;
    settings[i++] = Settings::values.layout_margin_bottom;
    settings[i++] = static_cast<int>(Settings::values.shaders_accurate_mul);
    settings[i++] = Settings::values.custom_layout;
    settings[i++] = Settings::values.frame_limit;

    jintArray array = env->NewIntArray(i);
    env->SetIntArrayRegion(array, 0, i, settings);
    return array;
}

JNIEXPORT jint JNICALL Java_org_citra_emu_NativeLibrary_getLargeScreenTopAutoFitProportion(
    JNIEnv* env, jclass obj) {
    if (!s_render_window) {
        return std::clamp(static_cast<int>(Settings::values.large_screen_proportion), 25, 100);
    }

    const auto& framebuffer_layout = s_render_window->GetFramebufferLayout();
    if (framebuffer_layout.width == 0 || framebuffer_layout.height == 0) {
        return std::clamp(static_cast<int>(Settings::values.large_screen_proportion), 25, 100);
    }

    return Layout::GetLargeFrameLayoutTopAndroidMaxFillProportion(
        framebuffer_layout.width, framebuffer_layout.height, Settings::values.swap_screen,
        Settings::values.layout_margin_left, Settings::values.layout_margin_top,
        Settings::values.layout_margin_right, Settings::values.layout_margin_bottom);
}

JNIEXPORT jint JNICALL
Java_org_citra_emu_NativeLibrary_getLargeScreenTopAutoFitProportionForDimensions(
    JNIEnv* env, jclass obj, jint width, jint height, jint margin_left, jint margin_top,
    jint margin_right, jint margin_bottom, jboolean swapped) {
    const auto safe_width = std::max(width, 0);
    const auto safe_height = std::max(height, 0);
    return Layout::GetLargeFrameLayoutTopAndroidMaxFillProportion(
        static_cast<u32>(safe_width), static_cast<u32>(safe_height), swapped,
        static_cast<u32>(std::max(margin_left, 0)), static_cast<u32>(std::max(margin_top, 0)),
        static_cast<u32>(std::max(margin_right, 0)),
        static_cast<u32>(std::max(margin_bottom, 0)));
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_setRunningSettings(JNIEnv* env, jclass obj,
                                                                           jintArray array) {
    int i = 0;
    jint* settings = env->GetIntArrayElements(array, nullptr);

    // FMV Hack
    Settings::SetFMVHack(settings[i++] > 0);

    // Skip Slow Draw
    Settings::values.skip_slow_draw = settings[i++] > 0;
    Config::Set(Config::SKIP_SLOW_DRAW, Settings::values.skip_slow_draw);

    // Skip CPU Write
    Settings::values.skip_cpu_write = settings[i++] > 0;
    Config::Set(Config::SKIP_CPU_WRITE, Settings::values.skip_cpu_write);

    // Skip Texture Copy
    Settings::values.skip_texture_copy = settings[i++] > 0;
    Config::Set(Config::SKIP_TEXTURE_COPY, Settings::values.skip_texture_copy);

    // Use Linear Filter
    Settings::values.force_texture_filter = settings[i++];
    Config::Set(Config::FORCE_TEXTURE_FILTER, Settings::values.force_texture_filter);

    // HW GS mode
    Settings::values.hw_gs_mode = static_cast<u8>(std::clamp(settings[i++], 0, 2));
    Config::Set(Config::HW_GS_MODE, Settings::values.hw_gs_mode);

    // Shadow Rendering
    Settings::values.shadow_rendering = settings[i++] > 0;
    Config::Set(Config::SHADOW_RENDERING, Settings::values.shadow_rendering);

    // Async Shader Compile
    Settings::values.async_shader_compile = settings[i++] > 0;
    Config::Set(Config::ASYNC_SHADER_COMPILE, Settings::values.async_shader_compile);

    // Compatible Mode
    Settings::values.use_compatible_mode = settings[i++] > 0;
    Config::Set(Config::USE_COMPATIBLE_MODE, Settings::values.use_compatible_mode);

    // Scale Factor
    Settings::values.resolution_factor = settings[i++] + 1;
    Config::Set(Config::RESOLUTION_FACTOR, Settings::values.resolution_factor);

    // Change Layout
    Settings::values.layout_option = static_cast<Settings::LayoutOption>(settings[i++]);
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::LAYOUT_OPTION, Settings::values.layout_option);
    } else {
        Config::Set(Config::LANDSCAPE_LAYOUT_OPTION, Settings::values.layout_option);
    }

    // Swap top and bottom screens for automatic layouts
    Settings::values.swap_screen = settings[i++] > 0;
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::PORTRAIT_SWAP_SCREEN, Settings::values.swap_screen);
    } else {
        Config::Set(Config::LANDSCAPE_SWAP_SCREEN, Settings::values.swap_screen);
    }

    Settings::values.large_screen_auto_fit = settings[i++] > 0;
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::LARGE_SCREEN_AUTO_FIT, Settings::values.large_screen_auto_fit);
    } else {
        Config::Set(Config::LANDSCAPE_LARGE_SCREEN_AUTO_FIT,
                    Settings::values.large_screen_auto_fit);
    }

    // Top-aligned large-screen secondary scale
    Settings::values.large_screen_proportion = std::clamp(settings[i++], 25, 100);
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::LARGE_SCREEN_PROPORTION, Settings::values.large_screen_proportion);
    } else {
        Config::Set(Config::LANDSCAPE_LARGE_SCREEN_PROPORTION,
                    Settings::values.large_screen_proportion);
    }

    Settings::values.large_screen_secondary_left = settings[i++] > 0;
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::LARGE_SCREEN_SECONDARY_LEFT,
                    Settings::values.large_screen_secondary_left);
    } else {
        Config::Set(Config::LANDSCAPE_LARGE_SCREEN_SECONDARY_LEFT,
                    Settings::values.large_screen_secondary_left);
    }

    Settings::values.large_screen_secondary_top = settings[i++] > 0;
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::LARGE_SCREEN_SECONDARY_TOP,
                    Settings::values.large_screen_secondary_top);
    } else {
        Config::Set(Config::LANDSCAPE_LARGE_SCREEN_SECONDARY_TOP,
                    Settings::values.large_screen_secondary_top);
    }

    Settings::values.hybrid_fit = static_cast<Settings::HybridFit>(std::clamp(settings[i++], 0, 1));
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::HYBRID_FIT, Settings::values.hybrid_fit);
    } else {
        Config::Set(Config::LANDSCAPE_HYBRID_FIT, Settings::values.hybrid_fit);
    }

    // Hybrid side column placement
    Settings::values.hybrid_side_column_left = settings[i++] > 0;
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::HYBRID_SIDE_COLUMN_LEFT, Settings::values.hybrid_side_column_left);
    } else {
        Config::Set(Config::LANDSCAPE_HYBRID_SIDE_COLUMN_LEFT,
                    Settings::values.hybrid_side_column_left);
    }

    // Hybrid stacked order
    Settings::values.hybrid_secondary_top = settings[i++] > 0;
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::HYBRID_SECONDARY_TOP, Settings::values.hybrid_secondary_top);
    } else {
        Config::Set(Config::LANDSCAPE_HYBRID_SECONDARY_TOP,
                    Settings::values.hybrid_secondary_top);
    }

    Settings::values.layout_margin_left =
        static_cast<u16>(std::clamp(settings[i++], 0, 1000));
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::LAYOUT_MARGIN_LEFT, Settings::values.layout_margin_left);
    } else {
        Config::Set(Config::LANDSCAPE_LAYOUT_MARGIN_LEFT,
                    Settings::values.layout_margin_left);
    }

    Settings::values.layout_margin_top =
        static_cast<u16>(std::clamp(settings[i++], 0, 1000));
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::LAYOUT_MARGIN_TOP, Settings::values.layout_margin_top);
    } else {
        Config::Set(Config::LANDSCAPE_LAYOUT_MARGIN_TOP,
                    Settings::values.layout_margin_top);
    }

    Settings::values.layout_margin_right =
        static_cast<u16>(std::clamp(settings[i++], 0, 1000));
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::LAYOUT_MARGIN_RIGHT, Settings::values.layout_margin_right);
    } else {
        Config::Set(Config::LANDSCAPE_LAYOUT_MARGIN_RIGHT,
                    Settings::values.layout_margin_right);
    }

    Settings::values.layout_margin_bottom =
        static_cast<u16>(std::clamp(settings[i++], 0, 1000));
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::LAYOUT_MARGIN_BOTTOM, Settings::values.layout_margin_bottom);
    } else {
        Config::Set(Config::LANDSCAPE_LAYOUT_MARGIN_BOTTOM,
                    Settings::values.layout_margin_bottom);
    }

    // Accurate Mul
    Settings::values.shaders_accurate_mul = static_cast<Settings::AccurateMul>(settings[i++]);
    Config::Set(Config::SHADERS_ACCURATE_MUL, Settings::values.shaders_accurate_mul);

    // Custom Layout
    Settings::values.custom_layout = settings[i++] > 0;
    if (NativeLibrary::IsPortrait()) {
        Config::Set(Config::PORTRAIT_CUSTOM_LAYOUT, Settings::values.custom_layout);
    } else {
        Config::Set(Config::LANDSCAPE_CUSTOM_LAYOUT, Settings::values.custom_layout);
    }

    // Frame Limit
    Settings::values.frame_limit = std::max(settings[i++], 1);

    VideoCore::SettingUpdate();

    env->ReleaseIntArrayElements(array, settings, 0);
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_setCustomLayout(JNIEnv* env, jclass obj,
                                                                        jboolean is_top_screen,
                                                                        jint left, jint top,
                                                                        jint right, jint bottom) {
    Settings::values.custom_layout = true;
    if (is_top_screen) {
        Settings::values.custom_top_left = left;
        Settings::values.custom_top_top = top;
        Settings::values.custom_top_right = right;
        Settings::values.custom_top_bottom = bottom;

        if (NativeLibrary::IsPortrait()) {
            Config::Set(Config::PORTRAIT_TOP_LEFT, left);
            Config::Set(Config::PORTRAIT_TOP_TOP, top);
            Config::Set(Config::PORTRAIT_TOP_RIGHT, right);
            Config::Set(Config::PORTRAIT_TOP_BOTTOM, bottom);
            Config::Set(Config::PORTRAIT_CUSTOM_LAYOUT, true);
        } else {
            Config::Set(Config::LANDSCAPE_TOP_LEFT, left);
            Config::Set(Config::LANDSCAPE_TOP_TOP, top);
            Config::Set(Config::LANDSCAPE_TOP_RIGHT, right);
            Config::Set(Config::LANDSCAPE_TOP_BOTTOM, bottom);
            Config::Set(Config::LANDSCAPE_CUSTOM_LAYOUT, true);
        }
    } else {
        Settings::values.custom_bottom_left = left;
        Settings::values.custom_bottom_top = top;
        Settings::values.custom_bottom_right = right;
        Settings::values.custom_bottom_bottom = bottom;

        if (NativeLibrary::IsPortrait()) {
            Config::Set(Config::PORTRAIT_BOTTOM_LEFT, left);
            Config::Set(Config::PORTRAIT_BOTTOM_TOP, top);
            Config::Set(Config::PORTRAIT_BOTTOM_RIGHT, right);
            Config::Set(Config::PORTRAIT_BOTTOM_BOTTOM, bottom);
            Config::Set(Config::PORTRAIT_CUSTOM_LAYOUT, true);
        } else {
            Config::Set(Config::LANDSCAPE_BOTTOM_LEFT, left);
            Config::Set(Config::LANDSCAPE_BOTTOM_TOP, top);
            Config::Set(Config::LANDSCAPE_BOTTOM_RIGHT, right);
            Config::Set(Config::LANDSCAPE_BOTTOM_BOTTOM, bottom);
            Config::Set(Config::LANDSCAPE_CUSTOM_LAYOUT, true);
        }
    }
    s_render_window->UpdateLayout();
}

JNIEXPORT jobject JNICALL Java_org_citra_emu_NativeLibrary_getCustomLayout(JNIEnv* env, jclass obj,
                                                                           jboolean is_top_screen) {
    int left, top, right, bottom;
    if (is_top_screen) {
        left = Settings::values.custom_top_left;
        top = Settings::values.custom_top_top;
        right = Settings::values.custom_top_right;
        bottom = Settings::values.custom_top_bottom;
    } else {
        left = Settings::values.custom_bottom_left;
        top = Settings::values.custom_bottom_top;
        right = Settings::values.custom_bottom_right;
        bottom = Settings::values.custom_bottom_bottom;
    }
    return JniHelper::RectObject(left, top, right, bottom);
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_Screenshot(JNIEnv* env, jclass obj,
                                                                   jobject listener) {
    if (VideoCore::g_screenshot_complete_callback) {
        return;
    }
    jobject listener_ref = env->NewGlobalRef(listener);
    VideoCore::g_screenshot_complete_callback = [listener_ref](u32 width, u32 height,
                                                               const std::vector<u32>& pixels) {
        JNIEnv* env = JniHelper::GetEnvForThread();
        jclass clazz = env->GetObjectClass(listener_ref);
        jmethodID method = env->GetMethodID(clazz, "OnScreenshotComplete", "(II[I)V");
        jintArray array = JniHelper::Wrap(pixels);
        env->CallVoidMethod(listener_ref, method, (jint)width, (jint)height, array);
        env->DeleteLocalRef(array);
        env->DeleteLocalRef(clazz);
        env->DeleteGlobalRef(listener_ref);
    };
}

JNIEXPORT jintArray JNICALL Java_org_citra_emu_NativeLibrary_searchMemory(
    JNIEnv* env, jclass obj, jint start_addr, jint stop_addr, jint value_type, jint search_type,
    jint scan_type, jint value) {
    return searchMemoryRegion(start_addr, stop_addr, value_type, search_type, scan_type, value);
}

JNIEXPORT jintArray JNICALL Java_org_citra_emu_NativeLibrary_getSearchResults(JNIEnv* env,
                                                                              jclass obj) {
    return getSearchResults();
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_resetSearchResults(JNIEnv* env,
                                                                           jclass obj) {
    resetSearchResults();
}

JNIEXPORT jintArray JNICALL Java_org_citra_emu_NativeLibrary_loadPageTable(JNIEnv* env,
                                                                           jclass obj) {
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

    return JniHelper::Wrap(pages);
}

JNIEXPORT jbyteArray JNICALL Java_org_citra_emu_NativeLibrary_loadPage(JNIEnv* env, jclass obj,
                                                                       jint index) {
    Core::System& system{Core::System::GetInstance()};
    auto p = system.Memory().GetCurrentPageTable()->pointers[index];
    if (p != nullptr) {
        return JniHelper::Wrap(p, Memory::PAGE_SIZE);
    } else {
        return nullptr;
    }
}

JNIEXPORT jint JNICALL Java_org_citra_emu_NativeLibrary_readMemory(JNIEnv* env, jclass obj,
                                                                   jint jAddr, jint valueType) {
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

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_writeMemory(JNIEnv* env, jclass obj,
                                                                    jint jAddr, jint valueType,
                                                                    jint value) {
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
    Core::System& system{Core::System::GetInstance()};
    Service::SM::ServiceManager& sm = system.ServiceManager();
    auto nfc = sm.GetService<Service::NFC::Module::Interface>("nfc:u");
    if (nfc) {
        nfc->LoadAmiibo(JniHelper::Unwrap(jPath));
    }
}

JNIEXPORT void JNICALL Java_org_citra_emu_NativeLibrary_ResetCamera(JNIEnv* env, jclass obj) {
    Settings::Apply();
}

JNIEXPORT jstring JNICALL Java_org_citra_emu_NativeLibrary_GetAppId(JNIEnv* env, jclass obj,
                                                                    jstring jPath) {
    return JniHelper::Wrap(GetGameInfo(JniHelper::Unwrap(jPath)).strid);
}

JNIEXPORT jstring JNICALL Java_org_citra_emu_NativeLibrary_GetAppTitle(JNIEnv* env, jclass obj,
                                                                       jstring jPath) {
    return JniHelper::Wrap(GetGameInfo(JniHelper::Unwrap(jPath)).name);
}

JNIEXPORT jbyteArray JNICALL Java_org_citra_emu_NativeLibrary_GetAppIcon(JNIEnv* env, jclass obj,
                                                                         jstring jPath) {
    auto& icon = GetGameInfo(JniHelper::Unwrap(jPath)).icon;
    return JniHelper::Wrap(reinterpret_cast<const u8*>(icon.data()), icon.size() * 2);
}

JNIEXPORT jint JNICALL Java_org_citra_emu_NativeLibrary_GetAppRegion(JNIEnv* env, jclass obj,
                                                                     jstring jPath) {
    auto& regions = GetGameInfo(JniHelper::Unwrap(jPath)).regions;
    return regions.size() == 7 ? 7 : static_cast<jint>(regions[0]);
}

JNIEXPORT jboolean JNICALL Java_org_citra_emu_NativeLibrary_IsAppExecutable(JNIEnv* env, jclass obj,
                                                                            jstring jPath) {
    return GetGameInfo(JniHelper::Unwrap(jPath)).executable;
}

JNIEXPORT jboolean JNICALL Java_org_citra_emu_NativeLibrary_IsAppVisible(JNIEnv* env, jclass obj,
                                                                         jstring jPath) {
    return GetGameInfo(JniHelper::Unwrap(jPath)).icon.size() > 0;
}

JNIEXPORT jstring JNICALL Java_org_citra_emu_utils_TranslateHelper_GoogleTranslateToken(
    JNIEnv* env, jclass clazz, jstring jText) {
    u32 ttk0 = 444129;
    u32 ttk1 = 803085091;
    u32 mask0[] = {'+', '-', 'a', '^', '+', 6};
    u32 mask1[] = {'+', '-', 3, '^', '+', 'b', '+', '-', 'f'};
    std::vector<u32> d;

    jclass stringClass = env->GetObjectClass(jText);
    jmethodID lengthID = env->GetMethodID(stringClass, "length", "()I");
    jmethodID codePointAtID = env->GetMethodID(stringClass, "codePointAt", "(I)I");
    int length = env->CallIntMethod(jText, lengthID);

    auto GoogleTranslateRL = [](u32 a, const u32* b, u32 size) -> u32 {
        for (u32 c = 0; c < size - 2; c += 3) {
            u32 d = b[c + 2];
            if (d >= 'a') {
                d -= 87;
            }
            d = '+' == b[c + 1] ? a >> d : a << d;
            a = '+' == b[c] ? a + d : a ^ d;
        }
        return a;
    };

    for (u32 f = 0; f < length; f++) {
        u32 g = env->CallIntMethod(jText, codePointAtID, (jint)f);
        if (128 > g) {
            d.push_back(g);
        } else {
            if (2048 > g) {
                d.push_back(g >> 6 | 192);
            } else {
                u32 gg = 0;
                if (f + 1 < length) {
                    gg = env->CallIntMethod(jText, codePointAtID, (jint)f + 1);
                }
                if (55296 == (g & 64512) && f + 1 < length && 56320 == (gg & 64512)) {
                    g = 65536 + ((g & 1023) << 10) + (gg & 1023);
                    d.push_back(g >> 18 | 240);
                    d.push_back(g >> 12 & 63 | 128);
                } else {
                    d.push_back(g >> 12 | 224);
                }
                d.push_back(g >> 6 & 63 | 128);
            }
            d.push_back(g & 63 | 128);
        }
    }
    u32 aa = ttk0;
    for (u32 e = 0; e < d.size(); e++) {
        aa += d[e];
        aa = GoogleTranslateRL(aa, mask0, 6);
    }
    aa = GoogleTranslateRL(aa, mask1, 9);
    aa ^= ttk1;
    if (0 > aa) {
        aa = (aa & 2147483647) + 2147483648;
    }
    aa %= 1000000;

    env->DeleteLocalRef(stringClass);
    return JniHelper::Wrap(fmt::format("{}.{}", aa, aa ^ ttk0));
}

JNIEXPORT jint JNICALL Java_org_citra_emu_utils_NetPlayManager_NetPlayCreateRoom(
    JNIEnv* env, jclass clazz, jstring ipaddress, jint port, jstring username) {
    return static_cast<jint>(
        NetPlayCreateRoom(JniHelper::Unwrap(ipaddress), port, JniHelper::Unwrap(username)));
}

JNIEXPORT jint JNICALL Java_org_citra_emu_utils_NetPlayManager_NetPlayJoinRoom(
    JNIEnv* env, jclass clazz, jstring ipaddress, jint port, jstring username) {
    return static_cast<jint>(
        NetPlayJoinRoom(JniHelper::Unwrap(ipaddress), port, JniHelper::Unwrap(username)));
}

JNIEXPORT jobjectArray JNICALL
Java_org_citra_emu_utils_NetPlayManager_NetPlayRoomInfo(JNIEnv* env, jclass clazz) {
    return JniHelper::Wrap(NetPlayRoomInfo());
}

JNIEXPORT jboolean JNICALL Java_org_citra_emu_utils_NetPlayManager_NetPlayIsJoined(JNIEnv* env,
                                                                                   jclass clazz) {
    return NetPlayIsJoined();
}

JNIEXPORT jboolean JNICALL
Java_org_citra_emu_utils_NetPlayManager_NetPlayIsHostedRoom(JNIEnv* env, jclass clazz) {
    return NetPlayIsHostedRoom();
}

JNIEXPORT void JNICALL Java_org_citra_emu_utils_NetPlayManager_NetPlaySendMessage(JNIEnv* env,
                                                                                  jclass clazz,
                                                                                  jstring msg) {
    NetPlaySendMessage(JniHelper::Unwrap(msg));
}

JNIEXPORT void JNICALL Java_org_citra_emu_utils_NetPlayManager_NetPlayKickUser(JNIEnv* env,
                                                                               jclass clazz,
                                                                               jstring username) {
    NetPlayKickUser(JniHelper::Unwrap(username));
}

JNIEXPORT void JNICALL Java_org_citra_emu_utils_NetPlayManager_NetPlayLeaveRoom(JNIEnv* env,
                                                                                jclass clazz) {
    NetPlayLeaveRoom();
}

JNIEXPORT jstring JNICALL
Java_org_citra_emu_utils_NetPlayManager_NetPlayGetConsoleId(JNIEnv* env, jclass clazz) {
    return JniHelper::Wrap(NetPlayGetConsoleId());
}

#ifdef __cplusplus
}
#endif
