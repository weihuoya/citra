// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <memory>
#include <utility>
#include "audio_core/dsp_interface.h"
#include "audio_core/hle/hle.h"
#include "audio_core/lle/lle.h"
#include "common/logging/log.h"
#include "common/texture.h"
#include "core/arm/arm_interface.h"
#if defined(ARCHITECTURE_x86_64) || defined(ARCHITECTURE_ARM64)
#include "core/arm/dynarmic/arm_dynarmic.h"
#endif
#include "core/arm/dyncom/arm_dyncom.h"
#include "core/cheats/cheats.h"
#include "core/core.h"
#include "core/core_timing.h"
#include "core/custom_tex_cache.h"
#include "core/file_sys/archive_source_sd_savedata.h"
#include "core/gdbstub/gdbstub.h"
#include "core/hle/kernel/client_port.h"
#include "core/hle/kernel/kernel.h"
#include "core/hle/kernel/process.h"
#include "core/hle/kernel/thread.h"
#include "core/hle/service/fs/archive.h"
#include "core/hle/service/service.h"
#include "core/hle/service/sm/sm.h"
#include "core/hw/hw.h"
#include "core/loader/loader.h"
#include "core/movie.h"
#include "core/rpc/rpc_server.h"
#include "core/settings.h"
#include "network/network.h"
#include "video_core/renderer_base.h"
#include "video_core/video_core.h"

namespace Core {

System System::s_instance;

System::ResultStatus System::RunLoop() {
    return cpu_cores.size() > 1 ? RunLoopMultiCores() : RunLoopOneCore();
}

System::ResultStatus System::RunLoopMultiCores() {
    // All cores should have executed the same amount of ticks. If this is not the case an event was
    // scheduled with a cycles_into_future smaller then the current downcount.
    // So we have to get those cores to the same global time first
    s64 max_delay = 0;
    ARM_Interface* current_core_to_execute = nullptr;
    for (auto& cpu_core : cpu_cores) {
        s64 delay = timing->GetGlobalTicks() - cpu_core->GetTimer().GetTicks();
        if (delay > 0) {
            cpu_core->GetTimer().Advance(delay);
            if (max_delay < delay) {
                max_delay = delay;
                current_core_to_execute = cpu_core.get();
            }
        }
    }

    if (max_delay > 0) {
        running_core = current_core_to_execute;
        kernel->SetRunningCPU(running_core);
        if (kernel->GetCurrentThreadManager().GetCurrentThread() == nullptr) {
            LOG_TRACE(Core_ARM11, "Core {} idling", current_core_to_execute->GetID());
            current_core_to_execute->GetTimer().Idle();
            PrepareReschedule();
        } else {
            current_core_to_execute->Run();
        }
    } else {
        // Now all cores are at the same global time. So we will run them one after the other
        // with a max slice that is the minimum of all max slices of all cores
        // TODO: Make special check for idle since we can easily revert the time of idle cores
        s64 max_slice = Timing::MAX_SLICE_LENGTH;
        for (const auto& cpu_core : cpu_cores) {
            max_slice = std::min(max_slice, cpu_core->GetTimer().GetMaxSliceLength());
        }
        for (auto& cpu_core : cpu_cores) {
            cpu_core->GetTimer().Advance(max_slice);
        }
        for (auto& cpu_core : cpu_cores) {
            running_core = cpu_core.get();
            kernel->SetRunningCPU(running_core);
            // If we don't have a currently active thread then don't execute instructions,
            // instead advance to the next event and try to yield to the next thread
            if (kernel->GetCurrentThreadManager().GetCurrentThread() == nullptr) {
                LOG_TRACE(Core_ARM11, "Core {} idling", cpu_core->GetID());
                cpu_core->GetTimer().Idle();
                PrepareReschedule();
            } else {
                cpu_core->Run();
            }
        }
        timing->AddToGlobalTicks(max_slice);
    }

    HW::Update();
    Reschedule();

    if (reset_requested.exchange(false)) {
        Reset();
    } else if (shutdown_requested.exchange(false)) {
        return ResultStatus::ShutdownRequested;
    }

    return status;
}

System::ResultStatus System::RunLoopOneCore() {
    // If we don't have a currently active thread then don't execute instructions,
    // instead advance to the next event and try to yield to the next thread
    if (kernel->GetCurrentThreadManager().GetCurrentThread() == nullptr) {
        running_core->GetTimer().Idle();
        running_core->GetTimer().Advance();
        PrepareReschedule();
    } else {
        running_core->GetTimer().Advance();
        running_core->Run();
    }

    HW::Update();
    Reschedule();

    if (reset_requested.exchange(false)) {
        Reset();
    } else if (shutdown_requested.exchange(false)) {
        return ResultStatus::ShutdownRequested;
    }

    return status;
}

System::ResultStatus System::SingleStep() {
    return status;
}

static void LoadOverrides(u64 title_id) {
    if (title_id == 0x0004000000068B00 || title_id == 0x0004000000061300 ||
        title_id == 0x000400000004A700) {
        // hack for Tales of the Abyss / Pac Man Party 3D
        Settings::values.display_transfer_hack = true;
        // crash on `g_state.geometry_pipeline.Reconfigure();`
        // state.regs.pipeline.gs_unit_exclusive_configuration = 0
        // state.regs.gs.max_input_attribute_index = 0
        Settings::values.skip_slow_draw = true;
        // may cause display issues
        Settings::values.texture_load_hack = false;
    } else if (title_id == 0x00040000001CCD00 || title_id == 0x00040000001B4500) {
        // The Alliance Alive
        Settings::SetFMVHack(true);
    } else if (title_id == 0x0004000000120900 || title_id == 0x0004000000164300) {
        // Lord of Magna: Maiden Heaven
        Settings::SetFMVHack(true);
    } else if (title_id == 0x000400000015CB00) {
        // New Atelier Rorona
        Settings::values.skip_slow_draw = true;
    } else if (title_id == 0x000400000018E900) {
        // My Hero Academia
        Settings::values.skip_slow_draw = true;
    } else if (title_id == 0x000400000016AD00) {
        // Dragon Quest Monsters Joker 3
        Settings::values.skip_slow_draw = true;
    } else if (title_id == 0x00040000001ACB00) {
        // Dragon Quest Monsters Joker 3 Professional
        Settings::values.skip_slow_draw = true;
    } else if (title_id == 0x000400000019E700 || title_id == 0x00040000001A5600) {
        // Armed Blue Gunvolt
        Settings::values.stream_buffer_hack = false;
    } else if (title_id == 0x000400000019B200 || title_id == 0x0004000000196A00 ||
               title_id == 0x00040000001A6E00) {
        // Armed Blue Gunvolt 2
        Settings::values.stream_buffer_hack = false;
    } else if (title_id == 0x0004000000149100) {
        // Gravity Falls - Legend of the Gnome Gemulets
        Settings::values.stream_buffer_hack = false;
    } else if (title_id == 0x0004000000196900 || title_id == 0x0004000000119A00 ||
               title_id == 0x000400000017C900 || title_id == 0x000400000017E100) {
        // Shovel Knight
        Settings::values.stream_buffer_hack = false;
    } else if (title_id == 0x00040000000D2800 || title_id == 0x0004000000065800 ||
               title_id == 0x0004000000766600) {
        // Rune Factory 4
        Settings::values.texture_load_hack = false;
    } else if (title_id == 0x0004000000055D00 || title_id == 0x0004000000055E00) {
        // Pokémon X/Y
        Settings::values.texture_load_hack = false;
    } else if (title_id == 0x000400000004B500) {
        // Monster Hunter 4
        Settings::values.texture_load_hack = true;
    } else if (title_id == 0x0004000000126300 || title_id == 0x000400000011D700 ||
               title_id == 0x0004000000126100 || title_id == 0x0004000000153200) {
        // Monster Hunter 4 Ultimate
        Settings::values.texture_load_hack = true;
    } else if (title_id == 0x0004000000155400) {
        // Monster Hunter X
        Settings::values.texture_load_hack = true;
    } else if (title_id == 0x000400000008FE00) {
        // 1001 Spikes [USA]
        Settings::values.stream_buffer_hack = false;
        Settings::SetFMVHack(true);
    } else if (title_id == 0x0004000000049100 || title_id == 0x0004000000030400 ||
               title_id == 0x0004000000049000) {
        // Star Fox 64
        Settings::values.disable_clip_coef = true;
    }

    const std::array<u64, 37> accurate_mul_ids = {
        0x0004000000134500, // Attack on Titan
        0x00040000000DF800, // Attack on Titan
        0x0004000000152000, // Attack on Titan
        0x00040000001AA200, // Attack on Titan
        0x0004000000054000, // Super Mario 3D Land
        0x0004000000053F00, // Super Mario 3D Land
        0x0004000000054100, // Super Mario 3D Land
        0x0004000000089F00, // Super Mario 3D Land
        0x0004000000089E00, // Super Mario 3D Land
        0x0004000000033400, // The Legend of Zelda: Ocarina of Time 3D
        0x0004000000033500, // The Legend of Zelda: Ocarina of Time 3D
        0x0004000000033600, // The Legend of Zelda: Ocarina of Time 3D
        0x000400000008F800, // The Legend of Zelda: Ocarina of Time 3D
        0x000400000008F900, // The Legend of Zelda: Ocarina of Time 3D
        0x0004000000132700, // Mario & Luigi: Paper Jam
        0x0004000000132600, // Mario & Luigi: Paper Jam
        0x0004000000132800, // Mario & Luigi: Paper Jam
        0x00040000001D1400, // Mario & Luigi: Bowsers Inside Story + Bowser Jrs Journey
        0x00040000001D1500, // Mario & Luigi: Bowsers Inside Story + Bowser Jrs Journey
        0x00040000001B8F00, // Mario & Luigi: Superstar Saga + Bowsers Minions
        0x00040000001B9000, // Mario & Luigi: Superstar Saga + Bowsers Minions
        0x0004000000194B00, // Mario & Luigi: Superstar Saga + Bowsers Minions
        0x00040000001CB000, // Captain Toad: Treasure Tracker
        0x00040000001CB200, // Captain Toad: Treasure Tracker
        0x00040000001CB100, // Captain Toad: Treasure Tracker
        0x00040000000EC200, // The Legend of Zelda: A Link Between Worlds
        0x00040000000EC300, // The Legend of Zelda: A Link Between Worlds
        0x00040000000EC400, // The Legend of Zelda: A Link Between Worlds
        0x000400000007AD00, // New Super Mario Bros. 2
        0x00040000000B8A00, // New Super Mario Bros. 2
        0x000400000007AE00, // New Super Mario Bros. 2
        0x000400000007AF00, // New Super Mario Bros. 2
        0x0004000000079600, // Jett Rocket II
        0x00040000000D0000, // Luigi's Mansion: Dark Moon
        0x0004000000076400, // Luigi's Mansion: Dark Moon
        0x0004000000055F00, // Luigi's Mansion: Dark Moon
        0x0004000000076500, // Luigi's Mansion: Dark Moon
    };
    for (auto id : accurate_mul_ids) {
        if (title_id == id) {
            Settings::values.shaders_accurate_mul = Settings::AccurateMul::FAST;
        }
    }

    const std::array<u64, 23> new3ds_game_ids = {
        0x000400000F700000, // Xenoblade Chronicles 3D [JPN]
        0x000400000F700100, // Xenoblade Chronicles 3D [USA]
        0x000400000F700200, // Xenoblade Chronicles 3D [EUR]
        0x000400000F70CC00, // Fire Emblem Warriors [USA]
        0x000400000F70CD00, // Fire Emblem Warriors [EUR]
        0x000400000F70C100, // Fire Emblem Warriors [JPN]
        0x000400000F700800, // The Binding of Isaac: Rebirth [USA]
        0x000400000F701700, // The Binding of Isaac: Rebirth [JPN]
        0x000400000F700900, // The Binding of Isaac: Rebirth [EUR]
        0x00040000000CCE00, // Donkey Kong Country Returns 3D [USA]
        0x00040000000CC000, // Donkey Kong Country Returns 3D [JPN]
        0x00040000000CCF00, // Donkey Kong Country Returns 3D [EUR]
        0x0004000000127500, // Sonic Boom: Shattered Crystal [USA]
        0x000400000014AE00, // Sonic Boom: Shattered Crystal [JPN]
        0x000400000012C200, // Sonic Boom: Shattered Crystal [EUR]
        0x0004000000161300, // Sonic Boom: Fire & Ice [USA]
        0x0004000000170700, // Sonic Boom: Fire & Ice [JPN]
        0x0004000000164700, // Sonic Boom: Fire & Ice [EUR]
        0x00040000000B3500, // Sonic & All-Stars Racing Transformed [USA]
        0x000400000008FC00, // Sonic & All-Stars Racing Transformed [EUR]
        0x00040000001B8700, // Minecraft [USA]
        0x000400000F707F00, // Hyperlight EX [USA]
        0x000400000008FE00, // 1001 Spikes [USA]
    };
    for (auto id : new3ds_game_ids) {
        if (title_id == id) {
            Settings::values.is_new_3ds = true;
        }
    }
}

System::ResultStatus System::Load(Frontend::EmuWindow& emu_window, const std::string& filepath) {
    app_loader = Loader::GetLoader(filepath);
    if (!app_loader) {
        LOG_CRITICAL(Core, "Failed to obtain loader for {}!", filepath);
        return ResultStatus::ErrorGetLoader;
    }
    std::pair<std::optional<u32>, Loader::ResultStatus> system_mode =
        app_loader->LoadKernelSystemMode();

    if (system_mode.second != Loader::ResultStatus::Success) {
        LOG_CRITICAL(Core, "Failed to determine system mode (Error {})!",
                     static_cast<int>(system_mode.second));

        switch (system_mode.second) {
        case Loader::ResultStatus::ErrorEncrypted:
            return ResultStatus::ErrorLoader_ErrorEncrypted;
        case Loader::ResultStatus::ErrorInvalidFormat:
            return ResultStatus::ErrorLoader_ErrorInvalidFormat;
        default:
            return ResultStatus::ErrorSystemMode;
        }
    }

    u64 title_id{0};
    if (app_loader->ReadProgramId(title_id) != Loader::ResultStatus::Success) {
        LOG_ERROR(Core, "Failed to find title id for ROM");
    }
    LoadOverrides(title_id);

    ASSERT(system_mode.first);
    auto n3ds_mode = app_loader->LoadKernelN3dsMode();
    ASSERT(n3ds_mode.first);
    ResultStatus init_result{Init(emu_window, *system_mode.first, *n3ds_mode.first)};
    if (init_result != ResultStatus::Success) {
        LOG_CRITICAL(Core, "Failed to initialize system (Error {})!",
                     static_cast<u32>(init_result));
        System::Shutdown();
        return init_result;
    }

    telemetry_session->AddInitialInfo(*app_loader);
    std::shared_ptr<Kernel::Process> process;
    const Loader::ResultStatus load_result{app_loader->Load(process)};
    kernel->SetCurrentProcess(process);
    if (Loader::ResultStatus::Success != load_result) {
        LOG_CRITICAL(Core, "Failed to load ROM (Error {})!", static_cast<u32>(load_result));
        System::Shutdown();

        switch (load_result) {
        case Loader::ResultStatus::ErrorEncrypted:
            return ResultStatus::ErrorLoader_ErrorEncrypted;
        case Loader::ResultStatus::ErrorInvalidFormat:
            return ResultStatus::ErrorLoader_ErrorInvalidFormat;
        default:
            return ResultStatus::ErrorLoader;
        }
    }
    cheat_engine = std::make_unique<Cheats::CheatEngine>(*this);
    perf_stats = std::make_unique<PerfStats>();

    custom_tex_cache = std::make_unique<Core::CustomTexCache>();

    if (Settings::values.custom_textures) {
        FileUtil::CreateFullPath(fmt::format(
            "{}textures/{:016X}/", FileUtil::GetUserPath(FileUtil::UserPath::LoadDir), title_id));
        custom_tex_cache->FindCustomTextures(title_id);
    }
    if (Settings::values.preload_textures) {
        custom_tex_cache->PreloadTextures(*GetImageInterface());
    }

    status = ResultStatus::Success;
    m_emu_window = &emu_window;
    m_filepath = filepath;

    // Reset counters and set time origin to current frame
    GetAndResetPerfStats();
    perf_stats->BeginSystemFrame();
    return status;
}

void System::PrepareReschedule() {
    running_core->PrepareReschedule();
    reschedule_pending = true;
}

PerfStats::Results System::GetAndResetPerfStats() {
    return perf_stats->GetAndResetStats(timing->GetGlobalTimeUs());
}

void System::Reschedule() {
    if (!reschedule_pending) {
        return;
    }

    reschedule_pending = false;
    for (const auto& core : cpu_cores) {
        LOG_TRACE(Core_ARM11, "Reschedule core {}", core->GetID());
        kernel->GetThreadManager(core->GetID()).Reschedule();
    }
}

System::ResultStatus System::Init(Frontend::EmuWindow& emu_window, u32 system_mode, u8 n3ds_mode) {
    LOG_DEBUG(HW_Memory, "initialized OK");

    u32 num_cores = 1;
    if (Settings::values.is_new_3ds) {
        num_cores = 4;
    }

    memory = std::make_unique<Memory::MemorySystem>();

    timing = std::make_unique<Timing>(num_cores);

    kernel = std::make_unique<Kernel::KernelSystem>(
        *memory, *timing, [this] { PrepareReschedule(); }, system_mode, num_cores, n3ds_mode);

    if (Settings::values.use_cpu_jit) {
#if defined(ARCHITECTURE_x86_64) || defined(ARCHITECTURE_ARM64)
        for (u32 i = 0; i < num_cores; ++i) {
            cpu_cores.push_back(
                std::make_shared<ARM_Dynarmic>(this, *memory, i, timing->GetTimer(i)));
        }
#else
        for (u32 i = 0; i < num_cores; ++i) {
            cpu_cores.push_back(
                std::make_shared<ARM_DynCom>(this, *memory, USER32MODE, i, timing->GetTimer(i)));
        }
        LOG_WARNING(Core, "CPU JIT requested, but Dynarmic not available");
#endif
    } else {
        for (u32 i = 0; i < num_cores; ++i) {
            cpu_cores.push_back(
                std::make_shared<ARM_DynCom>(this, *memory, USER32MODE, i, timing->GetTimer(i)));
        }
    }
    running_core = cpu_cores[0].get();

    kernel->SetCPUs(cpu_cores);
    kernel->SetRunningCPU(cpu_cores[0].get());

    if (Settings::values.enable_dsp_lle) {
        dsp_core = std::make_unique<AudioCore::DspLle>(*memory,
                                                       Settings::values.enable_dsp_lle_multithread);
    } else {
        dsp_core = std::make_unique<AudioCore::DspHle>(*memory);
    }

    memory->SetDSP(*dsp_core);

    dsp_core->SetSink(Settings::values.sink_id, Settings::values.audio_device_id);
    dsp_core->EnableStretching(Settings::values.enable_audio_stretching);

    telemetry_session = std::make_unique<Core::TelemetrySession>();

    rpc_server = std::make_unique<RPC::RPCServer>();

    service_manager = std::make_unique<Service::SM::ServiceManager>(*this);
    archive_manager = std::make_unique<Service::FS::ArchiveManager>(*this);

    HW::Init(*memory);
    Service::Init(*this);
    GDBStub::DeferStart();

    VideoCore::ResultStatus result = VideoCore::Init(emu_window, *memory);
    if (result != VideoCore::ResultStatus::Success) {
        switch (result) {
        case VideoCore::ResultStatus::ErrorGenericDrivers:
            return ResultStatus::ErrorVideoCore_ErrorGenericDrivers;
        case VideoCore::ResultStatus::ErrorBelowGL33:
            return ResultStatus::ErrorVideoCore_ErrorBelowGL33;
        default:
            return ResultStatus::ErrorVideoCore;
        }
    }

    return ResultStatus::Success;
}

VideoCore::RendererBase& System::Renderer() {
    return *VideoCore::g_renderer;
}

Service::SM::ServiceManager& System::ServiceManager() {
    return *service_manager;
}

const Service::SM::ServiceManager& System::ServiceManager() const {
    return *service_manager;
}

Service::FS::ArchiveManager& System::ArchiveManager() {
    return *archive_manager;
}

const Service::FS::ArchiveManager& System::ArchiveManager() const {
    return *archive_manager;
}

Kernel::KernelSystem& System::Kernel() {
    return *kernel;
}

const Kernel::KernelSystem& System::Kernel() const {
    return *kernel;
}

Timing& System::CoreTiming() {
    return *timing;
}

const Timing& System::CoreTiming() const {
    return *timing;
}

Memory::MemorySystem& System::Memory() {
    return *memory;
}

const Memory::MemorySystem& System::Memory() const {
    return *memory;
}

Cheats::CheatEngine& System::CheatEngine() {
    return *cheat_engine;
}

const Cheats::CheatEngine& System::CheatEngine() const {
    return *cheat_engine;
}

Core::CustomTexCache& System::CustomTexCache() {
    return *custom_tex_cache;
}

const Core::CustomTexCache& System::CustomTexCache() const {
    return *custom_tex_cache;
}

void System::RegisterMiiSelector(std::shared_ptr<Frontend::MiiSelector> mii_selector) {
    registered_mii_selector = std::move(mii_selector);
}

void System::RegisterSoftwareKeyboard(std::shared_ptr<Frontend::SoftwareKeyboard> swkbd) {
    registered_swkbd = std::move(swkbd);
}

void System::RegisterImageInterface(std::shared_ptr<Frontend::ImageInterface> image_interface) {
    registered_image_interface = std::move(image_interface);
}

void System::Shutdown() {
    // Shutdown emulation session
    GDBStub::Shutdown();
    VideoCore::Shutdown();
    HW::Shutdown();
    telemetry_session.reset();
    perf_stats.reset();
    rpc_server.reset();
    cheat_engine.reset();
    archive_manager.reset();
    service_manager.reset();
    cpu_cores.clear();
    dsp_core.reset();
    kernel.reset();
    timing.reset();
    memory.reset();
    app_loader.reset();
    custom_tex_cache.reset();

    running_core = nullptr;
    reschedule_pending = false;

    if (auto room_member = Network::GetRoomMember().lock()) {
        Network::GameInfo game_info{};
        room_member->SendGameInfo(game_info);
    }
}

void System::Reset() {
    // This is NOT a proper reset, but a temporary workaround by shutting down the system and
    // reloading.
    // TODO: Properly implement the reset

    Shutdown();
    // Reload the system with the same setting
    Load(*m_emu_window, m_filepath);
}

} // namespace Core
