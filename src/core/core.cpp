// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <memory>
#include <utility>
#include "audio_core/dsp_interface.h"
#include "audio_core/hle/hle.h"
#include "audio_core/lle/lle.h"
#include "common/logging/log.h"
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
    return Settings::values.is_new_3ds ? RunLoopMultiCores() : RunLoopSingleCore();
}

System::ResultStatus System::RunLoopMultiCores() {
    // All cores should have executed the same amount of ticks. If this is not the case an event was
    // scheduled with a cycles_into_future smaller then the current downcount.
    // So we have to get those cores to the same global time first
    s64 max_delay = 100;
    ARM_Interface* current_core_to_execute = nullptr;
    for (auto& cpu_core : cpu_cores) {
        s64 delay = timing->GetDelayTicks(cpu_core->GetID());
        if (delay > 0) {
            kernel->Advance(cpu_core.get(), max_delay);
            if (delay > max_delay) {
                max_delay = delay;
                current_core_to_execute = cpu_core.get();
            }
        }
    }

    if (max_delay > 100) {
        kernel->Run(current_core_to_execute);
    } else {
        // Now all cores are at the same global time. So we will run them one after the other
        // with a max slice that is the minimum of all max slices of all cores
        // TODO: Make special check for idle since we can easily revert the time of idle cores
        s64 max_slice = timing->GetMaxSliceLength();
        for (auto& cpu_core : cpu_cores) {
            kernel->Advance(cpu_core.get(), max_slice);
        }
        timing->AddToGlobalTicks(max_slice);
        for (auto& cpu_core : cpu_cores) {
            kernel->Run(cpu_core.get());
        }
    }

    kernel->RescheduleMultiCores();

    if (reset_requested.exchange(false)) {
        Reset();
    } else if (shutdown_requested.exchange(false)) {
        return ResultStatus::ShutdownRequested;
    }

    return status;
}

System::ResultStatus System::RunLoopSingleCore() {
    kernel->Advance(cpu_cores[0].get(), Timing::MAX_SLICE_LENGTH);
    kernel->Run(cpu_cores[0].get());
    kernel->RescheduleSingleCore();

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
        title_id == 0x000400000004A700 || title_id == 0x000400000005D700) {
        // hack for Tales of the Abyss / Pac Man Party 3D
        Settings::values.display_transfer_hack = true;
        // crash on `g_state.geometry_pipeline.Reconfigure();`
        // state.regs.pipeline.gs_unit_exclusive_configuration = 0
        // state.regs.gs.max_input_attribute_index = 0
        Settings::values.skip_slow_draw = true;
    } else if (title_id == 0x00040000001CCD00 || title_id == 0x00040000001B4500) {
        // The Alliance Alive
        Settings::values.core_downcount_hack = true;
    } else if (title_id == 0x0004000000120900 || title_id == 0x0004000000164300) {
        // Lord of Magna: Maiden Heaven
        Settings::values.core_downcount_hack = true;
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
    } else if (title_id == 0x000400000008FE00) {
        // 1001 Spikes
        Settings::values.stream_buffer_hack = false;
        Settings::values.core_downcount_hack = true;
    } else if (title_id == 0x0004000000049100 || title_id == 0x0004000000030400 ||
               title_id == 0x0004000000049000) {
        // Star Fox 64
        Settings::values.disable_clip_coef = true;
    } else if (title_id == 0x0004000000187E00 || title_id == 0x0004000000169A00) {
        // Picross 2
        Settings::values.disable_clip_coef = true;
    } else if (title_id == 0x00040000000DCA00 || title_id == 0x00040000000F4000) {
        // Danball Senki W Chou Custom, Danball Senki WARS
        Settings::values.y2r_perform_hack = true;
    } else if (title_id == 0x000400000008B400 || title_id == 0x0004000000030600 ||
               title_id == 0x0004000000030800 || title_id == 0x0004000000030700) {
        // Mario Kart 7
        Settings::values.skip_texture_copy = true;
    }

    const std::array<u64, 7> cpu_limit_ids = {
        0x000400000007C700, // Mario Tennis Open
        0x000400000007C800, // Mario Tennis Open
        0x0004000000064D00, // Mario Tennis Open
        0x00040000000B9100, // Mario Tennis Open
        0x00040000000DCD00, // Mario Golf: World Tour
        0x00040000000A5300, // Mario Golf: World Tour
        0x00040000000DCE00, // Mario Golf: World Tour
    };
    for (auto id : cpu_limit_ids) {
        if (title_id == id) {
            Settings::values.core_downcount_hack = true;
            break;
        }
    }

    const std::array<u64, 10> linear_ids = {
        0x00040000001AA200, // Attack On Titan 2
        0x0004000000134500, // Attack On Titan 1 CHAIN
        0x0004000000152000, // Attack On Titan 1 CHAIN
        0x0004000000134500, // Attack On Titan 1 CHAIN
        0x00040000000DF800, // Attack On Titan 1
    };
    for (auto id : linear_ids) {
        if (title_id == id) {
            Settings::values.use_linear_filter = true;
            break;
        }
    }

    const std::array<u64, 12> fifa_ids = {
        0x0004000000044700, // FIFA 12
        0x0004000000047A00, // FIFA 12
        0x0004000000044800, // FIFA 12
        0x00040000000A2B00, // FIFA 13
        0x00040000000A2900, // FIFA 13
        0x00040000000A3000, // FIFA 13
        0x00040000000E7900, // FIFA 14
        0x00040000000DEA00, // FIFA 14
        0x00040000000E7A00, // FIFA 14
        0x000400000013C700, // FIFA 15
        0x000400000013CA00, // FIFA 15
        0x000400000013CB00, // FIFA 15
    };
    for (auto id : fifa_ids) {
        if (title_id == id) {
            Settings::values.y2r_event_delay = true;
            break;
        }
    }

    const std::array<u64, 50> accurate_mul_ids = {
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
        0x00040000001CA900, // Mario & Luigi: Bowsers Inside Story + Bowser Jrs Journey
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
        0x0004000000112600, // Cut the Rope
        0x0004000000116700, // Cut the Rope
        0x00040000000D0000, // Luigi's Mansion: Dark Moon
        0x0004000000076400, // Luigi's Mansion: Dark Moon
        0x0004000000055F00, // Luigi's Mansion: Dark Moon
        0x0004000000076500, // Luigi's Mansion: Dark Moon
        0x00040000000AFC00, // Digimon World Re:Digitize Decode
        0x0004000000125600, // The Legend of Zelda: Majoras Mask 3D
        0x0004000000125500, // The Legend of Zelda: Majoras Mask 3D
        0x00040000000D6E00, // The Legend of Zelda: Majoras Mask 3D
        0x0004000000154700, // Lego City Undercover
        0x00040000000AD600, // Lego City Undercover
        0x00040000000AD500, // Lego City Undercover
        0x00040000001D1800, // Luigi's Mansion
        0x00040000001D1A00, // Luigi's Mansion
        0x00040000001D1900, // Luigi's Mansion
    };
    for (auto id : accurate_mul_ids) {
        if (title_id == id) {
            Settings::values.shaders_accurate_mul = Settings::AccurateMul::FAST;
            break;
        }
    }

    const std::array<u64, 30> new3ds_game_ids = {
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
        0x000400000007C700, // Mario Tennis Open
        0x000400000007C800, // Mario Tennis Open
        0x0004000000064D00, // Mario Tennis Open
        0x00040000000B9100, // Mario Tennis Open
        0x00040000000DCD00, // Mario Golf: World Tour
        0x00040000000A5300, // Mario Golf: World Tour
        0x00040000000DCE00, // Mario Golf: World Tour
    };
    for (auto id : new3ds_game_ids) {
        if (title_id == id) {
            Settings::values.is_new_3ds = true;
            break;
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
    ResultStatus init_result = Init(emu_window, *system_mode.first, *n3ds_mode.first);
    if (init_result != ResultStatus::Success) {
        LOG_CRITICAL(Core, "Failed to initialize system (Error {})!",
                     static_cast<u32>(init_result));
        System::Shutdown();
        return init_result;
    }

    std::shared_ptr<Kernel::Process> process;
    const Loader::ResultStatus load_result = app_loader->Load(process);
    kernel->Initialize(process, cpu_cores[0].get());
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
        if (Settings::values.preload_textures) {
            custom_tex_cache->PreloadTextures();
        }
    }

    status = ResultStatus::Success;
    m_emu_window = &emu_window;
    m_filepath = filepath;

    // Reset counters and set time origin to current frame
    GetAndResetPerfStats();
    perf_stats->BeginSystemFrame();
    return status;
}

PerfStats::Results System::GetAndResetPerfStats() {
    return perf_stats->GetAndResetStats(timing->GetGlobalTimeUs());
}

System::ResultStatus System::Init(Frontend::EmuWindow& emu_window, u32 system_mode, u8 n3ds_mode) {
    LOG_DEBUG(HW_Memory, "initialized OK");

    memory = std::make_unique<Memory::MemorySystem>();
    timing = std::make_unique<Timing>(Settings::values.cpu_clock_percentage);
    kernel = std::make_unique<Kernel::KernelSystem>(*memory, *timing, system_mode, n3ds_mode);

    if (Settings::values.use_cpu_jit) {
#if defined(ARCHITECTURE_x86_64) || defined(ARCHITECTURE_ARM64)
        for (u32 i = 0; i < 4; ++i) {
            cpu_cores[i] = std::make_shared<ARM_Dynarmic>(this, i, timing->GetTimer(i));
            kernel->GetThreadManager(i).SetCPU(cpu_cores[i].get());
        }
#else
        for (u32 i = 0; i < 4; ++i) {
            cpu_cores[i] = std::make_shared<ARM_DynCom>(this, i, timing->GetTimer(i));
            kernel->GetThreadManager(i).SetCPU(cpu_cores[i].get());
        }
        LOG_WARNING(Core, "CPU JIT requested, but Dynarmic not available");
#endif
    } else {
        for (u32 i = 0; i < 4; ++i) {
            cpu_cores[i] = std::make_shared<ARM_DynCom>(this, i, timing->GetTimer(i));
            kernel->GetThreadManager(i).SetCPU(cpu_cores[i].get());
        }
    }

    if (Settings::values.core_downcount_hack) {
        SetCpuUsageLimit(true);
    }

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

void System::SetCpuUsageLimit(bool enabled) {
    if (enabled) {
        u32 hacks[4] = {1, 4, 2, 2};
        for (u32 i = 0; i < 4; ++i) {
            timing->GetTimer(i)->SetDowncountHack(hacks[i]);
        }
    } else {
        for (u32 i = 0; i < 4; ++i) {
            timing->GetTimer(i)->SetDowncountHack(0);
        }
    }
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
    cpu_cores = {};
    dsp_core.reset();
    kernel.reset();
    timing.reset();
    memory.reset();
    app_loader.reset();
    custom_tex_cache.reset();

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
