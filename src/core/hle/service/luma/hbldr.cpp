// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "core/core.h"
#include "core/hle/ipc_helpers.h"
#include "core/hle/service/luma/hbldr.h"

namespace Service::HBLDR {

HBLDR::HBLDR() : ServiceFramework("hb:ldr") {
    static const FunctionInfo functions[] = {
        {0x00010180, nullptr, "LoadProcess"},
        {0x00020002, nullptr, "SetTarget"},
        {0x00030002, nullptr, "SetArgv"},
        {0x00040002, nullptr, "PatchExHeaderInfo"},
        {0x00050000, nullptr, "DebugNextApplicationByForce"},
    };
    RegisterHandlers(functions);
}

void InstallInterfaces(Core::System& system) {
    std::make_shared<HBLDR>()->InstallAsNamedPort(system.Kernel());
}

} // namespace Service::NDM
