// Copyright 2019 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/common_types.h"
#include "core/core.h"
#include "core/hle/service/mcu/hwc.h"
#include "core/hle/service/mcu/mcu.h"

namespace Service::MCU {

Module::Interface::Interface(std::shared_ptr<Module> module, const char* name, u32 max_sessions)
    : ServiceFramework(name, max_sessions), module(std::move(module)) {}

void InstallInterfaces(Core::System& system) {
    SM::ServiceManager& service_manager = system.ServiceManager();
    std::shared_ptr<Module> module = std::make_shared<Module>();
    std::make_shared<HWC>(module)->InstallAsService(service_manager);
}

} // namespace Service::MCU
