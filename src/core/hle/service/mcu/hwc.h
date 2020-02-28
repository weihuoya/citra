// Copyright 2019 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <memory>
#include "core/hle/service/mcu/mcu.h"

namespace Service::MCU {

class HWC final : public Module::Interface {
public:
    explicit HWC(std::shared_ptr<Module> module);
};

} // namespace Service::MCU
