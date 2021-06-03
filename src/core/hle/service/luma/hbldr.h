// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <array>
#include "core/hle/service/service.h"

namespace Core {
class System;
}

namespace Service::HBLDR {

class HBLDR final : public ServiceFramework<HBLDR> {
public:
    HBLDR();
};

void InstallInterfaces(Core::System& system);

} // namespace Service::NDM
