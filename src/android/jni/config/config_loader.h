// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <memory>

namespace Config {
class ConfigLayerLoader;
} // namespace Config

namespace ConfigLoaders {
std::unique_ptr<Config::ConfigLayerLoader> GenerateConfigLoader();
} // namespace ConfigLoaders
