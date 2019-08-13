// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "config/config_info.h"
#include "config/layer.h"

namespace Config {

Layer& GetLayer();

// Explicit load and save of layers
inline void Load() {
    GetLayer().Load();
}

inline void Save() {
    GetLayer().Save();
}

void SaveDefault();

template <typename T>
inline T Get(const ConfigInfo<T>& info) {
    return GetLayer().Get(info);
}

template <typename T>
inline void Set(const ConfigInfo<T>& info, const std::common_type_t<T>& value) {
    GetLayer().Set(info, value);
}

} // namespace Config
