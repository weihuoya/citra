// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <algorithm>
#include <cstring>
#include <map>

#include "config/config.h"
#include "config/layer.h"

namespace Config {
ConfigLayerLoader::ConfigLayerLoader() = default;
ConfigLayerLoader::~ConfigLayerLoader() = default;

Layer::Layer(std::unique_ptr<ConfigLayerLoader> loader) : m_loader(std::move(loader)) {}

Layer::~Layer() {
    Save();
}

bool Layer::Exists(const ConfigLocation& location) const {
    const auto iter = m_map.find(location);
    return iter != m_map.end() && iter->second.has_value();
}

bool Layer::DeleteKey(const ConfigLocation& location) {
    m_is_dirty = true;
    bool had_value = m_map[location].has_value();
    m_map[location].reset();
    return had_value;
}

void Layer::DeleteAllKeys() {
    m_is_dirty = true;
    for (auto& pair : m_map) {
        pair.second.reset();
    }
}

Section Layer::GetSection(const std::string& section) {
    return Section{m_map.lower_bound(ConfigLocation{section, ""}),
                   m_map.lower_bound(ConfigLocation{section + '\001', ""})};
}

ConstSection Layer::GetSection(const std::string& section) const {
    return ConstSection{m_map.lower_bound(ConfigLocation{section, ""}),
                        m_map.lower_bound(ConfigLocation{section + '\001', ""})};
}

void Layer::Load() {
    if (m_loader)
        m_loader->Load(this);
    m_is_dirty = false;
}

void Layer::Save() {
    if (!m_loader || !m_is_dirty)
        return;

    m_loader->Save(this);
    m_is_dirty = false;
}

void Layer::Clear() {
    m_map.clear();
}

const LayerMap& Layer::GetLayerMap() const {
    return m_map;
}
} // namespace Config
