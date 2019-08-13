// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "config/config_loader.h"

#include <cstring>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <variant>

#include "common/common_types.h"
#include "common/file_util.h"
#include "config/config.h"
#include "config/ini_file.h"

namespace ConfigLoaders {

static const char * CONFIG_FILE = "config-mmj.ini";

// INI layer configuration loader
class BaseConfigLayerLoader final : public Config::ConfigLayerLoader {
public:
    BaseConfigLayerLoader() : ConfigLayerLoader() {}
    void Load(Config::Layer* layer) override {
        IniFile ini;
        if (!ini.Load(FileUtil::GetUserPath(FileUtil::UserPath::ConfigDir) + CONFIG_FILE))
            return;

        const std::list<IniFile::Section>& system_sections = ini.GetSections();
        for (const auto& section : system_sections) {
            const std::string& section_name = section.GetName();
            const IniFile::Section::SectionMap& section_map = section.GetValues();
            for (const auto& value : section_map) {
                layer->Set({section_name, value.first}, value.second);
            }
        }
    }

    void Save(Config::Layer* layer) override {
        IniFile ini;
        std::string ini_path = FileUtil::GetUserPath(FileUtil::UserPath::ConfigDir) + CONFIG_FILE;
        ini.Load(ini_path);
        for (const auto& config : layer->GetLayerMap()) {
            const Config::ConfigLocation& location = config.first;
            const std::optional<std::string>& value = config.second;
            if (value) {
                IniFile::Section* ini_section = ini.GetOrCreateSection(location.section);
                ini_section->Set(location.key, *value);
            } else {
                ini.DeleteKey(location.section, location.key);
            }
        }
        ini.Save(ini_path);
    }
};

// Loader generation
std::unique_ptr<Config::ConfigLayerLoader> GenerateConfigLoader() {
    return std::make_unique<BaseConfigLayerLoader>();
}

} // namespace ConfigLoaders
