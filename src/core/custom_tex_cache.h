// Copyright 2019 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "common/common_types.h"

namespace Frontend {
class ImageInterface;
} // namespace Frontend

namespace Core {
struct CustomTexInfo {
    u64 hash;
    u32 width;
    u32 height;
    std::vector<u8> tex;
};

// This is to avoid parsing the filename multiple times
struct CustomTexPathInfo {
    std::string path;
    u64 hash;
};

// TODO: think of a better name for this class...
class CustomTexCache {
public:
    explicit CustomTexCache();
    ~CustomTexCache();

    bool IsTextureCached(u64 hash) const;
    const CustomTexInfo& LookupTexture(u64 hash) const;

    void AddTexturePath(u64 hash, const std::string& path);
    void FindCustomTextures(u64 program_id);
    void PreloadTextures();
    void LoadTexture(const CustomTexPathInfo& path_info);
    bool CustomTextureExists(u64 hash) const;
    const CustomTexPathInfo& LookupTexturePathInfo(u64 hash) const;

private:
    std::unordered_map<u64, CustomTexInfo> custom_textures;
    std::unordered_map<u64, CustomTexPathInfo> custom_texture_paths;
};
} // namespace Core
