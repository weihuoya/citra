// Copyright 2019 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <fmt/format.h>
#include "common/file_util.h"
#include "core.h"
#include "core/custom_tex_cache.h"

namespace Core {

inline u32 BGRA8888ToRGBA8888(u32 src) {
    u32 b = src & 0xFF;
    u32 g = (src >> 8) & 0xFF;
    u32 r = (src >> 16) & 0xFF;
    u32 a = (src >> 24) & 0xFF;
    return (a << 24) | (b << 16) | (g << 8) | r;
}

static void FlipCustomTexture(u32* pixels, u32 width, u32 height) {
    for (u32 y = 0; y < height / 2; ++y) {
        for (u32 x = 0; x < width; ++x) {
            u32 from = width * y + x;
            u32 to = width * (height - y - 1) + x;
            if (from == to) {
                u32 from_value = BGRA8888ToRGBA8888(pixels[from]);
                pixels[from] = from_value;
                return;
            } else if (from < to) {
                u32 from_value = BGRA8888ToRGBA8888(pixels[from]);
                u32 to_value = BGRA8888ToRGBA8888(pixels[to]);
                pixels[from] = to_value;
                pixels[to] = from_value;
            } else {
                return;
            }
        }
    }
}

CustomTexCache::CustomTexCache() = default;

CustomTexCache::~CustomTexCache() = default;

bool CustomTexCache::IsTextureCached(u64 hash) const {
    return custom_textures.find(hash) != custom_textures.end();
}

const CustomTexInfo& CustomTexCache::LookupTexture(u64 hash) const {
    return custom_textures.at(hash);
}

void CustomTexCache::AddTexturePath(u64 hash, const std::string& path) {
    if (custom_texture_paths.count(hash))
        LOG_ERROR(Core, "Textures {} and {} conflict!", custom_texture_paths[hash].path, path);
    else
        custom_texture_paths[hash] = {path, hash};
}

void CustomTexCache::FindCustomTextures(u64 program_id) {
    // Custom textures are currently stored as
    // [TitleID]/tex1_[width]x[height]_[64-bit hash]_[format].png

    const std::string load_path = fmt::format(
        "{}textures/{:016X}", FileUtil::GetUserPath(FileUtil::UserPath::LoadDir), program_id);

    if (FileUtil::Exists(load_path)) {
        FileUtil::FSTEntry texture_dir;
        std::vector<FileUtil::FSTEntry> textures;
        // 64 nested folders should be plenty for most cases
        FileUtil::ScanDirectoryTree(load_path, texture_dir, 64);
        FileUtil::GetAllFilesFromNestedEntries(texture_dir, textures);

        for (const auto& file : textures) {
            if (file.isDirectory)
                continue;
            if (file.virtualName.substr(0, 5) != "tex1_")
                continue;

            u32 width;
            u32 height;
            u64 hash;
            u32 format; // unused
            // TODO: more modern way of doing this
            if (std::sscanf(file.virtualName.c_str(), "tex1_%ux%u_%lX_%u.", &width, &height, &hash, &format) == 4) {
                AddTexturePath(hash, file.physicalName);
            }
        }
    }
}

void CustomTexCache::PreloadTextures() {
    for (const auto& path : custom_texture_paths) {
        LoadTexture(path.second);
    }
}

void CustomTexCache::LoadTexture(const CustomTexPathInfo& path_info) {
    const auto& image_interface = Core::System::GetInstance().GetImageInterface();
    auto& tex_info = custom_textures[path_info.hash];
    if (image_interface->DecodePNG(tex_info.tex, tex_info.width, tex_info.height, path_info.path)) {
        // Make sure the texture size is a power of 2
        if (!(tex_info.width & (tex_info.width - 1)) &&
            !(tex_info.height & (tex_info.height - 1))) {
            LOG_DEBUG(Render_OpenGL, "Loaded custom texture from {}", path_info.path);
            FlipCustomTexture(reinterpret_cast<u32*>(tex_info.tex.data()), tex_info.width, tex_info.height);
            tex_info.hash = path_info.hash;
        } else {
            LOG_ERROR(Render_OpenGL, "Texture {} size is not a power of 2", path_info.path);
            custom_textures.erase(path_info.hash);
        }
    } else {
        LOG_ERROR(Render_OpenGL, "Failed to load custom texture {}", path_info.path);
    }
}

bool CustomTexCache::CustomTextureExists(u64 hash) const {
    return custom_texture_paths.count(hash);
}

const CustomTexPathInfo& CustomTexCache::LookupTexturePathInfo(u64 hash) const {
    return custom_texture_paths.at(hash);
}

} // namespace Core
