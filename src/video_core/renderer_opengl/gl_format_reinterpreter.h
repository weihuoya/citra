// Copyright 2020 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <type_traits>
#include <vector>
#include <glad/glad.h>
#include "common/common_types.h"
#include "common/math_util.h"
#include "video_core/renderer_opengl/gl_resource_manager.h"
#include "video_core/renderer_opengl/gl_surface_params.h"

namespace OpenGL {

class RasterizerCacheOpenGL;

struct PixelFormatPair {
    const SurfaceParams::PixelFormat dst_format, src_format;
    struct less {
        using is_transparent = void;
        constexpr bool operator()(OpenGL::PixelFormatPair lhs, OpenGL::PixelFormatPair rhs) const {
            return std::tie(lhs.dst_format, lhs.src_format) <
                   std::tie(rhs.dst_format, rhs.src_format);
        }
        constexpr bool operator()(OpenGL::SurfaceParams::PixelFormat lhs,
                                  OpenGL::PixelFormatPair rhs) const {
            return lhs < rhs.dst_format;
        }
        constexpr bool operator()(OpenGL::PixelFormatPair lhs,
                                  OpenGL::SurfaceParams::PixelFormat rhs) const {
            return lhs.dst_format < rhs;
        }
    };
};

class FormatReinterpreterBase {
public:
    virtual ~FormatReinterpreterBase() = default;

    virtual void Reinterpret(GLuint src_tex, const Common::Rectangle<u32>& src_rect,
                             GLuint read_fb_handle, GLuint dst_tex,
                             const Common::Rectangle<u32>& dst_rect, GLuint draw_fb_handle) = 0;
};

class FormatReinterpreterOpenGL : NonCopyable {
public:
    explicit FormatReinterpreterOpenGL();
    ~FormatReinterpreterOpenGL();

    struct Reinterpreter {
        Reinterpreter(const SurfaceParams::PixelFormat dst_format,
                      const SurfaceParams::PixelFormat src_format,
                      std::unique_ptr<FormatReinterpreterBase> reinterpreter)
            : dst_format(dst_format), src_format(src_format),
              reinterpreter(std::move(reinterpreter)) {}
        const SurfaceParams::PixelFormat dst_format;
        const SurfaceParams::PixelFormat src_format;
        std::unique_ptr<FormatReinterpreterBase> reinterpreter;
    };

    const std::vector<Reinterpreter>& GetReinterpreters();

private:
    std::vector<Reinterpreter> reinterpreters;
};

} // namespace OpenGL
