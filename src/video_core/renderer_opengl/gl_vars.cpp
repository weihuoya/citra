// Copyright 2019 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "video_core/renderer_opengl/gl_vars.h"

#include <cstring>

#include "common/logging/log.h"

namespace OpenGL {
bool GLES;
bool AllowShadow;

void CheckGLError(const char* file, int line) {
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        LOG_DEBUG(Render_OpenGL, "GL error {} on {}:{}", GLEnumToString(err), file, line);
    }
}

const char* GLEnumToString(GLenum type) {
    switch (type) {
    // logic ops
    case GL_CLEAR:
        return "GL_CLEAR";
    case GL_AND:
        return "GL_AND";
    case GL_AND_REVERSE:
        return "GL_AND_REVERSE";
    case GL_COPY:
        return "GL_COPY";
    case GL_SET:
        return "GL_SET";
    case GL_COPY_INVERTED:
        return "GL_COPY_INVERTED";
    case GL_NOOP:
        return "GL_NOOP";
    case GL_INVERT:
        return "GL_INVERT";
    case GL_NAND:
        return "GL_NAND";
    case GL_OR:
        return "GL_OR";
    case GL_NOR:
        return "GL_NOR";
    case GL_XOR:
        return "GL_XOR";
    case GL_EQUIV:
        return "GL_EQUIV";
    case GL_AND_INVERTED:
        return "GL_AND_INVERTED";
    case GL_OR_REVERSE:
        return "GL_OR_REVERSE";
    case GL_OR_INVERTED:
        return "GL_OR_INVERTED";
        // format
    case GL_RGBA8:
        return "GL_RGBA8";
    case GL_RGB8:
        return "GL_RGB8";
    case GL_RGB5_A1:
        return "GL_RGB5_A1";
    case GL_RGB565:
        return "GL_RGB565";
    case GL_RGBA4:
        return "GL_RGBA4";
    case GL_DEPTH24_STENCIL8:
        return "GL_DEPTH24_STENCIL8";
    // format
    case GL_RGBA:
        return "GL_RGBA";
    case GL_BGR:
        return "GL_BGR";
    case GL_RGB:
        return "GL_RGB";
    case GL_DEPTH_STENCIL:
        return "GL_DEPTH_STENCIL";
    // type
    case GL_UNSIGNED_BYTE:
        return "GL_UNSIGNED_BYTE";
    case GL_UNSIGNED_SHORT_4_4_4_4:
        return "GL_UNSIGNED_SHORT_4_4_4_4";
    case GL_UNSIGNED_SHORT_5_6_5:
        return "GL_UNSIGNED_SHORT_5_6_5";
    case GL_UNSIGNED_SHORT_5_5_5_1:
        return "GL_UNSIGNED_SHORT_5_5_5_1";
    case GL_UNSIGNED_INT_8_8_8_8:
        return "GL_UNSIGNED_INT_8_8_8_8";
    case GL_UNSIGNED_INT_24_8:
        return "GL_UNSIGNED_INT_24_8";
    // error
    case GL_OUT_OF_MEMORY:
        return "GL_OUT_OF_MEMORY";
    case GL_PACK_ALIGNMENT:
        return "GL_PACK_ALIGNMENT";
    case GL_INVALID_ENUM:
        return "GL_INVALID_ENUM";
    case GL_INVALID_VALUE:
        return "GL_INVALID_VALUE";
    case GL_INVALID_OPERATION:
        return "GL_INVALID_OPERATION";
    default: {
        static char str[16];
        snprintf(str, sizeof(str), "0x%04X", type);
        return str;
    }
    }
}

inline u8 Convert5To8(u8 v) {
    // Swizzle bits: 00012345 -> 12345123
    return (v << 3) | (v >> 2);
}

inline u8 Convert6To8(u8 v) {
    // Swizzle bits: 00123456 -> 12345612
    return (v << 2) | (v >> 4);
}

inline u32 RGBA4444ToRGBA8888(u16 src) {
    const u32 r = (src & 0x000F) << 0;
    const u32 g = (src & 0x00F0) << 4;
    const u32 b = (src & 0x0F00) << 8;
    const u32 a = (src & 0xF000) << 12;

    const u32 c = r | g | b | a;
    return c | (c << 4);
}

inline u32 RGBA5551ToRGBA8888(u16 src) {
    u8 r = Convert5To8((src >> 0) & 0x1F);
    u8 g = Convert5To8((src >> 5) & 0x1F);
    u8 b = Convert5To8((src >> 10) & 0x1F);
    u8 a = (src >> 15) & 0x1;
    a = (a) ? 0xff : 0;
    return (a << 24) | (b << 16) | (g << 8) | r;
}

inline u32 RGB565ToRGBA8888(u16 src) {
    u8 r = Convert5To8((src >> 0) & 0x1F);
    u8 g = Convert6To8((src >> 5) & 0x3F);
    u8 b = Convert5To8((src >> 11) & 0x1F);
    u8 a = 0xFF;
    return (a << 24) | (b << 16) | (g << 8) | r;
}

inline u32 BGRA8888ToRGBA8888(u32 src) {
    u32 b = src & 0xFF;
    u32 g = (src >> 8) & 0xFF;
    u32 r = (src >> 16) & 0xFF;
    u32 a = (src >> 24) & 0xFF;
    return (a << 24) | (b << 16) | (g << 8) | r;
}

void ConvertToRGBA8888(u32* dst, const u8* src, u32 num_pixels, GLenum format, GLenum type) {
    switch (format) {
    case GL_RGBA:
        switch (type) {
        case GL_UNSIGNED_INT_8_8_8_8:
        case GL_UNSIGNED_BYTE:
            memcpy(dst, src, num_pixels * 4);
            break;
        case GL_UNSIGNED_SHORT_5_5_5_1: {
            const u16* src16 = reinterpret_cast<const u16*>(src);
            for (u32 i = 0; i < num_pixels; ++i) {
                dst[i] = RGBA5551ToRGBA8888(src16[i]);
            }
        } break;
        case GL_UNSIGNED_SHORT_4_4_4_4: {
            const u16* src16 = reinterpret_cast<const u16*>(src);
            for (u32 i = 0; i < num_pixels; ++i) {
                dst[i] = RGBA4444ToRGBA8888(src16[i]);
            }
        } break;
        default:
            break;
        }
        break;
    case GL_BGR:
        switch (type) {
        case GL_UNSIGNED_BYTE: {
            for (u32 i = 0; i < num_pixels; ++i) {
                const u32 r = src[i * 3 + 2];
                const u32 g = src[i * 3 + 1];
                const u32 b = src[i * 3 + 0];
                const u32 a = 0xFF;
                dst[i] = (a << 24) | (b << 16) | (g << 8) | r;
            }
        } break;
        default:
            break;
        }
        break;
    case GL_RGB:
        switch (type) {
        case GL_UNSIGNED_SHORT_5_6_5: {
            const u16* src16 = reinterpret_cast<const u16*>(src);
            for (u32 i = 0; i < num_pixels; ++i) {
                dst[i] = RGB565ToRGBA8888(src16[i]);
            }
        } break;
        case GL_UNSIGNED_BYTE:
            for (u32 i = 0; i < num_pixels; ++i) {
                const u32 r = src[i * 3 + 0];
                const u32 g = src[i * 3 + 1];
                const u32 b = src[i * 3 + 2];
                const u32 a = 0xFF;
                dst[i] = (a << 24) | (b << 16) | (g << 8) | r;
            }
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void FlipPixels(u32* pixels, u32 width, u32 height) {
    for (u32 y = 0; y < height; ++y) {
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

} // namespace OpenGL
