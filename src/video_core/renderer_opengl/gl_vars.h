// Copyright 2019 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <glad/glad.h>
#include "common/common_types.h"

namespace OpenGL {
extern bool GLES;
extern bool AllowShadow;

// #define DEBUG_OPENGL
#if defined(DEBUG_OPENGL)
void CheckGLError(const char* file, int line);
#define CHECK_GL_ERROR() CheckGLError(__FILE__, __LINE__)
#else
#define CHECK_GL_ERROR() (void(0))
#endif

const char* GLEnumToString(GLenum type);
void ConvertToRGBA8888(u32* dst, const u8* src, u32 num_pixels, GLenum format, GLenum type);
void FlipPixels(u32* pixels, u32 width, u32 height);

} // namespace OpenGL
