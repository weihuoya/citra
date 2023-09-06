// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <utility>
#include <glad/glad.h>
#include "common/common_types.h"
#include "common/logging/log.h"
#include "video_core/renderer_opengl/gl_resource_manager.h"
#include "video_core/renderer_opengl/gl_shader_util.h"
#include "video_core/renderer_opengl/gl_state.h"

namespace OpenGL {

void OGLRenderbuffer::Create() {
    if (handle != 0)
        return;

    glGenRenderbuffers(1, &handle);
}

void OGLRenderbuffer::Release() {
    if (handle == 0)
        return;

    glDeleteRenderbuffers(1, &handle);
    handle = 0;
}

void OGLTexture::Create() {
    if (handle != 0)
        return;

    glGenTextures(1, &handle);
}

void OGLTexture::Release() {
    if (handle == 0)
        return;

    glDeleteTextures(1, &handle);
    handle = 0;
}

void OGLSampler::Create() {
    if (handle != 0)
        return;

    glGenSamplers(1, &handle);
}

void OGLSampler::Release() {
    if (handle == 0)
        return;

    glDeleteSamplers(1, &handle);
    handle = 0;
}

void OGLShader::Create(const char* source, GLenum type) {
    if (handle != 0)
        return;
    handle = LoadShader(source, type);
}

void OGLShader::Release() {
    if (handle == 0)
        return;

    glDeleteShader(handle);
    handle = 0;
}

void OGLProgram::Create(bool separable_program, const std::vector<GLuint>& shaders) {
    if (handle != 0)
        return;
    handle = LoadProgram(separable_program, shaders);
}

void OGLProgram::Create(const char* vert_shader, const char* frag_shader) {
    OGLShader vert, frag;
    vert.Create(vert_shader, GL_VERTEX_SHADER);
    frag.Create(frag_shader, GL_FRAGMENT_SHADER);
    if (vert.handle && frag.handle) {
        Create(false, {vert.handle, frag.handle});
    }
}

void OGLProgram::Create(const char* compute_shader) {
    OGLShader shader;
    shader.Create(compute_shader, GL_COMPUTE_SHADER);

    Create(false, {shader.handle});
}

void OGLProgram::Create(GLenum format, const std::vector<GLbyte>& binary) {
    handle = glCreateProgram();
    glProgramBinary(handle, format, binary.data(), binary.size());

    // Check the link status. If this fails, it means the binary was invalid.
    GLint link_status;
    glGetProgramiv(handle, GL_LINK_STATUS, &link_status);
    if (link_status != GL_TRUE) {
        LOG_DEBUG(Render_OpenGL, "OGLProgram failed to create GL program from program binary!");
        glDeleteProgram(handle);
        handle = 0;
    }
}

void OGLProgram::GetProgramBinary(GLenum& format, std::vector<GLbyte>& binary) const {
    if (handle == 0) {
        LOG_DEBUG(Render_OpenGL, "OGLProgram GetProgramBinary failed: handle error!");
        return;
    }

    GLint program_size = 0;
    glGetProgramiv(handle, GL_PROGRAM_BINARY_LENGTH, &program_size);
    if (program_size == 0) {
        LOG_DEBUG(Render_OpenGL, "OGLProgram GetProgramBinary failed: program_size error!");
        return;
    }

    binary.resize(program_size);
    GLsizei data_size = 0;
    glGetProgramBinary(handle, program_size, &data_size, &format, binary.data());
    if (glGetError() != GL_NO_ERROR || data_size == 0) {
        LOG_DEBUG(Render_OpenGL, "OGLProgram GetProgramBinary failed: opengl error!");
        format = 0;
        binary.clear();
    }
}

void OGLProgram::Release() {
    if (handle == 0)
        return;

    glDeleteProgram(handle);
    handle = 0;
}

void OGLPipeline::Create() {
    if (handle != 0)
        return;

    glGenProgramPipelines(1, &handle);
}

void OGLPipeline::Release() {
    if (handle == 0)
        return;

    glDeleteProgramPipelines(1, &handle);
    handle = 0;
}

void OGLBuffer::Create() {
    if (handle != 0)
        return;

    glGenBuffers(1, &handle);
}

void OGLBuffer::Release() {
    if (handle == 0)
        return;

    glDeleteBuffers(1, &handle);
    handle = 0;
}

void OGLVertexArray::Create() {
    if (handle != 0)
        return;

    glGenVertexArrays(1, &handle);
}

void OGLVertexArray::Release() {
    if (handle == 0)
        return;

    glDeleteVertexArrays(1, &handle);
    handle = 0;
}

void OGLFramebuffer::Create() {
    if (handle != 0)
        return;

    glGenFramebuffers(1, &handle);
}

void OGLFramebuffer::Release() {
    if (handle == 0)
        return;

    glDeleteFramebuffers(1, &handle);
    handle = 0;
}

} // namespace OpenGL
