// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <utility>
#include <glad/glad.h>
#include "common/common_types.h"
#include "common/logging/log.h"
#include "common/microprofile.h"
#include "video_core/renderer_opengl/gl_resource_manager.h"
#include "video_core/renderer_opengl/gl_shader_util.h"
#include "video_core/renderer_opengl/gl_state.h"

MICROPROFILE_DEFINE(OpenGL_ResourceCreation, "OpenGL", "Resource Creation", MP_RGB(128, 128, 192));
MICROPROFILE_DEFINE(OpenGL_ResourceDeletion, "OpenGL", "Resource Deletion", MP_RGB(128, 128, 192));

namespace OpenGL {

void OGLTexture::Create() {
    if (handle != 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceCreation);
    glGenTextures(1, &handle);
}

void OGLTexture::Release() {
    if (handle == 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceDeletion);
    glDeleteTextures(1, &handle);
    OpenGLState::ResetTexture(handle);
    handle = 0;
}

void OGLSampler::Create() {
    if (handle != 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceCreation);
    glGenSamplers(1, &handle);
}

void OGLSampler::Release() {
    if (handle == 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceDeletion);
    glDeleteSamplers(1, &handle);
    OpenGLState::ResetSampler(handle);
    handle = 0;
}

void OGLShader::Create(const char* source, GLenum type) {
    if (handle != 0)
        return;
    if (source == nullptr)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceCreation);
    handle = LoadShader(source, type);
}

void OGLShader::Release() {
    if (handle == 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceDeletion);
    glDeleteShader(handle);
    handle = 0;
}

void OGLProgram::Create(bool separable_program, const std::vector<GLuint>& shaders) {
    if (handle != 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceCreation);
    handle = LoadProgram(separable_program, shaders);
}

void OGLProgram::Create(const char* vert_shader, const char* frag_shader) {
    OGLShader vert, frag;
    vert.Create(vert_shader, GL_VERTEX_SHADER);
    frag.Create(frag_shader, GL_FRAGMENT_SHADER);

    MICROPROFILE_SCOPE(OpenGL_ResourceCreation);
    Create(false, {vert.handle, frag.handle});
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

    MICROPROFILE_SCOPE(OpenGL_ResourceDeletion);
    glDeleteProgram(handle);
    OpenGLState::ResetProgram(handle);
    handle = 0;
}

void OGLPipeline::Create() {
    if (handle != 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceCreation);
    glGenProgramPipelines(1, &handle);
}

void OGLPipeline::Release() {
    if (handle == 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceDeletion);
    glDeleteProgramPipelines(1, &handle);
    OpenGLState::ResetPipeline(handle);
    handle = 0;
}

void OGLBuffer::Create() {
    if (handle != 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceCreation);
    glGenBuffers(1, &handle);
}

void OGLBuffer::Release() {
    if (handle == 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceDeletion);
    glDeleteBuffers(1, &handle);
    OpenGLState::ResetBuffer(handle);
    handle = 0;
}

void OGLVertexArray::Create() {
    if (handle != 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceCreation);
    glGenVertexArrays(1, &handle);
}

void OGLVertexArray::Release() {
    if (handle == 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceDeletion);
    glDeleteVertexArrays(1, &handle);
    OpenGLState::ResetVertexArray(handle);
    handle = 0;
}

void OGLFramebuffer::Create() {
    if (handle != 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceCreation);
    glGenFramebuffers(1, &handle);
}

void OGLFramebuffer::Release() {
    if (handle == 0)
        return;

    MICROPROFILE_SCOPE(OpenGL_ResourceDeletion);
    glDeleteFramebuffers(1, &handle);
    OpenGLState::ResetFramebuffer(handle);
    handle = 0;
}

} // namespace OpenGL
