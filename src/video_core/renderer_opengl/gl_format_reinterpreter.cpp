// Copyright 2020 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/assert.h"
#include "common/scope_exit.h"
#include "video_core/renderer_opengl/gl_format_reinterpreter.h"
#include "video_core/renderer_opengl/gl_rasterizer_cache.h"
#include "video_core/renderer_opengl/gl_state.h"
#include "video_core/renderer_opengl/gl_vars.h"

namespace OpenGL {

using PixelFormat = SurfaceParams::PixelFormat;

// Virtual Console
class RGBA4toRGB5A1 final : public FormatReinterpreterBase {
public:
    RGBA4toRGB5A1() {
        constexpr std::string_view vs_source = R"(
out vec2 tex_coord;

const vec2 vertices[4] =
    vec2[4](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(-1.0, 1.0), vec2(1.0, 1.0));

void main() {
    gl_Position = vec4(vertices[gl_VertexID], 0.0, 1.0);
    tex_coord = (vertices[gl_VertexID] / 2.0 + 0.5);
}
)";

        std::string fs_source = GLES ? fragment_shader_precision_OES : "";
        fs_source += R"(
in vec2 tex_coord;

out vec4 frag_color;

uniform sampler2D source;
uniform ivec2 src_size;
uniform ivec2 src_offset;

void main() {
    ivec2 coord = ivec2(tex_coord * vec2(src_size)) + src_offset;

    ivec4 rgba4 = ivec4(texelFetch(source, coord, 0) * 15.0f);
    ivec3 rgb5 = ((rgba4.rgb << ivec3(1, 2, 3)) | (rgba4.gba >> ivec3(3, 2, 1))) & 31;
    frag_color = vec4(vec3(rgb5) / 31.0, rgba4.a & 1);
}
)";

        program.Create(vs_source.data(), fs_source.data());
        auto old_program = OpenGLState::BindShaderProgram(program.handle);
        glUniform1i(glGetUniformLocation(program.handle, "source"), 0);
        src_size_loc = glGetUniformLocation(program.handle, "src_size");
        src_offset_loc = glGetUniformLocation(program.handle, "src_offset");
        OpenGLState::BindShaderProgram(old_program);
        vao.Create();
    }

    void Reinterpret(GLuint src_tex, const Common::Rectangle<u32>& src_rect, GLuint read_fb_handle,
                     GLuint dst_tex, const Common::Rectangle<u32>& dst_rect,
                     GLuint draw_fb_handle) override {
        OpenGLState prev_state = OpenGLState::GetCurState();
        SCOPE_EXIT({ prev_state.Apply(); });

        OpenGLState state;
        state.texture_units[0].texture_2d = src_tex;
        state.draw.draw_framebuffer = draw_fb_handle;
        state.draw.shader_program = program.handle;
        state.draw.vertex_array = vao.handle;
        state.viewport = {static_cast<GLint>(dst_rect.left), static_cast<GLint>(dst_rect.bottom),
                          static_cast<GLsizei>(dst_rect.GetWidth()),
                          static_cast<GLsizei>(dst_rect.GetHeight())};
        state.Apply();

        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dst_tex,
                               0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, 0,
                               0);

        glUniform2i(src_size_loc, src_rect.GetWidth(), src_rect.GetHeight());
        glUniform2i(src_offset_loc, src_rect.left, src_rect.bottom);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    }

private:
    OGLProgram program;
    GLint src_size_loc{-1}, src_offset_loc{-1};
    OGLVertexArray vao;
};

// Batman
class ShaderRGBA8toD24S8 final : public FormatReinterpreterBase {
public:
    ShaderRGBA8toD24S8() {
        constexpr std::string_view vs_source = R"(
out vec2 tex_coord;

const vec2 vertices[4] =
vec2[4](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(-1.0, 1.0), vec2(1.0, 1.0));

void main() {
    gl_Position = vec4(vertices[gl_VertexID], 0.0, 1.0);
    tex_coord = (vertices[gl_VertexID] / 2.0 + 0.5);
}
)";

        std::string fs_source = GLES ? fragment_shader_precision_OES : "";
        fs_source += R"(
in vec2 tex_coord;

uniform sampler2D source;
uniform ivec2 src_size;
uniform ivec2 src_offset;

void main() {
    ivec2 coord = ivec2(tex_coord * vec2(src_size)) + src_offset;
    vec4 rgba = ivec4(texelFetch(source, coord, 0) * 15.0f);
    gl_FragDepth = 0;
}
)";

        program.Create(vs_source.data(), fs_source.data());
        auto old_program = OpenGLState::BindShaderProgram(program.handle);
        glUniform1i(glGetUniformLocation(program.handle, "source"), 0);
        src_size_loc = glGetUniformLocation(program.handle, "src_size");
        src_offset_loc = glGetUniformLocation(program.handle, "src_offset");
        OpenGLState::BindShaderProgram(old_program);
        vao.Create();
    }

    void Reinterpret(GLuint src_tex, const Common::Rectangle<u32>& src_rect, GLuint read_fb_handle,
                     GLuint dst_tex, const Common::Rectangle<u32>& dst_rect,
                     GLuint draw_fb_handle) override {
        OpenGLState prev_state = OpenGLState::GetCurState();
        SCOPE_EXIT({ prev_state.Apply(); });

        OpenGLState state;
        state.texture_units[0].texture_2d = src_tex;
        state.draw.draw_framebuffer = draw_fb_handle;
        state.draw.shader_program = program.handle;
        state.draw.vertex_array = vao.handle;
        state.viewport = {static_cast<GLint>(dst_rect.left), static_cast<GLint>(dst_rect.bottom),
                          static_cast<GLsizei>(dst_rect.GetWidth()),
                          static_cast<GLsizei>(dst_rect.GetHeight())};
        state.Apply();

        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, dst_tex, 0);

        glUniform2i(src_size_loc, src_rect.GetWidth(), src_rect.GetHeight());
        glUniform2i(src_offset_loc, src_rect.left, src_rect.bottom);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    }

private:
    OGLProgram program;
    GLint src_size_loc{-1}, src_offset_loc{-1};
    OGLVertexArray vao;
};

// Dead or Alive
class ShaderD16toRG8 final : public FormatReinterpreterBase {
public:
    ShaderD16toRG8() {
        constexpr std::string_view vs_source = R"(
out vec2 tex_coord;

const vec2 vertices[4] =
vec2[4](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(-1.0, 1.0), vec2(1.0, 1.0));

void main() {
    gl_Position = vec4(vertices[gl_VertexID], 0.0, 1.0);
    tex_coord = (vertices[gl_VertexID] / 2.0 + 0.5);
}
)";

        std::string fs_source = GLES ? fragment_shader_precision_OES : "";
        fs_source += R"(
in vec2 tex_coord;
out vec4 frag_color;

uniform usampler2D depth;
uniform ivec2 src_size;
uniform ivec2 src_offset;

void main() {
    ivec2 coord = ivec2(tex_coord * vec2(src_size)) + src_offset;
    uint depth_val = uint(texelFetch(depth, coord, 0).x * (exp2(16.0) - 1.0));
    uvec2 components = (uvec2(depth_val) >> uvec2(8u, 0u)) & 255u;
    frag_color = vec4(vec2(components) / 255.0f, 0.0f, 0.0f);
}
)";

        program.Create(vs_source.data(), fs_source.data());
        auto old_program = OpenGLState::BindShaderProgram(program.handle);
        glUniform1i(glGetUniformLocation(program.handle, "source"), 0);
        src_size_loc = glGetUniformLocation(program.handle, "src_size");
        src_offset_loc = glGetUniformLocation(program.handle, "src_offset");
        OpenGLState::BindShaderProgram(old_program);
        vao.Create();
    }

    void Reinterpret(GLuint src_tex, const Common::Rectangle<u32>& src_rect, GLuint read_fb_handle,
                     GLuint dst_tex, const Common::Rectangle<u32>& dst_rect,
                     GLuint draw_fb_handle) override {
        OpenGLState prev_state = OpenGLState::GetCurState();
        SCOPE_EXIT({ prev_state.Apply(); });

        OpenGLState state;
        state.texture_units[0].texture_2d = src_tex;
        state.draw.draw_framebuffer = draw_fb_handle;
        state.draw.shader_program = program.handle;
        state.draw.vertex_array = vao.handle;
        state.viewport = {static_cast<GLint>(dst_rect.left), static_cast<GLint>(dst_rect.bottom),
                          static_cast<GLsizei>(dst_rect.GetWidth()),
                          static_cast<GLsizei>(dst_rect.GetHeight())};
        state.Apply();

        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dst_tex, 0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, 0, 0);

        glUniform2i(src_size_loc, src_rect.GetWidth(), src_rect.GetHeight());
        glUniform2i(src_offset_loc, src_rect.left, src_rect.bottom);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    }

private:
    OGLProgram program;
    GLint src_size_loc{-1}, src_offset_loc{-1};
    OGLVertexArray vao;
};

class PixelBufferD24S8toABGR final : public FormatReinterpreterBase {
public:
    PixelBufferD24S8toABGR() {
        attributeless_vao.Create();
        d24s8_abgr_buffer.Create();
        d24s8_abgr_buffer_size = 0;

        constexpr std::string_view vs_source = R"(
const vec2 vertices[4] = vec2[4](vec2(-1.0, -1.0), vec2(1.0, -1.0),
                                 vec2(-1.0,  1.0), vec2(1.0,  1.0));
void main() {
    gl_Position = vec4(vertices[gl_VertexID], 0.0, 1.0);
}
)";

        std::string fs_source = GLES ? fragment_shader_precision_OES : "";
        fs_source += R"(
uniform samplerBuffer tbo;
uniform vec2 tbo_size;
uniform vec4 viewport;

out vec4 color;

void main() {
    vec2 tbo_coord = (gl_FragCoord.xy - viewport.xy) * tbo_size / viewport.zw;
    int tbo_offset = int(tbo_coord.y) * int(tbo_size.x) + int(tbo_coord.x);
    color = texelFetch(tbo, tbo_offset).rabg;
}
)";

        d24s8_abgr_shader.Create(vs_source.data(), fs_source.c_str());
        auto old_program = OpenGLState::BindShaderProgram(d24s8_abgr_shader.handle);
        glUniform1i(glGetUniformLocation(d24s8_abgr_shader.handle, "tbo"), 0);
        d24s8_abgr_tbo_size_u_id = glGetUniformLocation(d24s8_abgr_shader.handle, "tbo_size");
        d24s8_abgr_viewport_u_id = glGetUniformLocation(d24s8_abgr_shader.handle, "viewport");
        OpenGLState::BindShaderProgram(old_program);
    }

    ~PixelBufferD24S8toABGR() {}

    void Reinterpret(GLuint src_tex, const Common::Rectangle<u32>& src_rect, GLuint read_fb_handle,
                     GLuint dst_tex, const Common::Rectangle<u32>& dst_rect,
                     GLuint draw_fb_handle) override {
        OpenGLState prev_state = OpenGLState::GetCurState();
        SCOPE_EXIT({ prev_state.Apply(); });

        OpenGLState state;
        state.draw.read_framebuffer = read_fb_handle;
        state.draw.draw_framebuffer = draw_fb_handle;
        state.Apply();

        glBindBuffer(GL_PIXEL_PACK_BUFFER, d24s8_abgr_buffer.handle);

        GLsizeiptr target_pbo_size =
            static_cast<GLsizeiptr>(src_rect.GetWidth()) * src_rect.GetHeight() * 4;
        if (target_pbo_size > d24s8_abgr_buffer_size) {
            d24s8_abgr_buffer_size = target_pbo_size * 2;
            glBufferData(GL_PIXEL_PACK_BUFFER, d24s8_abgr_buffer_size, nullptr, GL_STREAM_COPY);
        }

        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D,
                               src_tex, 0);
        glReadPixels(static_cast<GLint>(src_rect.left), static_cast<GLint>(src_rect.bottom),
                     static_cast<GLsizei>(src_rect.GetWidth()),
                     static_cast<GLsizei>(src_rect.GetHeight()), GL_DEPTH_STENCIL,
                     GL_UNSIGNED_INT_24_8, 0);

        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

        // PBO now contains src_tex in RABG format
        state.draw.shader_program = d24s8_abgr_shader.handle;
        state.draw.vertex_array = attributeless_vao.handle;
        state.viewport.x = static_cast<GLint>(dst_rect.left);
        state.viewport.y = static_cast<GLint>(dst_rect.bottom);
        state.viewport.width = static_cast<GLsizei>(dst_rect.GetWidth());
        state.viewport.height = static_cast<GLsizei>(dst_rect.GetHeight());
        state.Apply();

        OGLTexture tbo;
        tbo.Create();
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_BUFFER, tbo.handle);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA8, d24s8_abgr_buffer.handle);

        glUniform2f(d24s8_abgr_tbo_size_u_id, static_cast<GLfloat>(src_rect.GetWidth()),
                    static_cast<GLfloat>(src_rect.GetHeight()));
        glUniform4f(d24s8_abgr_viewport_u_id, static_cast<GLfloat>(state.viewport.x),
                    static_cast<GLfloat>(state.viewport.y),
                    static_cast<GLfloat>(state.viewport.width),
                    static_cast<GLfloat>(state.viewport.height));

        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dst_tex,
                               0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, 0,
                               0);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

        glBindTexture(GL_TEXTURE_BUFFER, 0);
    }

private:
    OGLVertexArray attributeless_vao;
    OGLBuffer d24s8_abgr_buffer;
    GLsizeiptr d24s8_abgr_buffer_size;
    OGLProgram d24s8_abgr_shader;
    GLint d24s8_abgr_tbo_size_u_id;
    GLint d24s8_abgr_viewport_u_id;
};

// Danball Senki W
class ShaderD24S8toRGBA8 final : public FormatReinterpreterBase {
public:
    ShaderD24S8toRGBA8() {
        constexpr std::string_view vs_source = R"(
out vec2 tex_coord;

const vec2 vertices[4] =
    vec2[4](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(-1.0, 1.0), vec2(1.0, 1.0));

void main() {
    gl_Position = vec4(vertices[gl_VertexID], 0.0, 1.0);
    tex_coord = vertices[gl_VertexID] / 2.0 + 0.5;
}
)";

        std::string fs_source = GLES ? fragment_shader_precision_OES : "";
        fs_source += R"(
in vec2 tex_coord;
out vec4 frag_color;

uniform sampler2D depth;
uniform usampler2D stencil;
uniform ivec2 src_size;
uniform ivec2 src_offset;

void main() {
    ivec2 coord = ivec2(tex_coord * vec2(src_size)) + src_offset;

    uint depth_val = uint(texelFetch(depth, coord, 0).x * (exp2(32.0) - 1.0));
    uint stencil_val = texelFetch(stencil, coord, 0).x;
    uvec4 components = uvec4(stencil_val, (uvec3(depth_val) >> uvec3(24u, 16u, 8u)) & 255u);
    frag_color = vec4(components) / 255.0f;
}
)";

        program.Create(vs_source.data(), fs_source.data());
        auto cur_program = OpenGLState::BindShaderProgram(program.handle);
        src_size_loc = glGetUniformLocation(program.handle, "src_size");
        src_offset_loc = glGetUniformLocation(program.handle, "src_offset");
        glUniform1i(glGetUniformLocation(program.handle, "depth"), 0);
        glUniform1i(glGetUniformLocation(program.handle, "stencil"), 1);
        OpenGLState::BindShaderProgram(cur_program);
        vao.Create();

        // OES_texture_view doesn't seem to support D24S8 views, at least on adreno
        // so instead it will do an intermediate copy before running through the shader
        if (GLAD_GL_ARB_texture_view) {
            texture_view_func = glTextureView;
        } else {
            LOG_INFO(Render_OpenGL,
                     "Texture views are unsupported, reinterpretation will do intermediate copy");
            temp_tex.Create();
        }
    }

    void Reinterpret(GLuint src_tex, const Common::Rectangle<u32>& src_rect, GLuint read_fb_handle,
                     GLuint dst_tex, const Common::Rectangle<u32>& dst_rect,
                     GLuint draw_fb_handle) override {
        OpenGLState prev_state = OpenGLState::GetCurState();
        SCOPE_EXIT({ prev_state.Apply(); });

        OpenGLState state;
        state.texture_units[0].texture_2d = src_tex;

        if (texture_view_func) {
            temp_tex.Create();
            glActiveTexture(GL_TEXTURE1);
            texture_view_func(temp_tex.handle, GL_TEXTURE_2D, src_tex, GL_DEPTH24_STENCIL8, 0, 1, 0,
                              1);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        } else if (src_rect.top > temp_rect.top || src_rect.right > temp_rect.right) {
            temp_tex.Release();
            temp_tex.Create();
            OpenGLState::BindTexture2D(1, temp_tex.handle);
            glActiveTexture(GL_TEXTURE1);
            glTexStorage2D(GL_TEXTURE_2D, 1, GL_DEPTH24_STENCIL8, src_rect.right, src_rect.top);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            temp_rect = src_rect;
        }

        state.texture_units[1].texture_2d = temp_tex.handle;
        state.draw.draw_framebuffer = draw_fb_handle;
        state.draw.shader_program = program.handle;
        state.draw.vertex_array = vao.handle;
        state.viewport = {static_cast<GLint>(dst_rect.left), static_cast<GLint>(dst_rect.bottom),
                          static_cast<GLsizei>(dst_rect.GetWidth()),
                          static_cast<GLsizei>(dst_rect.GetHeight())};
        state.Apply();

        glActiveTexture(GL_TEXTURE1);
        if (!texture_view_func) {
            glCopyImageSubData(src_tex, GL_TEXTURE_2D, 0, src_rect.left, src_rect.bottom, 0,
                               temp_tex.handle, GL_TEXTURE_2D, 0, src_rect.left, src_rect.bottom, 0,
                               src_rect.GetWidth(), src_rect.GetHeight(), 1);
        }
        glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_STENCIL_TEXTURE_MODE, GL_STENCIL_INDEX);

        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dst_tex,
                               0);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, 0,
                               0);

        glUniform2i(src_size_loc, src_rect.GetWidth(), src_rect.GetHeight());
        glUniform2i(src_offset_loc, src_rect.left, src_rect.bottom);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

        if (texture_view_func) {
            temp_tex.Release();
        }
    }

private:
    decltype(glTextureView) texture_view_func = nullptr;
    OGLProgram program{};
    GLint src_size_loc{-1}, src_offset_loc{-1};
    OGLVertexArray vao{};
    OGLTexture temp_tex{};
    Common::Rectangle<u32> temp_rect{0, 0, 0, 0};
};

class CopyImageSubData final : public FormatReinterpreterBase {
    void Reinterpret(GLuint src_tex, const Common::Rectangle<u32>& src_rect, GLuint read_fb_handle,
                     GLuint dst_tex, const Common::Rectangle<u32>& dst_rect,
                     GLuint draw_fb_handle) override {
        glCopyImageSubData(src_tex, GL_TEXTURE_2D, 0, src_rect.left, src_rect.bottom, 0, dst_tex,
                           GL_TEXTURE_2D, 0, dst_rect.left, dst_rect.bottom, 0, src_rect.GetWidth(),
                           src_rect.GetHeight(), 1);
    }
};

FormatReinterpreterOpenGL::FormatReinterpreterOpenGL() {
    std::string_view vendor{reinterpret_cast<const char*>(glGetString(GL_VENDOR))};
    if (vendor.find("NVIDIA") != vendor.npos) {
        reinterpreters.emplace(PixelFormatPair{PixelFormat::RGBA8, PixelFormat::D24S8},
                               std::make_unique<CopyImageSubData>());
        // Nvidia bends the spec and allows direct copies between color and depth formats
        // might as well take advantage of it
        LOG_INFO(Render_OpenGL, "Using glCopyImageSubData for D24S8 to RGBA8 reinterpretation");
    } else if ((GLAD_GL_ARB_stencil_texturing && GLAD_GL_ARB_texture_storage) || GLES) {
        reinterpreters.emplace(PixelFormatPair{PixelFormat::RGBA8, PixelFormat::D24S8},
                               std::make_unique<ShaderD24S8toRGBA8>());
        LOG_INFO(Render_OpenGL, "Using shader for D24S8 to RGBA8 reinterpretation");
    } else {
        reinterpreters.emplace(PixelFormatPair{PixelFormat::RGBA8, PixelFormat::D24S8},
                               std::make_unique<PixelBufferD24S8toABGR>());
        LOG_INFO(Render_OpenGL, "Using pbo for D24S8 to RGBA8 reinterpretation");
    }
    reinterpreters.emplace(PixelFormatPair{PixelFormat::D24S8, PixelFormat::RGBA8},
                           std::make_unique<ShaderRGBA8toD24S8>());
    reinterpreters.emplace(PixelFormatPair{PixelFormat::RG8, PixelFormat::D16},
                           std::make_unique<ShaderD16toRG8>());
    reinterpreters.emplace(PixelFormatPair{PixelFormat::RGB5A1, PixelFormat::RGBA4},
                           std::make_unique<RGBA4toRGB5A1>());
}

FormatReinterpreterOpenGL::~FormatReinterpreterOpenGL() = default;

std::pair<FormatReinterpreterOpenGL::ReinterpreterMap::iterator,
          FormatReinterpreterOpenGL::ReinterpreterMap::iterator>
FormatReinterpreterOpenGL::GetPossibleReinterpretations(PixelFormat dst_format) {
    return reinterpreters.equal_range(dst_format);
}

} // namespace OpenGL
