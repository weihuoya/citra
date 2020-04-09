// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <glad/glad.h>
#include "common/assert.h"
#include "common/bit_field.h"
#include "common/logging/log.h"
#include "core/core.h"
#include "core/core_timing.h"
#include "core/frontend/emu_window.h"
#include "core/frontend/framebuffer_layout.h"
#include "core/hw/gpu.h"
#include "core/hw/hw.h"
#include "core/hw/lcd.h"
#include "core/memory.h"
#include "core/settings.h"
#include "core/tracer/recorder.h"
#include "video_core/debug_utils/debug_utils.h"
#include "video_core/rasterizer_interface.h"
#include "video_core/renderer_opengl/gl_state.h"
#include "video_core/renderer_opengl/gl_vars.h"
#include "video_core/renderer_opengl/on_screen_display.h"
#include "video_core/renderer_opengl/renderer_opengl.h"
#include "video_core/video_core.h"

namespace OpenGL {

static const char vertex_shader[] = R"(
in vec2 vert_position;
in vec2 vert_tex_coord;
out vec2 frag_tex_coord;

// This is a truncated 3x3 matrix for 2D transformations:
// The upper-left 2x2 submatrix performs scaling/rotation/mirroring.
// The third column performs translation.
// The third row could be used for projection, which we don't need in 2D. It hence is assumed to
// implicitly be [0, 0, 1]
uniform mat3x2 modelview_matrix;

void main() {
    // Multiply input position by the rotscale part of the matrix and then manually translate by
    // the last column. This is equivalent to using a full 3x3 matrix and expanding the vector
    // to `vec3(vert_position.xy, 1.0)`
    gl_Position = vec4(mat2(modelview_matrix) * vert_position + modelview_matrix[2], 0.0, 1.0);
    frag_tex_coord = vert_tex_coord;
}
)";

static const char fragment_shader[] = R"(
in vec2 frag_tex_coord;
out vec4 color;

uniform sampler2D color_texture;

void main() {
    color = texture(color_texture, frag_tex_coord);
}
)";

static const char post_processing_header[] = R"(
// hlsl to glsl types
#define float2 vec2
#define float3 vec3
#define float4 vec4
#define uint2 uvec2
#define uint3 uvec3
#define uint4 uvec4
#define int2 ivec2
#define int3 ivec3
#define int4 ivec4

in float2 frag_tex_coord;
out float4 output_color;

uniform float4 resolution;
uniform sampler2D color_texture;

float4 Sample() { return texture(color_texture, frag_tex_coord); }
float4 SampleLocation(float2 location) { return texture(color_texture, location); }
float4 SampleFetch(int2 location) { return texelFetch(color_texture, location, 0); }
int2 SampleSize() { return textureSize(color_texture, 0); }
float2 GetResolution() { return resolution.xy; }
float2 GetInvResolution() { return resolution.zw; }
float2 GetCoordinates() { return frag_tex_coord; }
void SetOutput(float4 color) { output_color = color; }
)";

/**
 * Vertex structure that the drawn screen rectangles are composed of.
 */
struct ScreenRectVertex {
    ScreenRectVertex(GLfloat x, GLfloat y, GLfloat u, GLfloat v) {
        position[0] = x;
        position[1] = y;
        tex_coord[0] = u;
        tex_coord[1] = v;
    }

    GLfloat position[2];
    GLfloat tex_coord[2];
};

/**
 * Defines a 1:1 pixel ortographic projection matrix with (0,0) on the top-left
 * corner and (width, height) on the lower-bottom.
 *
 * The projection part of the matrix is trivial, hence these operations are represented
 * by a 3x2 matrix.
 */
static std::array<GLfloat, 3 * 2> MakeOrthographicMatrix(float width, float height) {
    std::array<GLfloat, 3 * 2> matrix; // Laid out in column-major order

    // clang-format off
    matrix[0] = 2.f / width; matrix[2] = 0.f;           matrix[4] = -1.f;
    matrix[1] = 0.f;         matrix[3] = -2.f / height; matrix[5] = 1.f;
    // Last matrix row is implicitly assumed to be [0, 0, 1].
    // clang-format on

    return matrix;
}

/**
 * option example: //! key = value
 * @param shader
 * @param options
 */
static void ParsePostShaderOptions(const std::string& shader, std::unordered_map<std::string, std::string>& options) {
    std::size_t i = 0;
    std::size_t size = shader.size();

    std::string key;
    std::string value;

    bool is_line_begin = true;
    u32 slash_counter = 0;
    bool is_option_key = false;
    bool is_option_value = false;
    while (i < size) {
        char c = shader[i++];
        switch (c) {
        case '/':
            slash_counter += 1;
            break;
        case '!':
            if (is_line_begin && (slash_counter == 2)) {
                is_option_key = true;
            }
            is_line_begin = false;
            break;
        case '=':
            if (is_option_key) {
                is_option_key = false;
                is_option_value = true;
            }
            is_line_begin = false;
            break;
        case ' ':
        case '\t':
            is_line_begin = false;
            break;
        case '\n':
        case '\r':
            is_line_begin = true;
            is_option_key = false;
            is_option_value = false;
            slash_counter = 0;
            if (!key.empty() && !value.empty()) {
                options[key] = value;
                key.clear();
                value.clear();
            }
            break;
        default:
            if (is_option_key) {
                key += c;
            } else if (is_option_value) {
                value += c;
            }
            is_line_begin = false;
            break;
        }
    }
}

RendererOpenGL::RendererOpenGL(Frontend::EmuWindow& window) : RendererBase{window} {}
RendererOpenGL::~RendererOpenGL() = default;

/// Swap buffers (render frame)
void RendererOpenGL::SwapBuffers() {
    const Layout::FramebufferLayout& layout = render_window.GetFramebufferLayout();
    // Maintain the rasterizer's state as a priority
    OpenGLState prev_state = OpenGLState::GetCurState();
    state.viewport.x = 0;
    state.viewport.y = 0;
    state.viewport.width = layout.width;
    state.viewport.height = layout.height;
    state.Apply();

    for (int i : {0, 1, 2}) {
        int fb_id = i == 2 ? 1 : 0;
        const auto& framebuffer = GPU::g_regs.framebuffer_config[fb_id];

        // Main LCD (0): 0x1ED02204, Sub LCD (1): 0x1ED02A04
        u32 lcd_color_addr =
            (fb_id == 0) ? LCD_REG_INDEX(color_fill_top) : LCD_REG_INDEX(color_fill_bottom);
        lcd_color_addr = HW::VADDR_LCD + 4 * lcd_color_addr;
        LCD::Regs::ColorFill color_fill = {0};
        LCD::Read(color_fill.raw, lcd_color_addr);

        if (color_fill.is_enabled) {
            LoadColorToActiveGLTexture(color_fill.color_r, color_fill.color_g, color_fill.color_b,
                                       screen_infos[i].texture);

            // Resize the texture in case the framebuffer size has changed
            screen_infos[i].texture.width = 1;
            screen_infos[i].texture.height = 1;
        } else {
            if (screen_infos[i].texture.width != (GLsizei)framebuffer.width ||
                screen_infos[i].texture.height != (GLsizei)framebuffer.height ||
                screen_infos[i].texture.format != framebuffer.color_format) {
                // Reallocate texture if the framebuffer size has changed.
                // This is expected to not happen very often and hence should not be a
                // performance problem.
                ConfigureFramebufferTexture(screen_infos[i].texture, framebuffer);
            }
            LoadFBToScreenInfo(framebuffer, screen_infos[i], i == 1);
        }
    }

    DrawScreens(layout);
    m_current_frame++;

    Core::System::GetInstance().perf_stats->EndSystemFrame();

    // draw on screen display
    OSD::DrawMessage(render_window.GetFramebufferLayout());

    // Swap buffers
    render_window.PollEvents();
    render_window.SwapBuffers();

    Core::System::GetInstance().perf_stats->BeginSystemFrame();

    prev_state.Apply();
    RefreshRasterizerSetting();
}

/**
 * Loads framebuffer from emulated memory into the active OpenGL texture.
 */
void RendererOpenGL::LoadFBToScreenInfo(const GPU::Regs::FramebufferConfig& framebuffer,
                                        ScreenInfo& screen_info, bool right_eye) {

    if (framebuffer.address_right1 == 0 || framebuffer.address_right2 == 0)
        right_eye = false;

    const PAddr framebuffer_addr =
        framebuffer.active_fb == 0
            ? (!right_eye ? framebuffer.address_left1 : framebuffer.address_right1)
            : (!right_eye ? framebuffer.address_left2 : framebuffer.address_right2);

    LOG_TRACE(Render_OpenGL, "0x{:08x} bytes from 0x{:08x}({}x{}), fmt {:x}",
              framebuffer.stride * framebuffer.height, framebuffer_addr, (int)framebuffer.width,
              (int)framebuffer.height, (int)framebuffer.format);

    int bpp = GPU::Regs::BytesPerPixel(framebuffer.color_format);
    std::size_t pixel_stride = framebuffer.stride / bpp;

    // OpenGL only supports specifying a stride in units of pixels, not bytes, unfortunately
    ASSERT(pixel_stride * bpp == framebuffer.stride);

    // Ensure no bad interactions with GL_UNPACK_ALIGNMENT, which by default
    // only allows rows to have a memory alignement of 4.
    ASSERT(pixel_stride % 4 == 0);

    if (!Rasterizer()->AccelerateDisplay(framebuffer, framebuffer_addr,
                                         static_cast<u32>(pixel_stride), screen_info)) {
        // Reset the screen info's display texture to its own permanent texture
        screen_info.display_texture = screen_info.texture.resource.handle;
        screen_info.display_texcoords = Common::Rectangle<float>(0.f, 0.f, 1.f, 1.f);

        Memory::RasterizerFlushRegion(framebuffer_addr, framebuffer.stride * framebuffer.height);

        const u8* framebuffer_data = VideoCore::g_memory->GetPhysicalPointer(framebuffer_addr);

        GLuint old_tex = OpenGLState::BindTexture2D(0, screen_info.texture.resource.handle);

        glPixelStorei(GL_UNPACK_ROW_LENGTH, (GLint)pixel_stride);

        // Update existing texture
        // TODO: Test what happens on hardware when you change the framebuffer dimensions so that
        //       they differ from the LCD resolution.
        // TODO: Applications could theoretically crash Citra here by specifying too large
        //       framebuffer sizes. We should make sure that this cannot happen.
        if (GLES) {
            u32 bytes_per_pixel = screen_info.texture.gl_format == GL_RGB ? 3 : 4;
            std::vector<u8> pixels(framebuffer.width * framebuffer.height * 4);
            u32 offsets[] = {2, 1, 0, 3};
            for (u32 i = 0; i < framebuffer.width * framebuffer.height * bytes_per_pixel;
                 i += bytes_per_pixel) {
                for (u32 j = 0; j < bytes_per_pixel; ++j) {
                    pixels[i + j] = framebuffer_data[i + offsets[j]];
                }
            }
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, framebuffer.width, framebuffer.height,
                            screen_info.texture.gl_format, screen_info.texture.gl_type,
                            pixels.data());
        } else {
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, framebuffer.width, framebuffer.height,
                            screen_info.texture.gl_format, screen_info.texture.gl_type,
                            framebuffer_data);
        }

        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

        OpenGLState::BindTexture2D(0, old_tex);
    }
}

/**
 * Fills active OpenGL texture with the given RGB color. Since the color is solid, the texture can
 * be 1x1 but will stretch across whatever it's rendered on.
 */
void RendererOpenGL::LoadColorToActiveGLTexture(u8 color_r, u8 color_g, u8 color_b,
                                                const TextureInfo& texture) {
    u8 framebuffer_data[3] = {color_r, color_g, color_b};
    GLuint old_tex = OpenGLState::BindTexture2D(0, texture.resource.handle);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, framebuffer_data);
    OpenGLState::BindTexture2D(0, old_tex);
}

/**
 * Initializes the OpenGL state and creates persistent objects.
 */
void RendererOpenGL::InitOpenGLObjects() {
    glClearColor(Settings::values.bg_red, Settings::values.bg_green, Settings::values.bg_blue,
                 0.0f);

    // Generate VBO handle for drawing
    vertex_buffer.Create();

    // Generate VAO
    vertex_array.Create();

    // sampler for post shader
    filter_sampler.Create();

    // Link shaders and get variable locations
    std::string frag_source;
    bool linear_mag_filter = true;
    bool linear_min_filter = true;
    if (GLES) {
        frag_source += fragment_shader_precision_OES;
    }
    if (!Settings::values.pp_shader_name.empty()) {
        std::string pp_shader = FileUtil::GetUserPath(FileUtil::UserPath::ShaderDir) +
                                Settings::values.pp_shader_name + ".glsl";
        std::size_t size = FileUtil::ReadFileToString(true, pp_shader, pp_shader);
        if (size > 0 && size == pp_shader.size()) {
            std::unordered_map<std::string, std::string> options;
            ParsePostShaderOptions(pp_shader, options);
            linear_mag_filter = options["mag_filter"] != "nearest";
            linear_min_filter = options["min_filter"] != "nearest";
            frag_source += post_processing_header;
            frag_source += pp_shader;
        } else {
            frag_source += fragment_shader;
        }
    } else {
        frag_source += fragment_shader;
    }
    shader.Create(vertex_shader, frag_source.data());

    // sampler
    glSamplerParameteri(filter_sampler.handle, GL_TEXTURE_MAG_FILTER, linear_mag_filter ? GL_LINEAR : GL_NEAREST);
    glSamplerParameteri(filter_sampler.handle, GL_TEXTURE_MIN_FILTER, linear_min_filter ? GL_LINEAR : GL_NEAREST);
    glSamplerParameteri(filter_sampler.handle, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glSamplerParameteri(filter_sampler.handle, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

    // apply
    state.draw.shader_program = shader.handle;
    state.draw.vertex_array = vertex_array.handle;
    state.draw.vertex_buffer = vertex_buffer.handle;
    state.Apply();

    uniform_modelview_matrix = glGetUniformLocation(shader.handle, "modelview_matrix");
    uniform_color_texture = glGetUniformLocation(shader.handle, "color_texture");
    uniform_resolution = glGetUniformLocation(shader.handle, "resolution");
    attrib_position = glGetAttribLocation(shader.handle, "vert_position");
    attrib_tex_coord = glGetAttribLocation(shader.handle, "vert_tex_coord");

    // Bind texture in Texture Unit 0
    glUniform1i(uniform_color_texture, 0);

    // Attach vertex data to VAO
    glVertexAttribPointer(attrib_position, 2, GL_FLOAT, GL_FALSE, sizeof(ScreenRectVertex),
                          (GLvoid*)offsetof(ScreenRectVertex, position));
    glVertexAttribPointer(attrib_tex_coord, 2, GL_FLOAT, GL_FALSE, sizeof(ScreenRectVertex),
                          (GLvoid*)offsetof(ScreenRectVertex, tex_coord));
    glEnableVertexAttribArray(attrib_position);
    glEnableVertexAttribArray(attrib_tex_coord);

    // Allocate textures for each screen
    for (auto& screen_info : screen_infos) {
        screen_info.texture.resource.Create();

        // Allocation of storage is deferred until the first frame, when we
        // know the framebuffer size.

        OpenGLState::BindTexture2D(0, screen_info.texture.resource.handle);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        screen_info.display_texture = screen_info.texture.resource.handle;
    }

    // init
    OSD::Initialize();

    OpenGLState::BindTexture2D(0, 0);
}

void RendererOpenGL::ConfigureFramebufferTexture(TextureInfo& texture,
                                                 const GPU::Regs::FramebufferConfig& framebuffer) {
    GPU::Regs::PixelFormat format = framebuffer.color_format;
    GLint internal_format;

    texture.format = format;
    texture.width = framebuffer.width;
    texture.height = framebuffer.height;

    switch (format) {
    case GPU::Regs::PixelFormat::RGBA8:
        internal_format = GL_RGBA;
        texture.gl_format = GL_RGBA;
        texture.gl_type = GLES ? GL_UNSIGNED_BYTE : GL_UNSIGNED_INT_8_8_8_8;
        break;

    case GPU::Regs::PixelFormat::RGB8:
        // This pixel format uses BGR since GL_UNSIGNED_BYTE specifies byte-order, unlike every
        // specific OpenGL type used in this function using native-endian (that is, little-endian
        // mostly everywhere) for words or half-words.
        // TODO: check how those behave on big-endian processors.
        internal_format = GL_RGB;

        // GLES Dosen't support BGR , Use RGB instead
        texture.gl_format = GLES ? GL_RGB : GL_BGR;
        texture.gl_type = GL_UNSIGNED_BYTE;
        break;

    case GPU::Regs::PixelFormat::RGB565:
        internal_format = GL_RGB;
        texture.gl_format = GL_RGB;
        texture.gl_type = GL_UNSIGNED_SHORT_5_6_5;
        break;

    case GPU::Regs::PixelFormat::RGB5A1:
        internal_format = GL_RGBA;
        texture.gl_format = GL_RGBA;
        texture.gl_type = GL_UNSIGNED_SHORT_5_5_5_1;
        break;

    case GPU::Regs::PixelFormat::RGBA4:
        internal_format = GL_RGBA;
        texture.gl_format = GL_RGBA;
        texture.gl_type = GL_UNSIGNED_SHORT_4_4_4_4;
        break;

    default:
        UNIMPLEMENTED();
    }

    GLuint old_tex = OpenGLState::BindTexture2D(0, texture.resource.handle);

    glTexImage2D(GL_TEXTURE_2D, 0, internal_format, texture.width, texture.height, 0,
                 texture.gl_format, texture.gl_type, nullptr);

    OpenGLState::BindTexture2D(0, old_tex);
}

/**
 * Draws a single texture to the emulator window, rotating the texture to correct for the 3DS's LCD
 * rotation.
 */
void RendererOpenGL::DrawSingleScreenRotated(u32 index) {
    const ScreenInfo& screen_info = screen_infos[index];
    OpenGLState::BindTexture2D(0, screen_info.display_texture);

    float src_width = screen_info.texture.width * Settings::values.resolution_factor;
    float src_height = screen_info.texture.height * Settings::values.resolution_factor;
    glUniform4f(uniform_resolution, src_width, src_height, 1.0f / src_width, 1.0f / src_height);

    glDrawArrays(GL_TRIANGLE_STRIP, index * 2, 4);
}

/**
 * Draws the emulated screens to the emulator window.
 */
void RendererOpenGL::DrawScreens(const Layout::FramebufferLayout& layout) {
    if (VideoCore::g_renderer_bg_color_update_requested.exchange(false)) {
        // Update background color before drawing
        glClearColor(Settings::values.bg_red, Settings::values.bg_green, Settings::values.bg_blue,
                     0.0f);
    }

    glClear(GL_COLOR_BUFFER_BIT);

    OpenGLState::BindSampler(0, filter_sampler.handle);

    // Set projection matrix
    std::array<GLfloat, 3 * 2> ortho_matrix = MakeOrthographicMatrix(layout.width, layout.height);
    glUniformMatrix3x2fv(uniform_modelview_matrix, 1, GL_FALSE, ortho_matrix.data());

    // Set vertices
    const auto& top_screen = layout.top_screen;
    const auto& top_texcoords = screen_infos[0].display_texcoords;

    const auto& bottom_screen = layout.bottom_screen;
    const auto& bottom_texcoords = screen_infos[2].display_texcoords;

    const std::array<ScreenRectVertex, 8> vertices = {{
        // top screen
        ScreenRectVertex(top_screen.left, top_screen.top, top_texcoords.bottom, top_texcoords.left),
        ScreenRectVertex(top_screen.left, top_screen.top + top_screen.GetHeight(), top_texcoords.top, top_texcoords.left),
        ScreenRectVertex(top_screen.left + top_screen.GetWidth(), top_screen.top, top_texcoords.bottom, top_texcoords.right),
        ScreenRectVertex(top_screen.left + top_screen.GetWidth(), top_screen.top + top_screen.GetHeight(), top_texcoords.top, top_texcoords.right),
        // bottom screen
        ScreenRectVertex(bottom_screen.left, bottom_screen.top, bottom_texcoords.bottom, bottom_texcoords.left),
        ScreenRectVertex(bottom_screen.left, bottom_screen.top + bottom_screen.GetHeight(), bottom_texcoords.top, bottom_texcoords.left),
        ScreenRectVertex(bottom_screen.left + bottom_screen.GetWidth(), bottom_screen.top, bottom_texcoords.bottom, bottom_texcoords.right),
        ScreenRectVertex(bottom_screen.left + bottom_screen.GetWidth(), bottom_screen.top + bottom_screen.GetHeight(), bottom_texcoords.top, bottom_texcoords.right),
    }};
    // prefer `glBufferData` than `glBufferSubData` on mobile device
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STREAM_DRAW);

    if (layout.top_screen_enabled) {
        DrawSingleScreenRotated(0);
    }
    if (layout.bottom_screen_enabled) {
        DrawSingleScreenRotated(2);
    }
}

/// Initialize the renderer
Core::System::ResultStatus RendererOpenGL::Init() {
    render_window.MakeCurrent();

    const char* gl_version{reinterpret_cast<char const*>(glGetString(GL_VERSION))};
    const char* gpu_vendor{reinterpret_cast<char const*>(glGetString(GL_VENDOR))};
    const char* gpu_model{reinterpret_cast<char const*>(glGetString(GL_RENDERER))};

    LOG_INFO(Render_OpenGL, "GL_VERSION: {}", gl_version);
    LOG_INFO(Render_OpenGL, "GL_VENDOR: {}", gpu_vendor);
    LOG_INFO(Render_OpenGL, "GL_RENDERER: {}", gpu_model);

    auto& telemetry_session = Core::System::GetInstance().TelemetrySession();
    telemetry_session.AddField(Telemetry::FieldType::UserSystem, "GPU_Vendor", gpu_vendor);
    telemetry_session.AddField(Telemetry::FieldType::UserSystem, "GPU_Model", gpu_model);
    telemetry_session.AddField(Telemetry::FieldType::UserSystem, "GPU_OpenGL_Version", gl_version);

    if (!strcmp(gpu_vendor, "GDI Generic")) {
        return Core::System::ResultStatus::ErrorVideoCore_ErrorGenericDrivers;
    }

    if (!(GLAD_GL_VERSION_3_3 || GLAD_GL_ES_VERSION_3_1)) {
        return Core::System::ResultStatus::ErrorVideoCore_ErrorBelowGL33;
    }

    OpenGL::GLES = Settings::values.use_gles;

    if (strcmp("Qualcomm", gpu_vendor) == 0 && (strstr(gpu_model, "620") || strstr(gpu_model, "650"))) {
        // Snapdragon 765(Adreno 620), Snapdragon 865(Adreno 650)
        OpenGL::ClipControl = false;
    } else if (GLAD_GL_ARB_clip_control) {
        glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE);
        OpenGL::ClipControl = true;
    } else if (GLAD_GL_EXT_clip_control) {
        glClipControlEXT(GL_LOWER_LEFT, GL_ZERO_TO_ONE);
        OpenGL::ClipControl = true;
    } else {
        OpenGL::ClipControl = false;
    }

    InitOpenGLObjects();

    RefreshRasterizerSetting();

    return Core::System::ResultStatus::Success;
}

/// Shutdown the renderer
void RendererOpenGL::ShutDown() {
    OSD::Shutdown();
}

} // namespace OpenGL
