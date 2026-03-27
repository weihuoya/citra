// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <vector>
#ifdef ANDROID
#include <android/log.h>
#endif
#include <glad/glad.h>
#include <queue>
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

namespace {

#ifdef ANDROID

using PerfClock = std::chrono::steady_clock;
constexpr u64 PERF_LOG_INTERVAL_NS = 5'000'000'000ULL;

u64 PerfNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               PerfClock::now().time_since_epoch())
        .count();
}

double PerfNsToMs(u64 ns) {
    return static_cast<double>(ns) / 1'000'000.0;
}

double BytesToMiB(u64 bytes) {
    return static_cast<double>(bytes) / (1024.0 * 1024.0);
}

void UpdateAtomicMax(std::atomic<u64>& target, u64 value) {
    u64 current = target.load(std::memory_order_relaxed);
    while (current < value &&
           !target.compare_exchange_weak(current, value, std::memory_order_relaxed,
                                         std::memory_order_relaxed)) {
    }
}

struct RendererPerfStats {
    std::atomic<u64> last_log_ns{0};
    std::atomic<u64> render_mailbox_calls{0};
    std::atomic<u64> render_mailbox_total_ns{0};
    std::atomic<u64> render_mailbox_max_ns{0};
    std::atomic<u64> render_draw_total_ns{0};
    std::atomic<u64> render_draw_max_ns{0};
    std::atomic<u64> mailbox_wait_calls{0};
    std::atomic<u64> mailbox_wait_total_ns{0};
    std::atomic<u64> mailbox_wait_max_ns{0};
    std::atomic<u64> mailbox_reuse_count{0};
    std::atomic<u64> present_calls{0};
    std::atomic<u64> present_empty{0};
    std::atomic<u64> present_total_ns{0};
    std::atomic<u64> present_max_ns{0};
    std::atomic<u64> present_wait_total_ns{0};
    std::atomic<u64> present_wait_max_ns{0};
    std::atomic<u64> present_blit_total_ns{0};
    std::atomic<u64> present_blit_max_ns{0};
    std::atomic<u64> present_skip_count{0};
    std::atomic<u64> reset_present_frames{0};
    std::atomic<u64> render_frame_reload_count{0};
    std::atomic<u64> render_frame_reload_bytes{0};
    std::atomic<u64> present_frame_reload_count{0};
    std::atomic<u64> framebuffer_reconfig_count{0};
    std::atomic<u64> framebuffer_reconfig_bytes{0};
    std::atomic<u64> accelerated_display_hits{0};
    std::atomic<u64> cpu_upload_hits{0};
    std::atomic<u64> cpu_upload_bytes{0};
};

RendererPerfStats g_renderer_perf_stats;

void MaybeLogRendererPerf(u64 now_ns) {
    u64 last_log_ns = g_renderer_perf_stats.last_log_ns.load(std::memory_order_relaxed);
    if (last_log_ns != 0 && now_ns - last_log_ns < PERF_LOG_INTERVAL_NS) {
        return;
    }
    if (!g_renderer_perf_stats.last_log_ns.compare_exchange_strong(last_log_ns, now_ns,
                                                                   std::memory_order_relaxed,
                                                                   std::memory_order_relaxed)) {
        return;
    }
    if (last_log_ns == 0) {
        return;
    }

    const auto render_mailbox_calls =
        g_renderer_perf_stats.render_mailbox_calls.exchange(0, std::memory_order_relaxed);
    const auto render_mailbox_total_ns =
        g_renderer_perf_stats.render_mailbox_total_ns.exchange(0, std::memory_order_relaxed);
    const auto render_mailbox_max_ns =
        g_renderer_perf_stats.render_mailbox_max_ns.exchange(0, std::memory_order_relaxed);
    const auto render_draw_total_ns =
        g_renderer_perf_stats.render_draw_total_ns.exchange(0, std::memory_order_relaxed);
    const auto render_draw_max_ns =
        g_renderer_perf_stats.render_draw_max_ns.exchange(0, std::memory_order_relaxed);
    const auto mailbox_wait_calls =
        g_renderer_perf_stats.mailbox_wait_calls.exchange(0, std::memory_order_relaxed);
    const auto mailbox_wait_total_ns =
        g_renderer_perf_stats.mailbox_wait_total_ns.exchange(0, std::memory_order_relaxed);
    const auto mailbox_wait_max_ns =
        g_renderer_perf_stats.mailbox_wait_max_ns.exchange(0, std::memory_order_relaxed);
    const auto mailbox_reuse_count =
        g_renderer_perf_stats.mailbox_reuse_count.exchange(0, std::memory_order_relaxed);
    const auto present_calls =
        g_renderer_perf_stats.present_calls.exchange(0, std::memory_order_relaxed);
    const auto present_empty =
        g_renderer_perf_stats.present_empty.exchange(0, std::memory_order_relaxed);
    const auto present_total_ns =
        g_renderer_perf_stats.present_total_ns.exchange(0, std::memory_order_relaxed);
    const auto present_max_ns =
        g_renderer_perf_stats.present_max_ns.exchange(0, std::memory_order_relaxed);
    const auto present_wait_total_ns =
        g_renderer_perf_stats.present_wait_total_ns.exchange(0, std::memory_order_relaxed);
    const auto present_wait_max_ns =
        g_renderer_perf_stats.present_wait_max_ns.exchange(0, std::memory_order_relaxed);
    const auto present_blit_total_ns =
        g_renderer_perf_stats.present_blit_total_ns.exchange(0, std::memory_order_relaxed);
    const auto present_blit_max_ns =
        g_renderer_perf_stats.present_blit_max_ns.exchange(0, std::memory_order_relaxed);
    const auto present_skip_count =
        g_renderer_perf_stats.present_skip_count.exchange(0, std::memory_order_relaxed);
    const auto reset_present_frames =
        g_renderer_perf_stats.reset_present_frames.exchange(0, std::memory_order_relaxed);
    const auto render_frame_reload_count =
        g_renderer_perf_stats.render_frame_reload_count.exchange(0, std::memory_order_relaxed);
    const auto render_frame_reload_bytes =
        g_renderer_perf_stats.render_frame_reload_bytes.exchange(0, std::memory_order_relaxed);
    const auto present_frame_reload_count =
        g_renderer_perf_stats.present_frame_reload_count.exchange(0, std::memory_order_relaxed);
    const auto framebuffer_reconfig_count =
        g_renderer_perf_stats.framebuffer_reconfig_count.exchange(0, std::memory_order_relaxed);
    const auto framebuffer_reconfig_bytes =
        g_renderer_perf_stats.framebuffer_reconfig_bytes.exchange(0, std::memory_order_relaxed);
    const auto accelerated_display_hits =
        g_renderer_perf_stats.accelerated_display_hits.exchange(0, std::memory_order_relaxed);
    const auto cpu_upload_hits =
        g_renderer_perf_stats.cpu_upload_hits.exchange(0, std::memory_order_relaxed);
    const auto cpu_upload_bytes =
        g_renderer_perf_stats.cpu_upload_bytes.exchange(0, std::memory_order_relaxed);

    if (render_mailbox_calls == 0 && present_calls == 0 && framebuffer_reconfig_count == 0 &&
        accelerated_display_hits == 0 && cpu_upload_hits == 0) {
        return;
    }

    const double avg_render_mailbox_ms =
        render_mailbox_calls == 0 ? 0.0 : PerfNsToMs(render_mailbox_total_ns) / render_mailbox_calls;
    const double avg_render_draw_ms =
        render_mailbox_calls == 0 ? 0.0 : PerfNsToMs(render_draw_total_ns) / render_mailbox_calls;
    const double avg_mailbox_wait_ms =
        mailbox_wait_calls == 0 ? 0.0 : PerfNsToMs(mailbox_wait_total_ns) / mailbox_wait_calls;
    const double avg_present_ms =
        present_calls == 0 ? 0.0 : PerfNsToMs(present_total_ns) / present_calls;
    const double avg_present_wait_ms =
        present_calls == 0 ? 0.0 : PerfNsToMs(present_wait_total_ns) / present_calls;
    const double avg_present_blit_ms =
        present_calls == 0 ? 0.0 : PerfNsToMs(present_blit_total_ns) / present_calls;

    __android_log_print(
        ANDROID_LOG_INFO, "citra",
        "[Perf][Renderer] window_ms=%.1f mailbox_calls=%llu avg_mailbox_ms=%.3f "
        "max_mailbox_ms=%.3f avg_draw_ms=%.3f max_draw_ms=%.3f wait_calls=%llu "
        "avg_wait_ms=%.3f max_wait_ms=%.3f reuses=%llu present_calls=%llu present_empty=%llu "
        "avg_present_ms=%.3f max_present_ms=%.3f avg_wait_fence_ms=%.3f max_wait_fence_ms=%.3f "
        "avg_blit_ms=%.3f max_blit_ms=%.3f skips=%llu reset_frames=%llu render_reloads=%llu "
        "render_reload_mib=%.2f present_reloads=%llu fb_reconfigs=%llu fb_reconfig_mib=%.2f "
        "accel_hits=%llu cpu_uploads=%llu cpu_upload_mib=%.2f",
        PerfNsToMs(now_ns - last_log_ns),
        static_cast<unsigned long long>(render_mailbox_calls), avg_render_mailbox_ms,
        PerfNsToMs(render_mailbox_max_ns), avg_render_draw_ms, PerfNsToMs(render_draw_max_ns),
        static_cast<unsigned long long>(mailbox_wait_calls), avg_mailbox_wait_ms,
        PerfNsToMs(mailbox_wait_max_ns), static_cast<unsigned long long>(mailbox_reuse_count),
        static_cast<unsigned long long>(present_calls),
        static_cast<unsigned long long>(present_empty), avg_present_ms,
        PerfNsToMs(present_max_ns), avg_present_wait_ms, PerfNsToMs(present_wait_max_ns),
        avg_present_blit_ms, PerfNsToMs(present_blit_max_ns),
        static_cast<unsigned long long>(present_skip_count),
        static_cast<unsigned long long>(reset_present_frames),
        static_cast<unsigned long long>(render_frame_reload_count),
        BytesToMiB(render_frame_reload_bytes),
        static_cast<unsigned long long>(present_frame_reload_count),
        static_cast<unsigned long long>(framebuffer_reconfig_count),
        BytesToMiB(framebuffer_reconfig_bytes),
        static_cast<unsigned long long>(accelerated_display_hits),
        static_cast<unsigned long long>(cpu_upload_hits), BytesToMiB(cpu_upload_bytes));
}

void RecordMailboxWait(u64 wait_ns, bool reused_frame) {
    const auto now_ns = PerfNowNs();
    g_renderer_perf_stats.mailbox_wait_calls.fetch_add(1, std::memory_order_relaxed);
    g_renderer_perf_stats.mailbox_wait_total_ns.fetch_add(wait_ns, std::memory_order_relaxed);
    UpdateAtomicMax(g_renderer_perf_stats.mailbox_wait_max_ns, wait_ns);
    if (reused_frame) {
        g_renderer_perf_stats.mailbox_reuse_count.fetch_add(1, std::memory_order_relaxed);
    }
    MaybeLogRendererPerf(now_ns);
}

void RecordPresentQueueSkip() {
    g_renderer_perf_stats.present_skip_count.fetch_add(1, std::memory_order_relaxed);
}

void RecordResetPresentFrames(u64 released_frames) {
    const auto now_ns = PerfNowNs();
    g_renderer_perf_stats.reset_present_frames.fetch_add(released_frames, std::memory_order_relaxed);
    MaybeLogRendererPerf(now_ns);
}

void RecordRenderFrameReload(u32 width, u32 height) {
    const auto now_ns = PerfNowNs();
    g_renderer_perf_stats.render_frame_reload_count.fetch_add(1, std::memory_order_relaxed);
    g_renderer_perf_stats.render_frame_reload_bytes.fetch_add(
        static_cast<u64>(width) * static_cast<u64>(height) * 4ULL, std::memory_order_relaxed);
    MaybeLogRendererPerf(now_ns);
}

void RecordPresentFrameReload() {
    const auto now_ns = PerfNowNs();
    g_renderer_perf_stats.present_frame_reload_count.fetch_add(1, std::memory_order_relaxed);
    MaybeLogRendererPerf(now_ns);
}

void RecordRenderToMailbox(u64 total_ns, u64 draw_ns) {
    const auto now_ns = PerfNowNs();
    g_renderer_perf_stats.render_mailbox_calls.fetch_add(1, std::memory_order_relaxed);
    g_renderer_perf_stats.render_mailbox_total_ns.fetch_add(total_ns, std::memory_order_relaxed);
    g_renderer_perf_stats.render_draw_total_ns.fetch_add(draw_ns, std::memory_order_relaxed);
    UpdateAtomicMax(g_renderer_perf_stats.render_mailbox_max_ns, total_ns);
    UpdateAtomicMax(g_renderer_perf_stats.render_draw_max_ns, draw_ns);
    MaybeLogRendererPerf(now_ns);
}

void RecordTryPresentEmpty() {
    const auto now_ns = PerfNowNs();
    g_renderer_perf_stats.present_empty.fetch_add(1, std::memory_order_relaxed);
    MaybeLogRendererPerf(now_ns);
}

void RecordTryPresent(u64 total_ns, u64 wait_ns, u64 blit_ns) {
    const auto now_ns = PerfNowNs();
    g_renderer_perf_stats.present_calls.fetch_add(1, std::memory_order_relaxed);
    g_renderer_perf_stats.present_total_ns.fetch_add(total_ns, std::memory_order_relaxed);
    g_renderer_perf_stats.present_wait_total_ns.fetch_add(wait_ns, std::memory_order_relaxed);
    g_renderer_perf_stats.present_blit_total_ns.fetch_add(blit_ns, std::memory_order_relaxed);
    UpdateAtomicMax(g_renderer_perf_stats.present_max_ns, total_ns);
    UpdateAtomicMax(g_renderer_perf_stats.present_wait_max_ns, wait_ns);
    UpdateAtomicMax(g_renderer_perf_stats.present_blit_max_ns, blit_ns);
    MaybeLogRendererPerf(now_ns);
}

void RecordFramebufferReconfig(u32 width, u32 height, u32 bytes_per_pixel) {
    const auto now_ns = PerfNowNs();
    g_renderer_perf_stats.framebuffer_reconfig_count.fetch_add(1, std::memory_order_relaxed);
    g_renderer_perf_stats.framebuffer_reconfig_bytes.fetch_add(
        static_cast<u64>(width) * static_cast<u64>(height) * static_cast<u64>(bytes_per_pixel),
        std::memory_order_relaxed);
    MaybeLogRendererPerf(now_ns);
}

void RecordDisplayPath(bool accelerated, u64 upload_bytes) {
    const auto now_ns = PerfNowNs();
    if (accelerated) {
        g_renderer_perf_stats.accelerated_display_hits.fetch_add(1, std::memory_order_relaxed);
    } else {
        g_renderer_perf_stats.cpu_upload_hits.fetch_add(1, std::memory_order_relaxed);
        g_renderer_perf_stats.cpu_upload_bytes.fetch_add(upload_bytes, std::memory_order_relaxed);
    }
    MaybeLogRendererPerf(now_ns);
}

#else

u64 PerfNowNs() {
    return 0;
}

void RecordMailboxWait(u64, bool) {}
void RecordPresentQueueSkip() {}
void RecordResetPresentFrames(u64) {}
void RecordRenderFrameReload(u32, u32) {}
void RecordPresentFrameReload() {}
void RecordRenderToMailbox(u64, u64) {}
void RecordTryPresentEmpty() {}
void RecordTryPresent(u64, u64, u64) {}
void RecordFramebufferReconfig(u32, u32, u32) {}
void RecordDisplayPath(bool, u64) {}

#endif

} // namespace

// If the size of this is too small, it ends up creating a soft cap on FPS as the renderer will have
// to wait on available presentation frames. There doesn't seem to be much of a downside to a larger
// number but 9 swap textures at 60FPS presentation allows for 800% speed so thats probably fine
constexpr std::size_t SWAP_CHAIN_SIZE = 6;

struct OGLFrame {
    u32 width;              /// Width of the frame (to detect resize)
    u32 height;             /// Height of the frame
    bool color_reloaded;    /// Texture attachment was recreated (ie: resized)
    OGLRenderbuffer color;  /// Buffer shared between the render/present FBO
    OGLFramebuffer render;  /// FBO created on the render thread
    OGLFramebuffer present; /// FBO created on the present thread
    GLsync render_fence;    /// Fence created on the render thread
    GLsync present_fence;   /// Fence created on the presentation thread
};

struct OGLTextureMailbox {
    std::mutex swap_chain_lock;
    std::condition_variable free_cv;
    std::array<OGLFrame, SWAP_CHAIN_SIZE> swap_chain{};
    std::queue<OGLFrame*> free_queue;
    std::deque<OGLFrame*> present_queue;
    OGLFrame* previous_frame = nullptr;
    std::chrono::milliseconds elapsed{};

    OGLTextureMailbox() {
        if (Settings::values.use_frame_limit) {
            // min to 1, speed 200%
            u16 time = std::max(200 - Settings::values.frame_limit, 2) >> 1;
            elapsed = std::chrono::milliseconds{time};
        }
        for (auto& frame : swap_chain) {
            free_queue.push(&frame);
        }
    }

    ~OGLTextureMailbox() {
        // lock the mutex and clear out the present and free_queues and notify any people who are
        // blocked to prevent deadlock on shutdown
        std::scoped_lock lock(swap_chain_lock);
        std::queue<OGLFrame*>().swap(free_queue);
        present_queue.clear();
        free_cv.notify_all();
    }

    /// called in core thread
    void ResetPresent() {
        if (!present_queue.empty()) {
            std::scoped_lock lock(swap_chain_lock);
            const auto released_frames = static_cast<u64>(present_queue.size());
            for (auto& frame : present_queue) {
                free_queue.push(frame);
            }
            present_queue.clear();
            RecordResetPresentFrames(released_frames);
        }
    }

    /// called in core thread
    /// Recreate the frame if the size of the window has changed
    void ReloadRenderFrame(OGLFrame* frame, u32 width, u32 height) {
        RecordRenderFrameReload(width, height);
        // Recreate the color texture attachment
        frame->color.Release();
        frame->color.Create();
        GLuint prev_render_buffer = OpenGLState::BindRenderbuffer(frame->color.handle);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width, height);
        // Recreate the FBO for the render target
        frame->render.Release();
        frame->render.Create();
        OpenGLState::BindReadFramebuffer(frame->render.handle);
        OpenGLState::BindDrawFramebuffer(frame->render.handle);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER,
                                  frame->color.handle);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            LOG_CRITICAL(Render_OpenGL, "Failed to recreate render FBO!");
        }
        OpenGLState::BindRenderbuffer(prev_render_buffer);
        frame->width = width;
        frame->height = height;
        frame->color_reloaded = true;
    }

    /// called in core thread
    OGLFrame* GetRenderFrame() {
        std::unique_lock<std::mutex> lock(swap_chain_lock);

        // If theres no free frames, we will reuse the oldest render frame
        if (free_queue.empty()) {
            const auto wait_start_ns = PerfNowNs();
            // wait for new entries in the present_queue
            free_cv.wait_for(lock, elapsed, [this] { return !free_queue.empty(); });
            if (free_queue.empty()) {
                RecordMailboxWait(PerfNowNs() - wait_start_ns, true);
                auto frame = present_queue.front();
                present_queue.pop_front();

                // recycle one more pending frame
                free_queue.push(present_queue.front());
                present_queue.pop_front();

                return frame;
            }
            RecordMailboxWait(PerfNowNs() - wait_start_ns, false);
        }

        OGLFrame* frame = free_queue.front();
        free_queue.pop();
        return frame;
    }

    /// called in core thread
    void ReleaseRenderFrame(OGLFrame* frame) {
        std::unique_lock<std::mutex> lock(swap_chain_lock);
        present_queue.push_back(frame);
    }

    /// called in present thread
    OGLFrame* GetPresentFrame() {
        std::unique_lock<std::mutex> lock(swap_chain_lock);

        // free the previous frame and add it back to the free queue
        if (previous_frame) {
            free_queue.push(previous_frame);
            free_cv.notify_one();
        }

        // the newest entries are pushed to the front of the queue
        previous_frame = present_queue.front();
        present_queue.pop_front();

        if (present_queue.size() > 1) {
            // skip frame if pending present frames more than one
            RecordPresentQueueSkip();
            free_queue.push(present_queue.front());
            present_queue.pop_front();
        }

        return previous_frame;
    }

    /// called in present thread
    bool IsPresentEmpty() const {
        return present_queue.empty();
    }

    /// called in present thread
    /// Recreate the presentation FBO if the color attachment was changed
    void ReloadPresentFrame(OGLFrame* frame, u32 height, u32 width) {
        RecordPresentFrameReload();
        frame->present.Release();
        frame->present.Create();
        GLint previous_draw_fbo{};
        glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &previous_draw_fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, frame->present.handle);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER,
                                  frame->color.handle);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            LOG_CRITICAL(Render_OpenGL, "Failed to recreate present FBO!");
        }
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, previous_draw_fbo);
        frame->color_reloaded = false;
    }
};

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

static const char border_fill_vertex_shader[] = R"(
void main() {
    vec2 rawpos = vec2(float(gl_VertexID & 1), float((gl_VertexID & 2) >> 1));
    gl_Position = vec4(rawpos * 2.0 - 1.0, 0.0, 1.0);
}
)";

static const char border_fill_fragment_shader[] = R"(
out vec4 color;

uniform vec2 framebuffer_size;
uniform vec4 screen_rect_0;
uniform vec4 screen_rect_1;
uniform vec4 screen_rect_2;
uniform vec4 screen_texcoords_0;
uniform vec4 screen_texcoords_1;
uniform vec4 screen_texcoords_2;
uniform int screen_enabled_0;
uniform int screen_enabled_1;
uniform int screen_enabled_2;
uniform sampler2D top_texture;
uniform sampler2D bottom_texture;
uniform sampler2D additional_texture;

const float BORDER_FILL_BRIGHTNESS = 1.18;
const float SCREEN_SAMPLE_INSET_FRACTION = 0.06;

bool Contains(vec4 rect, vec2 pixel) {
    return pixel.x >= rect.x && pixel.x < rect.z && pixel.y >= rect.y && pixel.y < rect.w;
}

vec2 ClosestRectPixel(vec4 rect, vec2 pixel) {
    return clamp(pixel, rect.xy, rect.zw);
}

float RectArea(vec4 rect) {
    vec2 size = max(rect.zw - rect.xy, vec2(0.0));
    return size.x * size.y;
}

vec2 RectPixelToTexcoord(vec4 rect, vec4 texcoords, vec2 pixel) {
    vec2 size = max(rect.zw - rect.xy, vec2(1.0));
    vec2 local = clamp((pixel - rect.xy) / size, vec2(0.0), vec2(1.0));
    float tex_y = mix(texcoords.w, texcoords.y, local.y);
    float tex_x = mix(texcoords.x, texcoords.z, local.x);
    return vec2(tex_y, tex_x);
}

vec4 SampleScreen(int screen_index, vec2 pixel) {
    if (screen_index == 0) {
        return texture(top_texture, RectPixelToTexcoord(screen_rect_0, screen_texcoords_0, pixel));
    }
    if (screen_index == 1) {
        return texture(bottom_texture,
                       RectPixelToTexcoord(screen_rect_1, screen_texcoords_1, pixel));
    }
    return texture(additional_texture,
                   RectPixelToTexcoord(screen_rect_2, screen_texcoords_2, pixel));
}

vec4 GetScreenRect(int screen_index) {
    if (screen_index == 0) {
        return screen_rect_0;
    }
    if (screen_index == 1) {
        return screen_rect_1;
    }
    return screen_rect_2;
}

vec2 InsetSamplePixel(vec4 rect, vec2 pixel) {
    vec2 size = max(rect.zw - rect.xy, vec2(1.0));
    vec2 inset = min(size * SCREEN_SAMPLE_INSET_FRACTION, size * 0.25);
    vec2 min_pixel = min(rect.xy + inset, rect.zw - inset);
    vec2 max_pixel = max(rect.xy + inset, rect.zw - inset);
    return clamp(pixel, min_pixel, max_pixel);
}

vec2 SafeNormalize(vec2 delta, vec2 fallback) {
    float len = length(delta);
    if (len <= 0.0001) {
        return fallback;
    }
    return delta / len;
}

vec2 GetPrimaryProjectedPixel(vec4 rect, vec2 pixel, float compression) {
    vec2 center = (rect.xy + rect.zw) * 0.5;
    vec2 half_size = max((rect.zw - rect.xy) * 0.5, vec2(1.0));
    vec2 normalized = clamp((pixel - center) / half_size, vec2(-1.0), vec2(1.0));
    return InsetSamplePixel(rect, center + normalized * (half_size * compression));
}

vec4 SampleScreenBlur5(int screen_index, vec2 base, vec2 blur_step) {
    vec4 center = SampleScreen(screen_index, base) * 0.40;
    vec4 left = SampleScreen(screen_index, base + vec2(-blur_step.x, 0.0)) * 0.15;
    vec4 right = SampleScreen(screen_index, base + vec2(blur_step.x, 0.0)) * 0.15;
    vec4 up = SampleScreen(screen_index, base + vec2(0.0, -blur_step.y)) * 0.15;
    vec4 down = SampleScreen(screen_index, base + vec2(0.0, blur_step.y)) * 0.15;
    return center + left + right + up + down;
}

vec4 SampleScreenBlur9(int screen_index, vec2 base, vec2 blur_step) {
    vec4 color = SampleScreen(screen_index, base) * 0.28;
    color += SampleScreen(screen_index, base + vec2(-blur_step.x, 0.0)) * 0.12;
    color += SampleScreen(screen_index, base + vec2(blur_step.x, 0.0)) * 0.12;
    color += SampleScreen(screen_index, base + vec2(0.0, -blur_step.y)) * 0.12;
    color += SampleScreen(screen_index, base + vec2(0.0, blur_step.y)) * 0.12;
    color += SampleScreen(screen_index, base + vec2(-blur_step.x, -blur_step.y)) * 0.06;
    color += SampleScreen(screen_index, base + vec2(blur_step.x, -blur_step.y)) * 0.06;
    color += SampleScreen(screen_index, base + vec2(-blur_step.x, blur_step.y)) * 0.06;
    color += SampleScreen(screen_index, base + vec2(blur_step.x, blur_step.y)) * 0.06;
    return color;
}

vec4 SampleScreenAmbient(int screen_index, vec2 pixel) {
    vec4 rect = GetScreenRect(screen_index);
    vec2 size = max(rect.zw - rect.xy, vec2(1.0));
    vec2 closest = ClosestRectPixel(rect, pixel);
    vec2 outward = SafeNormalize(pixel - closest, vec2(0.0, -1.0));
    vec2 inward = -outward;
    vec2 tangent = vec2(-outward.y, outward.x);

    float max_inset = min(size.x, size.y) * 0.18;
    vec2 anchor_near = InsetSamplePixel(rect, closest + inward * clamp(min(size.x, size.y) * 0.045, 4.0, max_inset));
    vec2 anchor_mid = InsetSamplePixel(rect, closest + inward * clamp(min(size.x, size.y) * 0.10, 8.0, max_inset));
    vec2 anchor_far = InsetSamplePixel(rect, closest + inward * clamp(min(size.x, size.y) * 0.17, 12.0, max_inset));

    vec2 local_step = clamp(size * 0.018, vec2(3.0), vec2(10.0));
    vec2 medium_step = clamp(size * 0.03, vec2(5.0), vec2(16.0));
    vec2 wide_step = clamp(size * 0.042, vec2(8.0), vec2(24.0));

    vec4 near_color = SampleScreenBlur5(screen_index, anchor_near, local_step) * 0.42;
    vec4 mid_color = SampleScreenBlur9(screen_index, anchor_mid, medium_step) * 0.34;
    vec4 far_color = SampleScreenBlur9(screen_index, anchor_far, wide_step) * 0.16;
    vec4 tangent_soft =
        (SampleScreen(screen_index, InsetSamplePixel(rect, anchor_mid - tangent * medium_step)) +
         SampleScreen(screen_index, InsetSamplePixel(rect, anchor_mid + tangent * medium_step))) *
        0.04;
    return near_color + mid_color + far_color + tangent_soft;
}

void ConsiderPrimaryScreen(int screen_index, vec4 rect, inout float best_area,
                           inout int best_index) {
    float area = RectArea(rect);
    if (area > best_area) {
        best_area = area;
        best_index = screen_index;
    }
}

void main() {
    vec2 pixel = vec2(gl_FragCoord.x, framebuffer_size.y - gl_FragCoord.y);

    if ((screen_enabled_0 != 0 && Contains(screen_rect_0, pixel)) ||
        (screen_enabled_1 != 0 && Contains(screen_rect_1, pixel)) ||
        (screen_enabled_2 != 0 && Contains(screen_rect_2, pixel))) {
        discard;
    }

    float best_area = -1.0;
    int best_index = -1;

    if (screen_enabled_0 != 0) {
        ConsiderPrimaryScreen(0, screen_rect_0, best_area, best_index);
    }
    if (screen_enabled_1 != 0) {
        ConsiderPrimaryScreen(1, screen_rect_1, best_area, best_index);
    }
    if (screen_enabled_2 != 0) {
        ConsiderPrimaryScreen(2, screen_rect_2, best_area, best_index);
    }

    if (best_index < 0) {
        discard;
    }

    vec4 fill_color = SampleScreenAmbient(best_index, pixel);

    fill_color.rgb = clamp(fill_color.rgb * BORDER_FILL_BRIGHTNESS, vec3(0.0), vec3(1.0));
    color = fill_color;
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
float2 GetResolution() { return resolution.zw; }
float2 GetInvResolution() { return 1.0 / resolution.zw; }
float2 GetOnScreenSize() { return resolution.xy; }
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
static void ParsePostShaderOptions(const std::string& shader,
                                   std::unordered_map<std::string, std::string>& options) {
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

RendererOpenGL::RendererOpenGL(Frontend::EmuWindow& window, bool use_gles) : RendererBase{window} {
    OpenGL::GLES = use_gles;
    mailbox = std::make_unique<OGLTextureMailbox>();
}

RendererOpenGL::~RendererOpenGL() {
    OSD::Shutdown();
}

/// Swap buffers (render frame)
void RendererOpenGL::SwapBuffers() {
    const auto& layout = render_window.GetFramebufferLayout();
    // Maintain the rasterizer's state as a priority
    OpenGLState prev_state = OpenGLState::GetCurState();
    state.viewport.x = 0;
    state.viewport.y = 0;
    state.viewport.width = layout.width;
    state.viewport.height = layout.height;
    state.Apply();

    for (int i : {0, 2}) {
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

    RenderScreenshot();

    if (Settings::values.use_present_thread) {
        RenderToMailbox(layout);
    } else {
        // The accelerated display path can leave an offscreen FBO bound.
        // Rebind the window/default framebuffer before the final composition pass.
        OpenGLState::BindReadFramebuffer(0);
        OpenGLState::BindDrawFramebuffer(0);
        DrawScreens(layout);
        render_window.SwapBuffers();
    }
    prev_state.Apply();

    VideoCore::FrameUpdate();
}

void RendererOpenGL::RenderScreenshot() {
    if (VideoCore::g_screenshot_complete_callback) {
        // Draw this frame to the screenshot framebuffer
        OGLFramebuffer screenshot_framebuffer;
        screenshot_framebuffer.Create();
        OpenGLState::BindReadFramebuffer(screenshot_framebuffer.handle);
        OpenGLState::BindDrawFramebuffer(screenshot_framebuffer.handle);

        const auto& layout = render_window.GetFramebufferLayout();
        GLuint renderbuffer;
        glGenRenderbuffers(1, &renderbuffer);
        glBindRenderbuffer(GL_RENDERBUFFER, renderbuffer);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, layout.width, layout.height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER,
                                  renderbuffer);

        DrawScreens(layout);

        std::vector<u32> pixels(layout.width * layout.height);
        glReadPixels(0, 0, layout.width, layout.height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
        FlipPixels(pixels.data(), layout.width, layout.height);

        screenshot_framebuffer.Release();
        glDeleteRenderbuffers(1, &renderbuffer);
        VideoCore::g_screenshot_complete_callback(layout.width, layout.height, pixels);
        VideoCore::g_screenshot_complete_callback = nullptr;
    }
}

/// run in core thread
void RendererOpenGL::RenderToMailbox(const Layout::FramebufferLayout& layout) {
    const auto total_start_ns = PerfNowNs();
    OGLFrame* frame = mailbox->GetRenderFrame();

    // Clean up sync objects before drawing

    // INTEL driver workaround. We can't delete the previous render sync object until we are
    // sure that the presentation is done
    if (frame->present_fence) {
        glClientWaitSync(frame->present_fence, 0, GL_TIMEOUT_IGNORED);
        glWaitSync(frame->present_fence, 0, GL_TIMEOUT_IGNORED);
        glDeleteSync(frame->present_fence);
        frame->present_fence = nullptr;
    }

    // drawing

    // Recreate the frame if the size of the window has changed
    if (layout.width != frame->width || layout.height != frame->height) {
        LOG_DEBUG(Render_OpenGL, "Reloading render frame");
        mailbox->ReloadRenderFrame(frame, layout.width, layout.height);
    }
    OpenGLState::BindDrawFramebuffer(frame->render.handle);

    // draw frame
    const auto draw_start_ns = PerfNowNs();
    DrawScreens(layout);
    const auto draw_ns = PerfNowNs() - draw_start_ns;

    // delete the draw fence if the frame wasn't presented
    if (frame->render_fence) {
        glDeleteSync(frame->render_fence);
    }
    // Create a fence for the frontend to wait on and swap this frame to OffTex
    frame->render_fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    glFlush();

    mailbox->ReleaseRenderFrame(frame);
    RecordRenderToMailbox(PerfNowNs() - total_start_ns, draw_ns);
}

/// run in present thread
bool RendererOpenGL::TryPresent() {
    if (mailbox->IsPresentEmpty()) {
        RecordTryPresentEmpty();
        return false;
    }

    const auto total_start_ns = PerfNowNs();
    auto frame = mailbox->GetPresentFrame();
    const auto& layout = render_window.GetFramebufferLayout();

    // Clearing before a full overwrite of a fbo can signal to drivers that they can avoid a
    // readback since we won't be doing any blending
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    // Recreate the presentation FBO if the color attachment was changed
    if (frame->color_reloaded) {
        mailbox->ReloadPresentFrame(frame, layout.width, layout.height);
    }
    const auto wait_start_ns = PerfNowNs();
    glWaitSync(frame->render_fence, 0, GL_TIMEOUT_IGNORED);
    const auto wait_ns = PerfNowNs() - wait_start_ns;
    // INTEL workaround.
    // Normally we could just delete the draw fence here, but due to driver bugs, we can just delete
    // it on the emulation thread without too much penalty
    // glDeleteSync(frame.render_sync);
    // frame.render_sync = 0;

    const auto blit_start_ns = PerfNowNs();
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, frame->present.handle);
    glBlitFramebuffer(0, 0, frame->width, frame->height, 0, 0, layout.width, layout.height,
                      GL_COLOR_BUFFER_BIT, GL_LINEAR);
    const auto blit_ns = PerfNowNs() - blit_start_ns;

    /* insert fence for the main thread to block on */
    frame->present_fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    glFlush();
    RecordTryPresent(PerfNowNs() - total_start_ns, wait_ns, blit_ns);

    return true;
}

///
void RendererOpenGL::ResetPresent() {
    mailbox->ResetPresent();
}

void RendererOpenGL::LoadBackgroundImage(u32* pixels, u32 width, u32 height) {
    u32 diff = 0;
    u32 size = width * height;
    for (u32 i = 0; i < size - 1; ++i) {
        diff |= pixels[i] ^ pixels[i + 1];
    }
    if (diff == 0) {
        // use solid color
        u32 pixel = pixels[0];
        float b = static_cast<float>(pixel & 255) / 255.0f;
        float g = static_cast<float>((pixel >> 8) & 255) / 255.0f;
        float r = static_cast<float>((pixel >> 16) & 255) / 255.0f;
        glClearColor(r, g, b, 0.0f);
        bg_texture.Release();
        return;
    }

    if (bg_shader.handle == 0) {
        const char bg_vertex_shader[] = R"(
out vec2 frag_tex_coord;
void main() {
    vec2 rawpos = vec2(gl_VertexID & 1, (gl_VertexID & 2) >> 1);
    frag_tex_coord = vec2(rawpos.x, rawpos.y);
    mat2 rotate = mat2(0, -1, 1, 0); // rotate -90�
    gl_Position = vec4((rawpos * 2.0 - 1.0) * rotate, 0.0, 1.0);
}
)";
        const char bg_fragment_shader[] = R"(
in vec2 frag_tex_coord;
out vec4 color;
layout(binding = 0) uniform sampler2D color_texture;
void main() {
    color = texture(color_texture, frag_tex_coord);
}
)";
        std::string frag_source;
        if (GLES) {
            frag_source = fragment_shader_precision_OES;
            frag_source += bg_fragment_shader;
        } else {
            frag_source = bg_fragment_shader;
        }
        bg_shader.Create(bg_vertex_shader, frag_source.c_str());
    }

    if (bg_texture.handle == 0) {
        bg_texture.Create();
    }

    auto old_tex = OpenGLState::BindTexture2D(0, bg_texture.handle);
    FlipPixels(pixels, width, height);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    OpenGLState::BindTexture2D(0, old_tex);
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

    const bool accelerated = VideoCore::Rasterizer()->AccelerateDisplay(
        framebuffer, framebuffer_addr, static_cast<u32>(pixel_stride), screen_info);
    RecordDisplayPath(accelerated, static_cast<u64>(framebuffer.stride) *
                                       static_cast<u64>(framebuffer.height));
    if (!accelerated) {
        // Reset the screen info's display texture to its own permanent texture
        screen_info.display_texture = screen_info.texture.resource.handle;
        screen_info.display_texcoords = Common::Rectangle<float>(0.f, 0.f, 1.f, 1.f);

        Memory::RasterizerFlushRegion(framebuffer_addr, framebuffer.stride * framebuffer.height);

        const u8* framebuffer_data = VideoCore::Memory()->GetPhysicalPointer(framebuffer_addr);

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
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

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
    if (!shader.handle) {
        // use default vertex and fragment shader
        frag_source = fragment_shader_precision_OES;
        frag_source += fragment_shader;
        shader.Create(vertex_shader, frag_source.data());
    }

    std::string border_fill_frag_source;
    if (GLES) {
        border_fill_frag_source += fragment_shader_precision_OES;
    }
    border_fill_frag_source += border_fill_fragment_shader;
    border_fill_shader.Create(border_fill_vertex_shader, border_fill_frag_source.data());

    // sampler
    glSamplerParameteri(filter_sampler.handle, GL_TEXTURE_MAG_FILTER,
                        linear_mag_filter ? GL_LINEAR : GL_NEAREST);
    glSamplerParameteri(filter_sampler.handle, GL_TEXTURE_MIN_FILTER,
                        linear_min_filter ? GL_LINEAR : GL_NEAREST);
    glSamplerParameteri(filter_sampler.handle, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glSamplerParameteri(filter_sampler.handle, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    state.texture_units[0].sampler = filter_sampler.handle;

    // apply
    state.draw.shader_program = shader.handle;
    state.draw.vertex_array = vertex_array.handle;
    state.draw.vertex_buffer = vertex_buffer.handle;
    state.Apply();

    uniform_modelview_matrix = glGetUniformLocation(shader.handle, "modelview_matrix");
    uniform_resolution = glGetUniformLocation(shader.handle, "resolution");
    GLuint color_texture = glGetUniformLocation(shader.handle, "color_texture");
    GLuint attrib_position = glGetAttribLocation(shader.handle, "vert_position");
    GLuint attrib_tex_coord = glGetAttribLocation(shader.handle, "vert_tex_coord");

    // Bind texture in Texture Unit 0
    glUniform1i(color_texture, 0);

    // Attach vertex data to VAO
    glVertexAttribPointer(attrib_position, 2, GL_FLOAT, GL_FALSE, sizeof(ScreenRectVertex),
                          (GLvoid*)offsetof(ScreenRectVertex, position));
    glVertexAttribPointer(attrib_tex_coord, 2, GL_FLOAT, GL_FALSE, sizeof(ScreenRectVertex),
                          (GLvoid*)offsetof(ScreenRectVertex, tex_coord));
    glEnableVertexAttribArray(attrib_position);
    glEnableVertexAttribArray(attrib_tex_coord);

    border_fill_uniform_framebuffer_size =
        glGetUniformLocation(border_fill_shader.handle, "framebuffer_size");
    border_fill_uniform_screen_rects[0] =
        glGetUniformLocation(border_fill_shader.handle, "screen_rect_0");
    border_fill_uniform_screen_rects[1] =
        glGetUniformLocation(border_fill_shader.handle, "screen_rect_1");
    border_fill_uniform_screen_rects[2] =
        glGetUniformLocation(border_fill_shader.handle, "screen_rect_2");
    border_fill_uniform_screen_texcoords[0] =
        glGetUniformLocation(border_fill_shader.handle, "screen_texcoords_0");
    border_fill_uniform_screen_texcoords[1] =
        glGetUniformLocation(border_fill_shader.handle, "screen_texcoords_1");
    border_fill_uniform_screen_texcoords[2] =
        glGetUniformLocation(border_fill_shader.handle, "screen_texcoords_2");
    border_fill_uniform_screen_enabled[0] =
        glGetUniformLocation(border_fill_shader.handle, "screen_enabled_0");
    border_fill_uniform_screen_enabled[1] =
        glGetUniformLocation(border_fill_shader.handle, "screen_enabled_1");
    border_fill_uniform_screen_enabled[2] =
        glGetUniformLocation(border_fill_shader.handle, "screen_enabled_2");
    border_fill_uniform_top_texture =
        glGetUniformLocation(border_fill_shader.handle, "top_texture");
    border_fill_uniform_bottom_texture =
        glGetUniformLocation(border_fill_shader.handle, "bottom_texture");
    border_fill_uniform_additional_texture =
        glGetUniformLocation(border_fill_shader.handle, "additional_texture");

    GLuint old_program = OpenGLState::BindShaderProgram(border_fill_shader.handle);
    glUniform1i(border_fill_uniform_top_texture, 0);
    glUniform1i(border_fill_uniform_bottom_texture, 1);
    glUniform1i(border_fill_uniform_additional_texture, 2);
    OpenGLState::BindShaderProgram(old_program);

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
    if (Settings::values.is_new_3ds) {
        OSD::AddMessage("New 3DS Model", OSD::MessageType::New3DS, OSD::Duration::NORMAL,
                        OSD::Color::YELLOW);
    }
    if (!Settings::values.use_hw_shader) {
        OSD::AddMessage("HW Shader Off", OSD::MessageType::HWShader, OSD::Duration::NORMAL,
                        OSD::Color::YELLOW);
    }
    if (!Settings::values.use_cpu_jit) {
        OSD::AddMessage("CPU JIT Off", OSD::MessageType::CPUJit, OSD::Duration::NORMAL,
                        OSD::Color::YELLOW);
    }

    OpenGLState::BindTexture2D(0, 0);
}

void RendererOpenGL::ConfigureFramebufferTexture(TextureInfo& texture,
                                                 const GPU::Regs::FramebufferConfig& framebuffer) {
    GPU::Regs::PixelFormat format = framebuffer.color_format;
    GLint internal_format;
    u32 bytes_per_pixel = GPU::Regs::BytesPerPixel(format);

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
    RecordFramebufferReconfig(texture.width, texture.height, bytes_per_pixel);

    OpenGLState::BindTexture2D(0, old_tex);
}

/**
 * Draws the emulated screens to the emulator window.
 */
void RendererOpenGL::DrawScreens(const Layout::FramebufferLayout& layout) {
    OpenGLState::BindShaderProgram(shader.handle);
    OpenGLState::BindVertexArray(vertex_array.handle);
    OpenGLState::BindVertexBuffer(vertex_buffer.handle);
    glViewport(0, 0, layout.width, layout.height);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_SCISSOR_TEST);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    OpenGLState::BindSampler(0, filter_sampler.handle);

    // Set projection matrix
    std::array<GLfloat, 3 * 2> ortho_matrix = MakeOrthographicMatrix(layout.width, layout.height);
    glUniformMatrix3x2fv(uniform_modelview_matrix, 1, GL_FALSE, ortho_matrix.data());

    // Set vertices
    const auto& top_screen = layout.top_screen;
    const auto& top_texcoords = screen_infos[0].display_texcoords;

    const auto& bottom_screen = layout.bottom_screen;
    const auto& bottom_texcoords = screen_infos[2].display_texcoords;

    const std::array<ScreenRectVertex, 8> base_vertices = {{
        ScreenRectVertex(top_screen.left, top_screen.top, top_texcoords.bottom, top_texcoords.left),
        ScreenRectVertex(top_screen.left, top_screen.top + top_screen.GetHeight(),
                         top_texcoords.top, top_texcoords.left),
        ScreenRectVertex(top_screen.left + top_screen.GetWidth(), top_screen.top,
                         top_texcoords.bottom, top_texcoords.right),
        ScreenRectVertex(top_screen.left + top_screen.GetWidth(),
                         top_screen.top + top_screen.GetHeight(), top_texcoords.top,
                         top_texcoords.right),
        ScreenRectVertex(bottom_screen.left, bottom_screen.top, bottom_texcoords.bottom,
                         bottom_texcoords.left),
        ScreenRectVertex(bottom_screen.left, bottom_screen.top + bottom_screen.GetHeight(),
                         bottom_texcoords.top, bottom_texcoords.left),
        ScreenRectVertex(bottom_screen.left + bottom_screen.GetWidth(), bottom_screen.top,
                         bottom_texcoords.bottom, bottom_texcoords.right),
        ScreenRectVertex(bottom_screen.left + bottom_screen.GetWidth(),
                         bottom_screen.top + bottom_screen.GetHeight(), bottom_texcoords.top,
                         bottom_texcoords.right),
    }};

    if (layout.additional_screen_enabled) {
        std::array<ScreenRectVertex, 12> vertices = {{
            base_vertices[0], base_vertices[1], base_vertices[2], base_vertices[3], base_vertices[4],
            base_vertices[5], base_vertices[6], base_vertices[7],
            ScreenRectVertex(0.0f, 0.0f, 0.0f, 0.0f),
            ScreenRectVertex(0.0f, 0.0f, 0.0f, 0.0f),
            ScreenRectVertex(0.0f, 0.0f, 0.0f, 0.0f),
            ScreenRectVertex(0.0f, 0.0f, 0.0f, 0.0f),
        }};
        const auto& additional_screen = layout.additional_screen;
        const auto& additional_texcoords =
            layout.additional_screen_top ? screen_infos[0].display_texcoords
                                         : screen_infos[2].display_texcoords;
        vertices[8] = ScreenRectVertex(additional_screen.left, additional_screen.top,
                                       additional_texcoords.bottom, additional_texcoords.left);
        vertices[9] = ScreenRectVertex(additional_screen.left,
                                       additional_screen.top + additional_screen.GetHeight(),
                                       additional_texcoords.top, additional_texcoords.left);
        vertices[10] = ScreenRectVertex(additional_screen.left + additional_screen.GetWidth(),
                                        additional_screen.top, additional_texcoords.bottom,
                                        additional_texcoords.right);
        vertices[11] = ScreenRectVertex(additional_screen.left + additional_screen.GetWidth(),
                                        additional_screen.top + additional_screen.GetHeight(),
                                        additional_texcoords.top, additional_texcoords.right);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices.data(), GL_STREAM_DRAW);
    } else {
        // Prefer `glBufferData` over `glBufferSubData` on mobile drivers.
        glBufferData(GL_ARRAY_BUFFER, sizeof(base_vertices), base_vertices.data(), GL_STREAM_DRAW);
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    if (bg_texture.handle) {
        // background image
        GLuint handle = OpenGLState::BindShaderProgram(bg_shader.handle);
        OpenGLState::BindTexture2D(0, bg_texture.handle);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        OpenGLState::BindShaderProgram(handle);
    }

    if (Settings::values.dynamic_screen_fill && border_fill_shader.handle) {
        const auto set_rect_uniform = [&](GLuint uniform, const Common::Rectangle<u32>& rect) {
            glUniform4f(uniform, static_cast<GLfloat>(rect.left), static_cast<GLfloat>(rect.top),
                        static_cast<GLfloat>(rect.right), static_cast<GLfloat>(rect.bottom));
        };
        const auto set_texcoord_uniform = [&](GLuint uniform,
                                              const Common::Rectangle<float>& texcoords) {
            glUniform4f(uniform, texcoords.left, texcoords.top, texcoords.right,
                        texcoords.bottom);
        };

        GLuint previous_program = OpenGLState::BindShaderProgram(border_fill_shader.handle);
        glUniform2f(border_fill_uniform_framebuffer_size, static_cast<GLfloat>(layout.width),
                    static_cast<GLfloat>(layout.height));

        set_rect_uniform(border_fill_uniform_screen_rects[0], layout.top_screen);
        set_rect_uniform(border_fill_uniform_screen_rects[1], layout.bottom_screen);
        set_rect_uniform(border_fill_uniform_screen_rects[2], layout.additional_screen);
        set_texcoord_uniform(border_fill_uniform_screen_texcoords[0],
                             screen_infos[0].display_texcoords);
        set_texcoord_uniform(border_fill_uniform_screen_texcoords[1],
                             screen_infos[2].display_texcoords);
        set_texcoord_uniform(border_fill_uniform_screen_texcoords[2],
                             layout.additional_screen_top ? screen_infos[0].display_texcoords
                                                          : screen_infos[2].display_texcoords);
        glUniform1i(border_fill_uniform_screen_enabled[0], layout.top_screen_enabled);
        glUniform1i(border_fill_uniform_screen_enabled[1], layout.bottom_screen_enabled);
        glUniform1i(border_fill_uniform_screen_enabled[2], layout.additional_screen_enabled);

        OpenGLState::BindTexture2D(0, screen_infos[0].display_texture);
        OpenGLState::BindTexture2D(1, screen_infos[2].display_texture);
        OpenGLState::BindTexture2D(2, layout.additional_screen_top ? screen_infos[0].display_texture
                                                                   : screen_infos[2].display_texture);
        OpenGLState::BindSampler(1, filter_sampler.handle);
        OpenGLState::BindSampler(2, filter_sampler.handle);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        OpenGLState::BindSampler(1, 0);
        OpenGLState::BindSampler(2, 0);
        OpenGLState::BindShaderProgram(previous_program);
    }

    // Draws texture to the emulator window,
    // rotating the texture to correct for the 3DS's LCD rotation.

    if (layout.top_screen_enabled) {
        const ScreenInfo& screen_info = screen_infos[0];
        OpenGLState::BindTexture2D(0, screen_info.display_texture);
        float res_width = top_screen.GetHeight();
        float res_height = top_screen.GetWidth();
        float src_width = screen_info.texture.width * Settings::values.resolution_factor;
        float src_height = screen_info.texture.height * Settings::values.resolution_factor;
        glUniform4f(uniform_resolution, res_width, res_height, src_width, src_height);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    }

    if (layout.bottom_screen_enabled) {
        const ScreenInfo& screen_info = screen_infos[2];
        OpenGLState::BindTexture2D(0, screen_info.display_texture);
        float res_width = bottom_screen.GetHeight();
        float res_height = bottom_screen.GetWidth();
        float src_width = screen_info.texture.width * Settings::values.resolution_factor;
        float src_height = screen_info.texture.height * Settings::values.resolution_factor;
        glUniform4f(uniform_resolution, res_width, res_height, src_width, src_height);
        glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);
    }

    if (layout.additional_screen_enabled) {
        const auto& additional_screen = layout.additional_screen;
        const ScreenInfo& screen_info =
            layout.additional_screen_top ? screen_infos[0] : screen_infos[2];
        OpenGLState::BindTexture2D(0, screen_info.display_texture);
        float res_width = additional_screen.GetHeight();
        float res_height = additional_screen.GetWidth();
        float src_width = screen_info.texture.width * Settings::values.resolution_factor;
        float src_height = screen_info.texture.height * Settings::values.resolution_factor;
        glUniform4f(uniform_resolution, res_width, res_height, src_width, src_height);
        glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);
    }
    // draw on screen display
    OSD::DrawMessage(render_window, layout);
}

/// Initialize the renderer
VideoCore::ResultStatus RendererOpenGL::Init() {
    const char* gl_version{reinterpret_cast<char const*>(glGetString(GL_VERSION))};
    const char* gpu_vendor{reinterpret_cast<char const*>(glGetString(GL_VENDOR))};
    const char* gpu_model{reinterpret_cast<char const*>(glGetString(GL_RENDERER))};

    LOG_INFO(Render_OpenGL, "GL_VERSION: {}", gl_version);
    LOG_INFO(Render_OpenGL, "GL_VENDOR: {}", gpu_vendor);
    LOG_INFO(Render_OpenGL, "GL_RENDERER: {}", gpu_model);

    if (!strcmp(gpu_vendor, "GDI Generic")) {
        return VideoCore::ResultStatus::ErrorGenericDrivers;
    }

    if (!(GLAD_GL_VERSION_3_3 || GLAD_GL_ES_VERSION_3_1)) {
        return VideoCore::ResultStatus::ErrorBelowGL33;
    }

    InitOpenGLObjects();

    return VideoCore::ResultStatus::Success;
}

} // namespace OpenGL
