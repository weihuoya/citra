// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <memory>
#include "common/common_types.h"
#include "video_core/rasterizer_interface.h"
#include "video_core/video_core.h"

namespace Frontend {
class EmuWindow;
}

namespace VideoCore {

class RendererBase : NonCopyable {
public:
    explicit RendererBase(Frontend::EmuWindow& window);
    virtual ~RendererBase();

    /// Initialize the renderer
    virtual VideoCore::ResultStatus Init() = 0;

    /// Shutdown the renderer
    virtual void ShutDown() = 0;

    /// Finalize rendering the guest frame and draw into the presentation texture
    virtual void SwapBuffers() = 0;

    /// Draws the latest frame to the window waiting timeout_ms for a frame to arrive (Renderer
    /// specific implementation)
    virtual bool TryPresent() = 0;

    ///
    virtual void ResetPresent() = 0;

    /// Updates the framebuffer layout of the contained render window handle.
    void UpdateCurrentFramebufferLayout();

    // Getter/setter functions:
    // ------------------------

    u32 GetCurrentFrame() const {
        return m_current_frame;
    }

    VideoCore::RasterizerInterface* Rasterizer() const {
        return rasterizer.get();
    }

    Frontend::EmuWindow& GetRenderWindow() {
        return render_window;
    }

    const Frontend::EmuWindow& GetRenderWindow() const {
        return render_window;
    }

    void RefreshRasterizerSetting();

protected:
    Frontend::EmuWindow& render_window; ///< Reference to the render window handle.
    std::unique_ptr<VideoCore::RasterizerInterface> rasterizer;
    u32 m_current_frame = 0; ///< Current frame, should be set by the renderer

private:
    bool opengl_rasterizer_active = false;
};

} // namespace VideoCore
