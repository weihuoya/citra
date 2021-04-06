// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <memory>
#include <vector>
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

    /// Finalize rendering the guest frame and draw into the presentation texture
    virtual void SwapBuffers() = 0;

    /// Draws the latest frame to the window waiting timeout_ms for a frame to arrive (Renderer
    /// specific implementation)
    virtual bool TryPresent() = 0;

    ///
    virtual void ResetPresent() = 0;

    ///
    virtual void LoadBackgroundImage(u32* pixels, u32 width, u32 height) {}

    // Getter/setter functions:
    // ------------------------

    Frontend::EmuWindow& GetRenderWindow() {
        return render_window;
    }

protected:
    Frontend::EmuWindow& render_window;
};

} // namespace VideoCore
