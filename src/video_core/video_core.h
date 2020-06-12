// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <atomic>
#include <memory>
#include "core/core.h"
#include "core/frontend/emu_window.h"

namespace Frontend {
class EmuWindow;
}

namespace Memory {
class MemorySystem;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Video Core namespace

namespace VideoCore {

class RendererBase;

extern std::unique_ptr<RendererBase> g_renderer; ///< Renderer plugin

// TODO: Wrap these in a user settings struct along with any other graphics settings (often set from
// qt ui)
extern std::atomic<bool> g_hw_renderer_enabled;
extern std::atomic<bool> g_shader_jit_enabled;
extern std::atomic<bool> g_hw_shader_enabled;
extern std::atomic<bool> g_renderer_bg_color_update_requested;

extern Memory::MemorySystem* g_memory;

enum class ResultStatus {
    Success,
    ErrorGenericDrivers,
    ErrorBelowGL33,
};

/// Initialize the video core
ResultStatus Init(Frontend::EmuWindow& emu_window, Memory::MemorySystem& memory);

/// Shutdown the video core
void Shutdown();

u16 GetResolutionScaleFactor();

} // namespace VideoCore
