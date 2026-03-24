// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include "core/3ds.h"
#include "core/frontend/emu_window.h"
#include "core/frontend/input.h"
#include "core/settings.h"

namespace Frontend {

class EmuWindow::TouchState : public Input::Factory<Input::TouchDevice>,
                              public std::enable_shared_from_this<TouchState> {
public:
    std::unique_ptr<Input::TouchDevice> Create(const Common::ParamPackage&) override {
        return std::make_unique<Device>(shared_from_this());
    }

    std::mutex mutex;

    bool touch_pressed = false; ///< True if touchpad area is currently pressed, otherwise false

    float touch_x = 0.0f; ///< Touchpad X-position
    float touch_y = 0.0f; ///< Touchpad Y-position

private:
    class Device : public Input::TouchDevice {
    public:
        explicit Device(std::weak_ptr<TouchState>&& touch_state) : touch_state(touch_state) {}
        std::tuple<float, float, bool> GetStatus() const override {
            if (auto state = touch_state.lock()) {
                std::lock_guard guard{state->mutex};
                return std::make_tuple(state->touch_x, state->touch_y, state->touch_pressed);
            }
            return std::make_tuple(0.0f, 0.0f, false);
        }

    private:
        std::weak_ptr<TouchState> touch_state;
    };
};

EmuWindow::EmuWindow() {
    touch_state = std::make_shared<TouchState>();
    Input::RegisterFactory<Input::TouchDevice>("emu_window", touch_state);
}

EmuWindow::~EmuWindow() {
    Input::UnregisterFactory<Input::TouchDevice>("emu_window");
}

/**
 * Check if the given x/y coordinates are within the touchpad specified by the framebuffer layout
 * @param layout FramebufferLayout object describing the framebuffer size and screen positions
 * @param framebuffer_x Framebuffer x-coordinate to check
 * @param framebuffer_y Framebuffer y-coordinate to check
 * @return True if the coordinates are within the touchpad, otherwise false
 */
static bool IsWithinTouchscreen(const Layout::FramebufferLayout& layout, unsigned framebuffer_x,
                                unsigned framebuffer_y) {
    return (
        framebuffer_y >= layout.bottom_screen.top && framebuffer_y < layout.bottom_screen.bottom &&
        framebuffer_x >= layout.bottom_screen.left && framebuffer_x < layout.bottom_screen.right);
}

static Common::Rectangle<u32> GetLayoutBounds(const Layout::FramebufferLayout& layout) {
    u32 left = std::numeric_limits<u32>::max();
    u32 top = std::numeric_limits<u32>::max();
    u32 right = 0;
    u32 bottom = 0;

    const auto accumulate = [&](const Common::Rectangle<u32>& rect) {
        left = std::min(left, rect.left);
        top = std::min(top, rect.top);
        right = std::max(right, rect.right);
        bottom = std::max(bottom, rect.bottom);
    };

    if (layout.top_screen_enabled) {
        accumulate(layout.top_screen);
    }
    if (layout.bottom_screen_enabled) {
        accumulate(layout.bottom_screen);
    }
    if (layout.additional_screen_enabled) {
        accumulate(layout.additional_screen);
    }

    if (left == std::numeric_limits<u32>::max()) {
        return {};
    }
    return {left, top, right, bottom};
}

static void ApplyMinimumMargins(Layout::FramebufferLayout& layout, u32 min_left, u32 min_top,
                                u32 min_right, u32 min_bottom) {
    const Common::Rectangle<u32> bounds = GetLayoutBounds(layout);
    if (bounds.GetWidth() == 0 || bounds.GetHeight() == 0 || layout.width == 0 ||
        layout.height == 0) {
        return;
    }

    const u32 target_left = std::min(min_left, layout.width);
    const u32 target_top = std::min(min_top, layout.height);
    const u32 target_right = layout.width > min_right ? layout.width - min_right : target_left;
    const u32 target_bottom = layout.height > min_bottom ? layout.height - min_bottom : target_top;

    if (target_right <= target_left || target_bottom <= target_top) {
        return;
    }

    const u32 available_width = target_right - target_left;
    const u32 available_height = target_bottom - target_top;
    float scale = 1.0f;
    if (bounds.GetWidth() > available_width || bounds.GetHeight() > available_height) {
        scale = std::min(static_cast<float>(available_width) / bounds.GetWidth(),
                         static_cast<float>(available_height) / bounds.GetHeight());
        if (scale <= 0.0f) {
            return;
        }
    }

    const u32 scaled_width =
        std::max<u32>(1, static_cast<u32>(std::round(bounds.GetWidth() * scale)));
    const u32 scaled_height =
        std::max<u32>(1, static_cast<u32>(std::round(bounds.GetHeight() * scale)));
    const u32 max_left = target_right > scaled_width ? target_right - scaled_width : target_left;
    const u32 max_top = target_bottom > scaled_height ? target_bottom - scaled_height : target_top;
    const u32 new_left =
        std::clamp(bounds.left, target_left, std::max(target_left, max_left));
    const u32 new_top =
        std::clamp(bounds.top, target_top, std::max(target_top, max_top));

    const auto transform = [&](Common::Rectangle<u32>& rect) {
        const auto scaled_left =
            new_left + static_cast<u32>(std::round((rect.left - bounds.left) * scale));
        const auto scaled_top =
            new_top + static_cast<u32>(std::round((rect.top - bounds.top) * scale));
        const auto scaled_right =
            new_left + static_cast<u32>(std::round((rect.right - bounds.left) * scale));
        const auto scaled_bottom =
            new_top + static_cast<u32>(std::round((rect.bottom - bounds.top) * scale));
        rect = {scaled_left, scaled_top, scaled_right, scaled_bottom};
    };

    if (layout.top_screen_enabled) {
        transform(layout.top_screen);
    }
    if (layout.bottom_screen_enabled) {
        transform(layout.bottom_screen);
    }
    if (layout.additional_screen_enabled) {
        transform(layout.additional_screen);
    }
}

std::tuple<unsigned, unsigned> EmuWindow::ClipToTouchScreen(unsigned new_x, unsigned new_y) const {
    new_x = std::max(new_x, framebuffer_layout.bottom_screen.left);
    new_x = std::min(new_x, framebuffer_layout.bottom_screen.right - 1);
    new_y = std::max(new_y, framebuffer_layout.bottom_screen.top);
    new_y = std::min(new_y, framebuffer_layout.bottom_screen.bottom - 1);
    return std::make_tuple(new_x, new_y);
}

void EmuWindow::TouchPressed(unsigned framebuffer_x, unsigned framebuffer_y) {
    if (!IsWithinTouchscreen(framebuffer_layout, framebuffer_x, framebuffer_y))
        return;
    std::lock_guard guard(touch_state->mutex);
    touch_state->touch_x =
        static_cast<float>(framebuffer_x - framebuffer_layout.bottom_screen.left) /
        (framebuffer_layout.bottom_screen.right - framebuffer_layout.bottom_screen.left);
    touch_state->touch_y =
        static_cast<float>(framebuffer_y - framebuffer_layout.bottom_screen.top) /
        (framebuffer_layout.bottom_screen.bottom - framebuffer_layout.bottom_screen.top);
    touch_state->touch_pressed = true;
}

void EmuWindow::TouchReleased() {
    std::lock_guard guard{touch_state->mutex};
    touch_state->touch_pressed = false;
    touch_state->touch_x = 0;
    touch_state->touch_y = 0;
}

void EmuWindow::TouchMoved(unsigned framebuffer_x, unsigned framebuffer_y) {
    if (!touch_state->touch_pressed)
        return;

    if (!IsWithinTouchscreen(framebuffer_layout, framebuffer_x, framebuffer_y))
        std::tie(framebuffer_x, framebuffer_y) = ClipToTouchScreen(framebuffer_x, framebuffer_y);

    TouchPressed(framebuffer_x, framebuffer_y);
}

void EmuWindow::UpdateFramebufferLayout(u32 width, u32 height) {
    Layout::FramebufferLayout layout;
    bool android_large_full_width = false;
    bool android_preserve_horizontal_center = false;
    if (Settings::values.custom_layout) {
        layout = Layout::CustomFrameLayout(width, height);
        if (Settings::values.swap_screen) {
            std::swap(layout.top_screen_enabled, layout.bottom_screen_enabled);
            std::swap(layout.top_screen, layout.bottom_screen);
        }
    } else {
        switch (Settings::values.layout_option) {
        case Settings::LayoutOption::SingleScreen:
            layout = Layout::SingleFrameLayout(width, height, Settings::values.swap_screen);
            break;
        case Settings::LayoutOption::LargeScreen:
#ifdef ANDROID
            layout = Layout::LargeFrameLayoutTopAndroid(
                width, height, Settings::values.swap_screen,
                Settings::values.large_screen_proportion / 100.0f,
                Settings::values.large_screen_secondary_left,
                Settings::values.large_screen_secondary_top);
            android_large_full_width = true;
#else
            layout = Layout::LargeFrameLayout(width, height, Settings::values.swap_screen);
#endif
            break;
        case Settings::LayoutOption::LargeScreenTop:
#ifdef ANDROID
            layout = Layout::LargeFrameLayoutTopAndroid(
                width, height, Settings::values.swap_screen,
                Settings::values.large_screen_proportion / 100.0f,
                Settings::values.large_screen_secondary_left,
                Settings::values.large_screen_secondary_top);
            android_large_full_width = true;
#else
            layout = Layout::LargeFrameLayoutTop(
                width, height, Settings::values.swap_screen,
                Settings::values.large_screen_proportion / 100.0f,
                Settings::values.large_screen_secondary_left,
                Settings::values.large_screen_secondary_top);
#endif
            break;
        case Settings::LayoutOption::HybridScreen:
            layout = Layout::HybridFrameLayout(width, height, Settings::values.swap_screen,
                                               Settings::values.hybrid_side_column_left,
                                               Settings::values.hybrid_secondary_top,
                                               Settings::values.hybrid_fit);
#ifdef ANDROID
            android_large_full_width = true;
            android_preserve_horizontal_center = true;
#endif
            break;
        case Settings::LayoutOption::SideScreen:
            width -= safe_inset_left + safe_inset_right;
            layout = Layout::SideFrameLayout(width, height, Settings::values.swap_screen);
            layout.width += safe_inset_left + safe_inset_right;
            break;
        case Settings::LayoutOption::Default:
        default:
            layout = Layout::DefaultFrameLayout(width, height, Settings::values.swap_screen);
            break;
        }
        const auto translate_x = [&](u32 offset) {
            layout.top_screen = layout.top_screen.TranslateX(offset);
            layout.bottom_screen = layout.bottom_screen.TranslateX(offset);
            if (layout.additional_screen_enabled) {
                layout.additional_screen = layout.additional_screen.TranslateX(offset);
            }
        };
        const auto translate_y = [&](u32 offset) {
            layout.top_screen = layout.top_screen.TranslateY(offset);
            layout.bottom_screen = layout.bottom_screen.TranslateY(offset);
            if (layout.additional_screen_enabled) {
                layout.additional_screen = layout.additional_screen.TranslateY(offset);
            }
        };
        if (Settings::values.swap_screen) {
            if (!android_large_full_width && safe_inset_left > layout.bottom_screen.left) {
                u32 offset = safe_inset_left - layout.bottom_screen.left;
                translate_x(offset);
            }
            if (!android_large_full_width && safe_inset_top > layout.bottom_screen.top) {
                u32 offset = safe_inset_top - layout.bottom_screen.top;
                translate_y(offset);
            }
        } else {
            if (!android_large_full_width && safe_inset_left > layout.top_screen.left) {
                u32 offset = safe_inset_left - layout.top_screen.left;
                translate_x(offset);
            }
            if (!android_large_full_width && safe_inset_top > layout.top_screen.top) {
                u32 offset = safe_inset_top - layout.top_screen.top;
                translate_y(offset);
            }
        }
#ifdef ANDROID
        if (android_large_full_width && !android_preserve_horizontal_center) {
            const u32 current_right =
                std::max({layout.top_screen.right, layout.bottom_screen.right,
                          layout.additional_screen_enabled ? layout.additional_screen.right : 0u});
            const u32 desired_right = width;
            if (current_right < desired_right) {
                const u32 offset = desired_right - current_right;
                translate_x(offset);
            }
        }
        // The user-configured layout margins are an extra constraint layer on top of the legacy
        // automatic layout behavior. Keep them independent from Android safe insets so that
        // setting all four margins to 0 restores the pre-margin layout instead of still forcing
        // hidden inset-driven padding.
        ApplyMinimumMargins(layout, static_cast<u32>(Settings::values.layout_margin_left),
                            static_cast<u32>(Settings::values.layout_margin_top),
                            static_cast<u32>(Settings::values.layout_margin_right),
                            static_cast<u32>(Settings::values.layout_margin_bottom));
#endif
    }
    NotifyFramebufferLayoutChanged(layout);
}

} // namespace Frontend
