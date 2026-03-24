// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include "common/math_util.h"

namespace Layout {

/// Describes the layout of the window framebuffer (size and top/bottom screen positions)
struct FramebufferLayout {
    u32 width;
    u32 height;
    bool top_screen_enabled;
    bool bottom_screen_enabled;
    Common::Rectangle<u32> top_screen;
    Common::Rectangle<u32> bottom_screen;
    bool additional_screen_enabled;
    bool additional_screen_top;
    Common::Rectangle<u32> additional_screen;

    /**
     * Returns the ration of pixel size of the top screen, compared to the native size of the 3DS
     * screen.
     */
    u32 GetScalingRatio() const;
};

/**
 * Factory method for constructing a default FramebufferLayout
 * @param width Window framebuffer width in pixels
 * @param height Window framebuffer height in pixels
 * @param is_swapped if true, the bottom screen will be displayed above the top screen
 * @return Newly created FramebufferLayout object with default screen regions initialized
 */
FramebufferLayout DefaultFrameLayout(u32 width, u32 height, bool is_swapped);

/**
 * Factory method for constructing a FramebufferLayout with only the top or bottom screen
 * @param width Window framebuffer width in pixels
 * @param height Window framebuffer height in pixels
 * @param is_swapped if true, the bottom screen will be displayed (and the top won't be displayed)
 * @return Newly created FramebufferLayout object with default screen regions initialized
 */
FramebufferLayout SingleFrameLayout(u32 width, u32 height, bool is_swapped);

/**
 * Factory method for constructing a Frame with the a 4x size Top screen with a 1x size bottom
 * screen on the right
 * This is useful in particular because it matches well with a 1920x1080 resolution monitor
 * @param width Window framebuffer width in pixels
 * @param height Window framebuffer height in pixels
 * @param is_swapped if true, the bottom screen will be the large display
 * @return Newly created FramebufferLayout object with default screen regions initialized
 */
FramebufferLayout LargeFrameLayout(u32 width, u32 height, bool is_swapped);

/**
 * Android-oriented Large Screen layout that keeps the primary screen at maximum height and sizes
 * the secondary screen to consume the remaining width while preserving aspect ratio.
 * @param width Window framebuffer width in pixels
 * @param height Window framebuffer height in pixels
 * @param is_swapped if true, the bottom screen will be the large display
 * @return Newly created FramebufferLayout object sized to fill the available width more
 * aggressively than the legacy fixed-quarter Large Screen layout
 */
FramebufferLayout LargeFrameLayoutAndroid(u32 width, u32 height, bool is_swapped);

/**
 * Factory method for constructing a top-aligned large-screen layout with a configurable secondary
 * screen scale.
 * @param width Window framebuffer width in pixels
 * @param height Window framebuffer height in pixels
 * @param is_swapped if true, the bottom screen will be the large display
 * @param scale_ratio secondary screen scale relative to the primary screen
 * @return Newly created FramebufferLayout object with top-aligned screen regions initialized
 */
FramebufferLayout LargeFrameLayoutTop(u32 width, u32 height, bool is_swapped, float scale_ratio,
                                      bool secondary_left = false,
                                      bool secondary_top = true);

/**
 * Android-oriented top-aligned large-screen layout that keeps the primary screen at maximum
 * height and sizes the secondary screen from the remaining width, capped by the requested scale.
 * @param width Window framebuffer width in pixels
 * @param height Window framebuffer height in pixels
 * @param is_swapped if true, the bottom screen will be the large display
 * @param scale_ratio secondary screen scale relative to the primary screen
 * @return Newly created FramebufferLayout object with top-aligned screen regions initialized
 */
FramebufferLayout LargeFrameLayoutTopAndroid(u32 width, u32 height, bool is_swapped,
                                             float scale_ratio,
                                             bool secondary_left = false,
                                             bool secondary_top = true);

/**
 * Factory method for constructing a hybrid large-screen layout with a large primary screen, a
 * duplicated primary screen in the upper side column, and the secondary screen below it.
 * @param width Window framebuffer width in pixels
 * @param height Window framebuffer height in pixels
 * @param is_swapped if true, the bottom screen becomes the large primary screen and duplicate
 * @return Newly created FramebufferLayout object with hybrid screen regions initialized
 */
FramebufferLayout HybridFrameLayout(u32 width, u32 height, bool is_swapped,
                                    bool side_column_left, bool secondary_screen_top);

/**
 * Calculates the top-aligned large-screen proportion at which the secondary screen exactly fills
 * the remaining width beside the full-height Android large primary screen.
 * @param width Window framebuffer width in pixels
 * @param height Window framebuffer height in pixels
 * @param is_swapped if true, the bottom screen will be the large display
 * @return Recommended secondary screen size as a percentage of the primary screen size
 */
u16 GetLargeFrameLayoutTopAndroidMaxFillProportion(u32 width, u32 height, bool is_swapped,
                                                   u32 min_left = 0, u32 min_top = 0,
                                                   u32 min_right = 0, u32 min_bottom = 0);

/**
 * Factory method for constructing a Frame with the Top screen and bottom
 * screen side by side
 * This is useful for devices with small screens, like the GPDWin
 * @param width Window framebuffer width in pixels
 * @param height Window framebuffer height in pixels
 * @param is_swapped if true, the bottom screen will be the left display
 * @return Newly created FramebufferLayout object with default screen regions initialized
 */
FramebufferLayout SideFrameLayout(u32 width, u32 height, bool is_swapped);

/**
 * Factory method for constructing a custom FramebufferLayout
 * @param width Window framebuffer width in pixels
 * @param height Window framebuffer height in pixels
 * @return Newly created FramebufferLayout object with default screen regions initialized
 */
FramebufferLayout CustomFrameLayout(u32 width, u32 height);

} // namespace Layout
