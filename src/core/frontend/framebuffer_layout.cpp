// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <algorithm>
#include <array>
#include <cmath>
#include "common/assert.h"
#include "core/3ds.h"
#include "core/frontend/framebuffer_layout.h"
#include "core/settings.h"

namespace Layout {

static const float TOP_SCREEN_ASPECT_RATIO = static_cast<float>(Core::kScreenTopHeight) / Core::kScreenTopWidth;
static const float BOT_SCREEN_ASPECT_RATIO = static_cast<float>(Core::kScreenBottomHeight) / Core::kScreenBottomWidth;

u32 FramebufferLayout::GetScalingRatio() const {
    return static_cast<u32>(((top_screen.GetWidth() - 1) / Core::kScreenTopWidth) + 1);
}

// Finds the largest size subrectangle contained in window area that is confined to the aspect ratio
template <class T>
static Common::Rectangle<T> maxRectangle(Common::Rectangle<T> window_area, float aspect_ratio) {
    float scale = std::min(static_cast<float>(window_area.GetWidth()), window_area.GetHeight() / aspect_ratio);
    return Common::Rectangle<T>{0, 0, static_cast<T>(std::round(scale)), static_cast<T>(std::round(scale * aspect_ratio))};
}

FramebufferLayout DefaultFrameLayout(u32 width, u32 height, bool swapped) {
    FramebufferLayout res{width, height, true, true, {}, {}, false, false, {}};
    // Default layout gives equal screen sizes to the top and bottom screen
    Common::Rectangle<u32> screen_area{0, 0, width, height / 2};
    Common::Rectangle<u32> top_screen = maxRectangle(screen_area, TOP_SCREEN_ASPECT_RATIO);
    Common::Rectangle<u32> bot_screen = maxRectangle(screen_area, BOT_SCREEN_ASPECT_RATIO);

    float window_aspect_ratio = static_cast<float>(height) / width;
    // both screens height are taken into account by multiplying by 2
    float emulation_aspect_ratio = TOP_SCREEN_ASPECT_RATIO * 2;

    if (window_aspect_ratio < emulation_aspect_ratio) {
        // Apply borders to the left and right sides of the window.
        top_screen = top_screen.TranslateX((screen_area.GetWidth() - top_screen.GetWidth()) / 2);
        bot_screen = bot_screen.TranslateX((screen_area.GetWidth() - bot_screen.GetWidth()) / 2);
        // Move the top screen to the bottom if we are swapped.
        res.top_screen = swapped ? top_screen.TranslateY(height / 2) : top_screen;
        res.bottom_screen = swapped ? bot_screen : bot_screen.TranslateY(height / 2);
    } else {
        // Window is narrower than the emulation content => apply borders to the top and bottom
        // Recalculate the bottom screen to account for the width difference between top and bottom
        screen_area = {0, 0, width, top_screen.GetHeight()};
        bot_screen = maxRectangle(screen_area, BOT_SCREEN_ASPECT_RATIO);
        bot_screen = bot_screen.TranslateX((top_screen.GetWidth() - bot_screen.GetWidth()) / 2);
        if (swapped) {
            res.top_screen = top_screen.TranslateY(bot_screen.GetHeight());
            res.bottom_screen = bot_screen;
        } else {
            res.top_screen = top_screen;
            res.bottom_screen = bot_screen.TranslateY(top_screen.GetHeight());
        }
    }
    return res;
}

FramebufferLayout SingleFrameLayout(u32 width, u32 height, bool swapped) {
    // The drawing code needs at least somewhat valid values for both screens
    // so just calculate them both even if the other isn't showing.
    FramebufferLayout res{width, height, !swapped, swapped, {}, {}, false, false, {}};

    Common::Rectangle<u32> screen_area{0, 0, width, height};
    Common::Rectangle<u32> top_screen = maxRectangle(screen_area, TOP_SCREEN_ASPECT_RATIO);
    Common::Rectangle<u32> bot_screen = maxRectangle(screen_area, BOT_SCREEN_ASPECT_RATIO);

    float window_aspect_ratio = static_cast<float>(height) / width;
    float emulation_aspect_ratio = (swapped) ? BOT_SCREEN_ASPECT_RATIO : TOP_SCREEN_ASPECT_RATIO;

    if (window_aspect_ratio < emulation_aspect_ratio) {
        top_screen = top_screen.TranslateX((screen_area.GetWidth() - top_screen.GetWidth()) / 2);
        bot_screen = bot_screen.TranslateX((screen_area.GetWidth() - bot_screen.GetWidth()) / 2);
    } else {
        top_screen = top_screen.TranslateY((height - top_screen.GetHeight()) / 2);
        bot_screen = bot_screen.TranslateY((height - bot_screen.GetHeight()) / 2);
    }
    res.top_screen = top_screen;
    res.bottom_screen = bot_screen;
    return res;
}

FramebufferLayout LargeFrameLayout(u32 width, u32 height, bool swapped) {
    FramebufferLayout res{width, height, true, true, {}, {}, false, false, {}};
    // Split the window into two parts. Give 4x width to the main screen and 1x width to the small
    // To do that, find the total emulation box and maximize that based on window size
    float window_aspect_ratio = static_cast<float>(height) / width;
    float emulation_aspect_ratio =
        swapped ? Core::kScreenBottomHeight * 4 / (Core::kScreenBottomWidth * 4.0f + Core::kScreenTopWidth)
                : Core::kScreenTopHeight * 4 / (Core::kScreenTopWidth * 4.0f + Core::kScreenBottomWidth);
    float large_screen_aspect_ratio = swapped ? BOT_SCREEN_ASPECT_RATIO : TOP_SCREEN_ASPECT_RATIO;
    float small_screen_aspect_ratio = swapped ? TOP_SCREEN_ASPECT_RATIO : BOT_SCREEN_ASPECT_RATIO;

    Common::Rectangle<u32> screen_area{0, 0, width, height};
    Common::Rectangle<u32> total_rect = maxRectangle(screen_area, emulation_aspect_ratio);
    Common::Rectangle<u32> large_screen = maxRectangle(total_rect, large_screen_aspect_ratio);
    Common::Rectangle<u32> fourth_size_rect = total_rect.Scale(.25f);
    Common::Rectangle<u32> small_screen = maxRectangle(fourth_size_rect, small_screen_aspect_ratio);

    if (window_aspect_ratio < emulation_aspect_ratio) {
        large_screen = large_screen.TranslateX((screen_area.GetWidth() - total_rect.GetWidth()) / 2);
    } else {
        large_screen = large_screen.TranslateY((height - total_rect.GetHeight()) / 2);
    }
    // Shift the small screen to the bottom right corner
    small_screen = small_screen.TranslateX(large_screen.right).TranslateY(large_screen.top + small_screen.GetHeight() / 2);
    res.top_screen = swapped ? small_screen : large_screen;
    res.bottom_screen = swapped ? large_screen : small_screen;
    return res;
}

FramebufferLayout LargeFrameLayoutAndroid(u32 width, u32 height, bool swapped) {
    FramebufferLayout res{width, height, true, true, {}, {}, false, false, {}};

    const float large_screen_aspect_ratio = swapped ? BOT_SCREEN_ASPECT_RATIO : TOP_SCREEN_ASPECT_RATIO;
    const float small_screen_aspect_ratio = swapped ? TOP_SCREEN_ASPECT_RATIO : BOT_SCREEN_ASPECT_RATIO;

    Common::Rectangle<u32> screen_area{0, 0, width, height};
    Common::Rectangle<u32> large_screen = maxRectangle(screen_area, large_screen_aspect_ratio);
    large_screen = large_screen.TranslateY((height - large_screen.GetHeight()) / 2);

    const u32 remaining_width = width > large_screen.GetWidth() ? width - large_screen.GetWidth() : 0;
    if (remaining_width == 0) {
        return LargeFrameLayout(width, height, swapped);
    }

    Common::Rectangle<u32> remaining_area{0, 0, remaining_width, height};
    Common::Rectangle<u32> small_screen = maxRectangle(remaining_area, small_screen_aspect_ratio);
    small_screen = small_screen.TranslateX(large_screen.right)
                               .TranslateY((height - small_screen.GetHeight()) / 2);

    res.top_screen = swapped ? small_screen : large_screen;
    res.bottom_screen = swapped ? large_screen : small_screen;
    return res;
}

FramebufferLayout LargeFrameLayoutTop(u32 width, u32 height, bool swapped, float scale_ratio,
                                      bool secondary_left, bool secondary_top) {
    FramebufferLayout res{width, height, true, true, {}, {}, false, false, {}};

    const float clamped_ratio = std::clamp(scale_ratio, 0.25f, 1.0f);
    const float primary_width = swapped ? Core::kScreenBottomWidth : Core::kScreenTopWidth;
    const float primary_height = swapped ? Core::kScreenBottomHeight : Core::kScreenTopHeight;
    const float secondary_width = swapped ? Core::kScreenTopWidth : Core::kScreenBottomWidth;
    const float secondary_height = swapped ? Core::kScreenTopHeight : Core::kScreenBottomHeight;

    const float total_width = primary_width + (secondary_width * clamped_ratio);
    const float total_height = std::max(primary_height, secondary_height * clamped_ratio);
    const FramebufferLayout large_layout = LargeFrameLayout(width, height, swapped);
    const Common::Rectangle<u32>& large_primary = swapped ? large_layout.bottom_screen
                                                          : large_layout.top_screen;
    const Common::Rectangle<u32>& large_secondary = swapped ? large_layout.top_screen
                                                            : large_layout.bottom_screen;
    // Keep the top-aligned layout inside the original Large layout envelope so it stays within the
    // same on-screen bounds on Android devices.
    const float large_layout_width_limit =
        static_cast<float>(large_secondary.right - large_primary.left);
    const float large_layout_height_limit = static_cast<float>(large_primary.GetHeight());
    const float base_scale =
        std::min(large_layout_width_limit / total_width,
                 large_layout_height_limit / total_height);

    const u32 primary_rect_width = static_cast<u32>(std::round(primary_width * base_scale));
    const u32 primary_rect_height = static_cast<u32>(std::round(primary_height * base_scale));
    const u32 secondary_rect_width =
        static_cast<u32>(std::round(secondary_width * clamped_ratio * base_scale));
    const u32 secondary_rect_height =
        static_cast<u32>(std::round(secondary_height * clamped_ratio * base_scale));

    const u32 layout_width = primary_rect_width + secondary_rect_width;
    const u32 layout_left = large_secondary.right - layout_width;
    const u32 layout_height = std::max(primary_rect_height, secondary_rect_height);
    const u32 layout_top =
        large_primary.top + (large_primary.GetHeight() - layout_height) / 2;

    const u32 primary_left = secondary_left ? layout_left + secondary_rect_width : layout_left;
    const u32 secondary_left_edge = secondary_left ? layout_left : primary_left + primary_rect_width;
    const u32 secondary_top_edge =
        secondary_top ? layout_top : layout_top + layout_height - secondary_rect_height;

    const Common::Rectangle<u32> primary_screen{primary_left, layout_top,
                                                primary_left + primary_rect_width,
                                                layout_top + primary_rect_height};
    const Common::Rectangle<u32> secondary_screen{
        secondary_left_edge, secondary_top_edge,
        secondary_left_edge + secondary_rect_width,
        secondary_top_edge + secondary_rect_height};

    res.top_screen = swapped ? secondary_screen : primary_screen;
    res.bottom_screen = swapped ? primary_screen : secondary_screen;
    return res;
}

FramebufferLayout LargeFrameLayoutTopAndroid(u32 width, u32 height, bool swapped,
                                             float scale_ratio, bool secondary_left,
                                             bool secondary_top) {
    FramebufferLayout res{width, height, true, true, {}, {}, false, false, {}};

    const float clamped_ratio = std::clamp(scale_ratio, 0.25f, 1.0f);
    const float primary_width = swapped ? Core::kScreenBottomWidth : Core::kScreenTopWidth;
    const float primary_height = swapped ? Core::kScreenBottomHeight : Core::kScreenTopHeight;
    const float secondary_width = swapped ? Core::kScreenTopWidth : Core::kScreenBottomWidth;
    const float secondary_height = swapped ? Core::kScreenTopHeight : Core::kScreenBottomHeight;
    Common::Rectangle<u32> screen_area{0, 0, width, height};
    Common::Rectangle<u32> primary_screen = maxRectangle(screen_area, primary_height / primary_width);
    primary_screen = primary_screen.TranslateY((height - primary_screen.GetHeight()) / 2);

    const float primary_scale = static_cast<float>(primary_screen.GetHeight()) / primary_height;
    const u32 requested_secondary_width =
        static_cast<u32>(std::round(secondary_width * clamped_ratio * primary_scale));
    const u32 requested_secondary_height =
        static_cast<u32>(std::round(secondary_height * clamped_ratio * primary_scale));
    const u32 remaining_width =
        width > primary_screen.GetWidth() ? width - primary_screen.GetWidth() : 0;

    if (requested_secondary_width == 0 || requested_secondary_height == 0) {
        return LargeFrameLayoutTop(width, height, swapped, scale_ratio, secondary_left,
                                   secondary_top);
    }

    Common::Rectangle<u32> secondary_screen{};

    if (requested_secondary_width <= remaining_width) {
        const u32 primary_left = secondary_left ? requested_secondary_width : 0;
        primary_screen = primary_screen.TranslateX(primary_left);
        const u32 secondary_left_edge = secondary_left ? 0 : primary_screen.right;
        const u32 secondary_top_edge =
            secondary_top ? primary_screen.top : primary_screen.bottom - requested_secondary_height;
        secondary_screen = {secondary_left_edge, secondary_top_edge,
                            secondary_left_edge + requested_secondary_width,
                            secondary_top_edge + requested_secondary_height};
    } else {
        const float total_width = primary_width + (secondary_width * clamped_ratio);
        const float total_height = std::max(primary_height, secondary_height * clamped_ratio);
        const float base_scale =
            std::min(static_cast<float>(width) / total_width,
                     static_cast<float>(height) / total_height);

        const u32 primary_rect_width = static_cast<u32>(std::round(primary_width * base_scale));
        const u32 primary_rect_height = static_cast<u32>(std::round(primary_height * base_scale));
        const u32 secondary_rect_width =
            static_cast<u32>(std::round(secondary_width * clamped_ratio * base_scale));
        const u32 secondary_rect_height =
            static_cast<u32>(std::round(secondary_height * clamped_ratio * base_scale));
        const u32 layout_height = std::max(primary_rect_height, secondary_rect_height);
        const u32 layout_top = (height - layout_height) / 2;
        const u32 primary_left = secondary_left ? secondary_rect_width : 0;
        const u32 secondary_left_edge = secondary_left ? 0 : primary_left + primary_rect_width;
        const u32 secondary_top_edge =
            secondary_top ? layout_top : layout_top + layout_height - secondary_rect_height;

        primary_screen = {primary_left, layout_top, primary_left + primary_rect_width,
                          layout_top + primary_rect_height};
        secondary_screen = {secondary_left_edge, secondary_top_edge,
                            secondary_left_edge + secondary_rect_width,
                            secondary_top_edge + secondary_rect_height};
    }

    res.top_screen = swapped ? secondary_screen : primary_screen;
    res.bottom_screen = swapped ? primary_screen : secondary_screen;
    return res;
}

u16 GetLargeFrameLayoutTopAndroidMaxFillProportion(u32 width, u32 height, bool swapped,
                                                   u32 min_left, u32 min_top, u32 min_right,
                                                   u32 min_bottom) {
    if (width == 0 || height == 0 || width <= min_left + min_right ||
        height <= min_top + min_bottom) {
        return 75;
    }

    const u32 available_width = width - min_left - min_right;
    const u32 available_height = height - min_top - min_bottom;
    const float primary_height = swapped ? Core::kScreenBottomHeight : Core::kScreenTopHeight;
    const float secondary_width = swapped ? Core::kScreenTopWidth : Core::kScreenBottomWidth;
    Common::Rectangle<u32> screen_area{0, 0, available_width, available_height};
    Common::Rectangle<u32> primary_screen =
        maxRectangle(screen_area, (swapped ? BOT_SCREEN_ASPECT_RATIO : TOP_SCREEN_ASPECT_RATIO));
    const u32 remaining_width =
        available_width > primary_screen.GetWidth() ? available_width - primary_screen.GetWidth()
                                                    : 0;

    if (remaining_width == 0 || primary_screen.GetHeight() == 0 || secondary_width == 0.0f) {
        return 25;
    }

    const float primary_scale = static_cast<float>(primary_screen.GetHeight()) / primary_height;
    const float proportion = static_cast<float>(remaining_width) / (secondary_width * primary_scale);
    return static_cast<u16>(std::clamp(std::round(proportion * 100.0f), 25.0f, 100.0f));
}

FramebufferLayout HybridFrameLayout(u32 width, u32 height, bool swapped,
                                    bool side_column_left, bool secondary_screen_top) {
    FramebufferLayout res{width, height, true, true, {}, {}, true, !swapped, {}};

    const float primary_width = swapped ? Core::kScreenBottomWidth : Core::kScreenTopWidth;
    const float primary_height = swapped ? Core::kScreenBottomHeight : Core::kScreenTopHeight;
    const float secondary_width = swapped ? Core::kScreenTopWidth : Core::kScreenBottomWidth;
    const float secondary_height = swapped ? Core::kScreenTopHeight : Core::kScreenBottomHeight;
    const float primary_aspect_ratio = primary_height / primary_width;
    const float secondary_aspect_ratio = secondary_height / secondary_width;

    const float layout_height_units = primary_aspect_ratio + secondary_aspect_ratio;
    const float primary_width_units = layout_height_units / primary_aspect_ratio;
    const float layout_width_units = primary_width_units + 1.0f;
    const float scale = std::min(static_cast<float>(width) / layout_width_units,
                                 static_cast<float>(height) / layout_height_units);

    if (width == 0 || height == 0 || scale < 1.0f) {
        return LargeFrameLayout(width, height, swapped);
    }

    auto compute_dimensions = [&](u32 column_width) {
        const u32 duplicated_primary_height =
            std::max<u32>(1, static_cast<u32>(std::round(column_width * primary_aspect_ratio)));
        const u32 secondary_screen_height =
            std::max<u32>(1, static_cast<u32>(std::round(column_width * secondary_aspect_ratio)));
        const u32 large_primary_height = duplicated_primary_height + secondary_screen_height;
        const u32 large_primary_width = std::max<u32>(
            1, static_cast<u32>(std::round(static_cast<float>(large_primary_height) /
                                           primary_aspect_ratio)));
        return std::array<u32, 4>{duplicated_primary_height, secondary_screen_height,
                                  large_primary_height, large_primary_width};
    };

    u32 side_column_width = std::max<u32>(1, static_cast<u32>(std::floor(scale)));
    auto dimensions = compute_dimensions(side_column_width);
    while (side_column_width > 1 &&
           (dimensions[3] + side_column_width > width || dimensions[2] > height)) {
        --side_column_width;
        dimensions = compute_dimensions(side_column_width);
    }

    const u32 duplicated_primary_height = dimensions[0];
    const u32 secondary_screen_height = dimensions[1];
    const u32 large_primary_height = dimensions[2];
    const u32 large_primary_width = dimensions[3];
    const u32 layout_width = large_primary_width + side_column_width;
    const u32 layout_left = width > layout_width ? (width - layout_width) / 2 : 0;
    const u32 layout_top =
        height > large_primary_height ? (height - large_primary_height) / 2 : 0;

    const u32 side_column_left_edge = side_column_left ? layout_left : layout_left + large_primary_width;
    const u32 primary_left = side_column_left ? layout_left + side_column_width : layout_left;
    const Common::Rectangle<u32> primary_screen{
        primary_left, layout_top, primary_left + large_primary_width,
        layout_top + large_primary_height};
    const Common::Rectangle<u32> top_side_screen{
        side_column_left_edge, layout_top, side_column_left_edge + side_column_width,
        layout_top + (secondary_screen_top ? secondary_screen_height : duplicated_primary_height)};
    const Common::Rectangle<u32> bottom_side_screen{
        side_column_left_edge, top_side_screen.bottom, side_column_left_edge + side_column_width,
        top_side_screen.bottom +
            (secondary_screen_top ? duplicated_primary_height : secondary_screen_height)};
    const Common::Rectangle<u32> secondary_screen =
        secondary_screen_top ? top_side_screen : bottom_side_screen;
    const Common::Rectangle<u32> duplicated_primary_screen =
        secondary_screen_top ? bottom_side_screen : top_side_screen;

    res.top_screen = swapped ? secondary_screen : primary_screen;
    res.bottom_screen = swapped ? primary_screen : secondary_screen;
    res.additional_screen = duplicated_primary_screen;
    return res;
}

FramebufferLayout SideFrameLayout(u32 width, u32 height, bool swapped) {
    FramebufferLayout res{width, height, true, true, {}, {}, false, false, {}};
    const float window_aspect_ratio = static_cast<float>(height) / width;

    if (height > width + width / 2) {
        Common::Rectangle<u32> screen_area{0, 0, width, height};
        Common::Rectangle<u32> top_screen = maxRectangle(screen_area, TOP_SCREEN_ASPECT_RATIO);
        Common::Rectangle<u32> bot_screen{top_screen.left, top_screen.top, top_screen.right, static_cast<u32>(top_screen.GetWidth() * BOT_SCREEN_ASPECT_RATIO)};
        res.top_screen = swapped ? top_screen.TranslateY(bot_screen.GetHeight()) : top_screen;
        res.bottom_screen = swapped ? bot_screen : bot_screen.TranslateY(top_screen.GetHeight());
    } else {
        // Aspect ratio of both screens side by side
        const float emulation_aspect_ratio = static_cast<float>(Core::kScreenTopHeight) / (Core::kScreenTopWidth + Core::kScreenBottomWidth);
        Common::Rectangle<u32> screen_area{0, 0, width, height};
        // Find largest Rectangle that can fit in the window size with the given aspect ratio
        Common::Rectangle<u32> screen_rect = maxRectangle(screen_area, emulation_aspect_ratio);
        // Find sizes of top and bottom screen
        Common::Rectangle<u32> top_screen = maxRectangle(screen_rect, TOP_SCREEN_ASPECT_RATIO);
        Common::Rectangle<u32> bot_screen = maxRectangle(screen_rect, BOT_SCREEN_ASPECT_RATIO);

        if (window_aspect_ratio < emulation_aspect_ratio) {
            // Apply borders to the left and right sides of the window.
            u32 shift_horizontal = (screen_area.GetWidth() - screen_rect.GetWidth()) / 2;
            top_screen = top_screen.TranslateX(shift_horizontal);
            bot_screen = bot_screen.TranslateX(shift_horizontal);
        } else {
            // Window is narrower than the emulation content => apply borders to the top and bottom
            u32 shift_vertical = (screen_area.GetHeight() - screen_rect.GetHeight()) / 2;
            top_screen = top_screen.TranslateY(shift_vertical);
            bot_screen = bot_screen.TranslateY(shift_vertical);
        }
        // Move the top screen to the right if we are swapped.
        res.top_screen = swapped ? top_screen.TranslateX(bot_screen.GetWidth()) : top_screen;
        res.bottom_screen = swapped ? bot_screen : bot_screen.TranslateX(top_screen.GetWidth());
    }

    return res;
}

FramebufferLayout CustomFrameLayout(u32 width, u32 height) {
    FramebufferLayout res{width, height, true, true, {}, {}, false, false, {}};

    Common::Rectangle<u32> top_screen{
        Settings::values.custom_top_left, Settings::values.custom_top_top,
        Settings::values.custom_top_right, Settings::values.custom_top_bottom};
    Common::Rectangle<u32> bot_screen{
        Settings::values.custom_bottom_left, Settings::values.custom_bottom_top,
        Settings::values.custom_bottom_right, Settings::values.custom_bottom_bottom};

    res.top_screen = top_screen;
    res.bottom_screen = bot_screen;
    return res;
}

} // namespace Layout
