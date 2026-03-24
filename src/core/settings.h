// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <array>
#include <atomic>
#include <string>
#include <unordered_map>
#include <vector>
#include "common/common_types.h"
#include "core/hle/service/cam/cam.h"

namespace Settings {

enum class InitClock {
    SystemTime = 0,
    FixedTime = 1,
};

enum class LayoutOption {
    Default,
    SingleScreen,
    LargeScreen,
    SideScreen,
    LargeScreenTop,
    HybridScreen,
};

enum class MicInputType {
    None,
    Real,
    Static,
};

enum class StereoRenderOption {
    Off = 0,
    SideBySide = 1,
    Anaglyph = 2,
    Interlaced = 3,
};

enum class PresentationMode {
    Default,
    SingleScreen,
};

enum class HybridFit {
    Horizontal = 0,
    Vertical = 1,
};

enum class AccurateMul {
    OFF = 0,
    FAST = 1,
    SAFE = 2,
};

enum class SharedFontType {
    Auto = -1,
    Legacy = 0,
    JPN = 1,
    CHN = 2,
    KOR = 3,
    TWN = 4,
};

namespace NativeButton {
enum Values {
    A,
    B,
    X,
    Y,
    Up,
    Down,
    Left,
    Right,
    L,
    R,
    Start,
    Select,
    Debug,
    Gpio14,

    ZL,
    ZR,

    Home,
    Power,

    NumButtons,
};

constexpr int BUTTON_HID_BEGIN = A;
constexpr int BUTTON_IR_BEGIN = ZL;
constexpr int BUTTON_NS_BEGIN = Home;

constexpr int BUTTON_HID_END = BUTTON_IR_BEGIN;
constexpr int BUTTON_IR_END = BUTTON_NS_BEGIN;
constexpr int BUTTON_NS_END = NumButtons;

constexpr int NUM_BUTTONS_HID = BUTTON_HID_END - BUTTON_HID_BEGIN;
constexpr int NUM_BUTTONS_IR = BUTTON_IR_END - BUTTON_IR_BEGIN;
constexpr int NUM_BUTTONS_NS = BUTTON_NS_END - BUTTON_NS_BEGIN;

const std::array mapping = {
    "button_a",
    "button_b",
    "button_x",
    "button_y",
    "button_up",
    "button_down",
    "button_left",
    "button_right",
    "button_l",
    "button_r",
    "button_start",
    "button_select",
    "button_debug",
    "button_gpio14",
    "button_zl",
    "button_zr",
    "button_home",
};
} // namespace NativeButton

namespace NativeAnalog {
enum Values {
    CirclePad,
    CStick,

    NumAnalogs,
};

const std::array mapping = {
    "circle_pad",
    "c_stick",
};
} // namespace NativeAnalog

struct InputProfile {
    std::string name;
    std::array<std::string, NativeButton::NumButtons> buttons;
    std::array<std::string, NativeAnalog::NumAnalogs> analogs;
    std::string motion_device;
    std::string touch_device;
    std::string udp_input_address;
    u16 udp_input_port;
    u8 udp_pad_index;
};

struct Values {
    // CheckNew3DS
    bool is_new_3ds;

    // Controls
    InputProfile current_input_profile;       ///< The current input profile
    int current_input_profile_index;          ///< The current input profile index
    std::vector<InputProfile> input_profiles; ///< The list of input profiles

    // Core
    bool use_cpu_jit;

    // Data Storage
    bool use_virtual_sd;

    // System
    int region_value;
    InitClock init_clock;
    u64 init_time;

    // Renderer
    bool show_fps;
    bool use_gles;
    bool use_hw_renderer;
    bool use_hw_shader;
    bool use_shader_jit;
    bool filter_mode;
    bool allow_shadow;
    bool display_transfer_hack;
    bool use_linear_filter;
    u16 resolution_factor;
    StereoRenderOption render_3d;
    bool use_disk_shader_cache;
    bool vsync_enabled;
    bool use_frame_limit;
    u16 frame_limit;
    float bg_red;
    float bg_green;
    float bg_blue;
    std::string texture_filter_name;

    LayoutOption layout_option;
    bool swap_screen;
    bool upright_screen;
    u16 large_screen_proportion;
    bool large_screen_auto_fit;
    bool large_screen_secondary_left;
    bool large_screen_secondary_top;
    HybridFit hybrid_fit;
    bool hybrid_side_column_left;
    bool hybrid_secondary_top;
    u16 layout_margin_left;
    u16 layout_margin_top;
    u16 layout_margin_right;
    u16 layout_margin_bottom;
    bool custom_layout;
    u16 custom_top_left;
    u16 custom_top_top;
    u16 custom_top_right;
    u16 custom_top_bottom;
    u16 custom_bottom_left;
    u16 custom_bottom_top;
    u16 custom_bottom_right;
    u16 custom_bottom_bottom;

    std::atomic<u8> factor_3d;
    std::string pp_shader_name;

    bool dump_textures;
    bool custom_textures;
    bool preload_textures;

    // Audio
    bool enable_dsp_lle;
    bool enable_dsp_lle_multithread;
    std::string sink_id;
    bool enable_audio_stretching;
    std::string audio_device_id;
    float audio_volume;
    float mic_volume;
    MicInputType mic_input_type;
    std::string mic_input_device;

    // Camera
    std::array<std::string, Service::CAM::NumCameras> camera_name;
    std::array<std::string, Service::CAM::NumCameras> camera_config;
    std::array<int, Service::CAM::NumCameras> camera_flip;

    // Debugging
    bool use_gdbstub;
    u16 gdbstub_port;
    std::string log_filter;
    std::unordered_map<std::string, bool> lle_modules;

    s64 core_ticks_hack;
    bool core_downcount_hack;
    bool async_shader_compile;
    bool use_separable_shader;
    bool shadow_rendering;
    bool use_shader_cache;
    bool skip_slow_draw;
    bool skip_cpu_write;
    bool skip_texture_copy;
    bool skip_load_buffer;
    bool merge_framebuffer;
    bool disable_clip_coef;
    bool stream_buffer_hack;
    bool y2r_perform_hack;
    bool y2r_event_delay;
    bool use_present_thread;
    bool use_direct_display;
    bool use_fence_sync;
    bool accurate_max_min;
    bool accurate_rcp_rsq;
    bool use_compatible_mode;
    SharedFontType shared_font_type;
    AccurateMul shaders_accurate_mul;
    std::string remote_shader_host;
    u8 hw_gs_mode;
    u8 force_texture_filter;
    float joystick_range;
    float joystick_deadzone;

    // WebService
    bool enable_telemetry;
    std::string web_api_url;
    std::string citra_username;
    std::string citra_token;
} extern values;

// a special value for Values::region_value indicating that citra will automatically select a region
// value to fit the region lockout info of the game
static constexpr int REGION_VALUE_AUTO_SELECT = -1;

void Apply();
void LogSettings();

void SetFMVHack(bool enable);
void SetLLEModules(const std::string& modules);
void SwapScreenLayout();

// Input profiles
void LoadProfile(int index);
void SaveProfile(int index);
void CreateProfile(std::string name);
void DeleteProfile(int index);
void RenameCurrentProfile(std::string new_name);
} // namespace Settings
