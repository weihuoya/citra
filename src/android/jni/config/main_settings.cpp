
#include "config/main_settings.h"
#include "core/settings.h"

namespace Config {

// core
const ConfigInfo<bool> USE_CPU_JIT{{"Core", "use_cpu_jit"}, true};
const ConfigInfo<bool> IS_NEW_3DS{{"Core", "is_new_3ds"}, false};
const ConfigInfo<bool> USE_GAME_CONFIG{{"Core", "use_game_config"}, false};
const ConfigInfo<bool> USE_VIRTUAL_SD{{"Core", "use_virtual_sd"}, true};
const ConfigInfo<int> SYSTEM_REGION{{"Core", "region_value"}, Settings::REGION_VALUE_AUTO_SELECT};
const ConfigInfo<Service::CFG::SystemLanguage> SYSTEM_LANGUAGE{
    {"Core", "language"}, Service::CFG::SystemLanguage::LANGUAGE_EN};
const ConfigInfo<Settings::SharedFontType> SHARED_FONT_TYPE{{"Core", "shared_font_type"},
                                                            Settings::SharedFontType::Auto};

// renderer
const ConfigInfo<bool> USE_GLES{{"Renderer", "use_gles"}, true};
const ConfigInfo<bool> SHOW_FPS{{"Renderer", "show_fps"}, true};
const ConfigInfo<bool> USE_HW_SHADER{{"Renderer", "use_hw_shader"}, true};
const ConfigInfo<bool> USE_SHADER_JIT{{"Renderer", "use_shader_jit"}, false};
const ConfigInfo<Settings::AccurateMul> SHADERS_ACCURATE_MUL{{"Renderer", "accurate_mul_type"},
                                                             Settings::AccurateMul::OFF};
const ConfigInfo<u16> RESOLUTION_FACTOR{{"Renderer", "resolution_factor"}, 1};
const ConfigInfo<bool> USE_FRAME_LIMIT{{"Renderer", "use_frame_limit"}, true};
const ConfigInfo<u16> FRAME_LIMIT{{"Renderer", "frame_limit"}, 100};
const ConfigInfo<u8> FACTOR_3D{{"Renderer", "factor_3d"}, 0};
const ConfigInfo<bool> USE_FENCE_SYNC{{"Renderer", "use_fence_sync"}, false};
const ConfigInfo<bool> CUSTOM_TEXTURES{{"Renderer", "custom_textures"}, false};
const ConfigInfo<Settings::LayoutOption> LAYOUT_OPTION{{"Renderer", "layout_option"},
                                                       Settings::LayoutOption::Default};
const ConfigInfo<Settings::LayoutOption> LANDSCAPE_LAYOUT_OPTION{
    {"Renderer", "landscape_layout_option"}, Settings::LayoutOption::Default};
const ConfigInfo<Settings::PresentationMode> SCREEN_PRESENTATION_MODE{
        {"Renderer", "screen_presentation_mode"}, Settings::PresentationMode::Default};
const ConfigInfo<std::string> POST_PROCESSING_SHADER{{"Renderer", "pp_shader_name"}, ""};
const ConfigInfo<std::string> REMOTE_SHADER_HOST{
    {"Renderer", "remote_shader_host"},
    "https://raw.githubusercontent.com/weihuoya/citra/master/cache/"};

// audio
const ConfigInfo<bool> ENABLE_DSP_LLE{{"Audio", "enable_dsp_lle"}, false};
const ConfigInfo<bool> DSP_LLE_MULTITHREAD{{"Audio", "dsp_lle_multithread"}, true};
const ConfigInfo<bool> AUDIO_STRETCHING{{"Audio", "enable_audio_stretching"}, false};
const ConfigInfo<float> AUDIO_VOLUME{{"Audio", "audio_volume"}, 1.0F};
const ConfigInfo<float> MIC_VOLUME{{"Audio", "mic_volume"}, 1.5F};
const ConfigInfo<u8> AUDIO_OUTPUT_TYPE{{"Audio", "audio_output_type"}, 2};
const ConfigInfo<std::string> AUDIO_OUTPUT_DEVICE{{"Audio", "audio_output_device"}, "auto"};

// mic
const ConfigInfo<u8> AUDIO_INPUT_TYPE{{"Audio", "audio_input_type"}, 1};
const ConfigInfo<std::string> AUDIO_INPUT_DEVICE{{"Audio", "audio_input_device"}, "auto"};

// camera
const ConfigInfo<std::string> CAMERA_DEVICE{{"Camera", "camera_type"}, "blank"};

// debug
const ConfigInfo<u8> HW_GS_MODE{{"Debug", "hw_gs_mode"}, 2};
const ConfigInfo<u8> SHADER_TYPE{{"Debug", "shader_type"}, 1};
const ConfigInfo<bool> USE_FMV_HACK{{"Debug", "use_fmv_hack"}, false};
const ConfigInfo<bool> SKIP_SLOW_DRAW{{"Debug", "skip_slow_draw"}, false};
const ConfigInfo<bool> SKIP_CPU_WRITE{{"Debug", "skip_cpu_write"}, false};
const ConfigInfo<bool> SKIP_TEXTURE_COPY{{"Debug", "skip_texture_copy"}, false};
const ConfigInfo<u8> FORCE_TEXTURE_FILTER{{"Debug", "force_texture_filter"}, 0};
const ConfigInfo<bool> ASYNC_SHADER_COMPILE{{"Debug", "async_shader_compile"}, false};
const ConfigInfo<bool> USE_COMPATIBLE_MODE{{"Debug", "use_compatible_mode"}, false};
const ConfigInfo<bool> USE_PRESENT_THREAD{{"Debug", "use_present_thread"}, true};
const ConfigInfo<bool> SHADOW_RENDERING{{"Debug", "shadow_rendering"}, true};
const ConfigInfo<bool> CPU_USAGE_LIMIT{{"Debug", "cpu_usage_limit"}, false};
const ConfigInfo<std::string> LLE_MODULES{{"Debug", "lle_modules"}, ""};

// controls
const ConfigInfo<std::string> BUTTON_A{{"Controls", "button_a"}, "code:96"};
const ConfigInfo<std::string> BUTTON_B{{"Controls", "button_b"}, "code:97"};
const ConfigInfo<std::string> BUTTON_X{{"Controls", "button_x"}, "code:99"};
const ConfigInfo<std::string> BUTTON_Y{{"Controls", "button_y"}, "code:100"};
const ConfigInfo<std::string> BUTTON_UP{{"Controls", "button_up"}, "code:16,dir:-"};
const ConfigInfo<std::string> BUTTON_DOWN{{"Controls", "button_down"}, "code:16,dir:+"};
const ConfigInfo<std::string> BUTTON_LEFT{{"Controls", "button_left"}, "code:15,dir:-"};
const ConfigInfo<std::string> BUTTON_RIGHT{{"Controls", "button_right"}, "code:15,dir:+"};
const ConfigInfo<std::string> BUTTON_L{{"Controls", "button_l"}, "code:102"};
const ConfigInfo<std::string> BUTTON_R{{"Controls", "button_r"}, "code:103"};
const ConfigInfo<std::string> BUTTON_START{{"Controls", "button_start"}, "code:108"};
const ConfigInfo<std::string> BUTTON_SELECT{{"Controls", "button_select"}, "code:109"};
const ConfigInfo<std::string> BUTTON_DEBUG{{"Controls", "button_debug"}, "code:111"};
const ConfigInfo<std::string> BUTTON_GPIO14{{"Controls", "button_gpio14"}, "code:112"};
const ConfigInfo<std::string> BUTTON_ZL{{"Controls", "button_zl"}, "code:104"};
const ConfigInfo<std::string> BUTTON_ZR{{"Controls", "button_zr"}, "code:105"};
const ConfigInfo<std::string> BUTTON_HOME{{"Controls", "button_home"}, "code:113"};
const ConfigInfo<std::string> BUTTON_POWER{{"Controls", "button_power"}, "code:114"};
const ConfigInfo<std::string> BUTTON_COMBO_1{{"Controls", "button_combo_1"}, "code:115"};
const ConfigInfo<std::string> BUTTON_COMBO_2{{"Controls", "button_combo_2"}, "code:116"};
const ConfigInfo<std::string> BUTTON_COMBO_3{{"Controls", "button_combo_3"}, "code:117"};
const ConfigInfo<std::string> CIRCLE_PAD_UP{{"Controls", "circle_pad_up"}, "code:1,dir:-"};
const ConfigInfo<std::string> CIRCLE_PAD_DOWN{{"Controls", "circle_pad_down"}, "code:1,dir:+"};
const ConfigInfo<std::string> CIRCLE_PAD_LEFT{{"Controls", "circle_pad_left"}, "code:0,dir:-"};
const ConfigInfo<std::string> CIRCLE_PAD_RIGHT{{"Controls", "circle_pad_right"}, "code:0,dir:+"};
const ConfigInfo<std::string> C_STICK_UP{{"Controls", "c_stick_up"}, "code:14,dir:-"};
const ConfigInfo<std::string> C_STICK_DOWN{{"Controls", "c_stick_down"}, "code:14,dir:+"};
const ConfigInfo<std::string> C_STICK_LEFT{{"Controls", "c_stick_left"}, "code:11,dir:-"};
const ConfigInfo<std::string> C_STICK_RIGHT{{"Controls", "c_stick_right"}, "code:11,dir:+"};
const ConfigInfo<std::string> COMBO_KEY_0{{"Controls", "combo_key_0"}, ""};
const ConfigInfo<std::string> COMBO_KEY_1{{"Controls", "combo_key_1"}, ""};
const ConfigInfo<std::string> COMBO_KEY_2{{"Controls", "combo_key_2"}, ""};
const ConfigInfo<u16> INPUT_OVERLAY_ALPHA{{"Controls", "input_overlay_alpha"}, 100};
const ConfigInfo<u16> INPUT_OVERLAY_SCALE{{"Controls", "input_overlay_scale"}, 40};
const ConfigInfo<bool> INPUT_OVERLAY_FEEDBACK{{"Controls", "input_overlay_feedback"}, true};
const ConfigInfo<bool> INPUT_OVERLAY_HIDE{{"Controls", "input_overlay_hide"}, false};
const ConfigInfo<bool> INPUT_JOYSTICK_RELATIVE{{"Controls", "input_joystick_relative"}, true};
const ConfigInfo<u32> INPUT_JOYSTICK_RANGE{{"Controls", "input_joystick_range"}, 100};
const ConfigInfo<u32> INPUT_JOYSTICK_DEADZONE{{"Controls", "input_joystick_deadzone"}, 0};

// custom layout
const ConfigInfo<bool> PORTRAIT_CUSTOM_LAYOUT{{"Layout", "portrait_custom_layout"}, false};
const ConfigInfo<bool> LANDSCAPE_CUSTOM_LAYOUT{{"Layout", "landscape_custom_layout"}, false};

const ConfigInfo<bool> PORTRAIT_SWAP_SCREEN{{"Layout", "portrait_swap_screen"}, false};
const ConfigInfo<bool> LANDSCAPE_SWAP_SCREEN{{"Layout", "landscape_swap_screen"}, false};

const ConfigInfo<u16> PORTRAIT_TOP_LEFT{{"Layout", "portrait_top_left"}, 0};
const ConfigInfo<u16> PORTRAIT_TOP_TOP{{"Layout", "portrait_top_top"}, 0};
const ConfigInfo<u16> PORTRAIT_TOP_RIGHT{{"Layout", "portrait_top_right"}, 800};
const ConfigInfo<u16> PORTRAIT_TOP_BOTTOM{{"Layout", "portrait_top_bottom"}, 480};
const ConfigInfo<u16> PORTRAIT_BOTTOM_LEFT{{"Layout", "portrait_bottom_left"}, 80};
const ConfigInfo<u16> PORTRAIT_BOTTOM_TOP{{"Layout", "portrait_bottom_top"}, 480};
const ConfigInfo<u16> PORTRAIT_BOTTOM_RIGHT{{"Layout", "portrait_bottom_right"}, 720};
const ConfigInfo<u16> PORTRAIT_BOTTOM_BOTTOM{{"Layout", "portrait_bottom_bottom"}, 960};

const ConfigInfo<u16> LANDSCAPE_TOP_LEFT{{"Layout", "landscape_top_left"}, 0};
const ConfigInfo<u16> LANDSCAPE_TOP_TOP{{"Layout", "landscape_top_top"}, 0};
const ConfigInfo<u16> LANDSCAPE_TOP_RIGHT{{"Layout", "landscape_top_right"}, 800};
const ConfigInfo<u16> LANDSCAPE_TOP_BOTTOM{{"Layout", "landscape_top_bottom"}, 480};
const ConfigInfo<u16> LANDSCAPE_BOTTOM_LEFT{{"Layout", "landscape_bottom_left"}, 80};
const ConfigInfo<u16> LANDSCAPE_BOTTOM_TOP{{"Layout", "landscape_bottom_top"}, 480};
const ConfigInfo<u16> LANDSCAPE_BOTTOM_RIGHT{{"Layout", "landscape_bottom_right"}, 720};
const ConfigInfo<u16> LANDSCAPE_BOTTOM_BOTTOM{{"Layout", "landscape_bottom_bottom"}, 960};

} // namespace Config
