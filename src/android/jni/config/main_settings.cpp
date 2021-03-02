
#include "config/main_settings.h"
#include "core/settings.h"

namespace Config {

// core
const ConfigInfo<bool> USE_CPU_JIT{{"Core", "use_cpu_jit"}, true};
const ConfigInfo<bool> IS_NEW_3DS{{"Core", "is_new_3ds"}, false};
const ConfigInfo<bool> USE_VIRTUAL_SD{{"Core", "use_virtual_sd"}, true};
const ConfigInfo<int> SYSTEM_REGION{{"Core", "region_value"}, Settings::REGION_VALUE_AUTO_SELECT};
const ConfigInfo<Service::CFG::SystemLanguage> SYSTEM_LANGUAGE{{"Core", "language"}, Service::CFG::SystemLanguage::LANGUAGE_EN};

// renderer
const ConfigInfo<bool> USE_GLES{{"Renderer", "use_gles"}, true};
const ConfigInfo<bool> SHOW_FPS{{"Renderer", "show_fps"}, true};
const ConfigInfo<bool> USE_HW_RENDERER{{"Renderer", "use_hw_renderer"}, true};
const ConfigInfo<bool> USE_HW_SHADER{{"Renderer", "use_hw_shader"}, true};
const ConfigInfo<bool> USE_SHADER_JIT{{"Renderer", "use_shader_jit"}, false};
const ConfigInfo<Settings::AccurateMul> SHADERS_ACCURATE_MUL{{"Renderer", "accurate_mul_type"}, Settings::AccurateMul::OFF};
const ConfigInfo<u16> RESOLUTION_FACTOR{{"Renderer", "resolution_factor"}, 1};
const ConfigInfo<bool> USE_FRAME_LIMIT{{"Renderer", "use_frame_limit"}, true};
const ConfigInfo<u16> FRAME_LIMIT{{"Renderer", "frame_limit"}, 100};
const ConfigInfo<u8> FACTOR_3D{{"Renderer", "factor_3d"}, 0};
const ConfigInfo<bool> CUSTOM_TEXTURES{{"Renderer", "custom_textures"}, false};
const ConfigInfo<bool> PRELOAD_TEXTURES{{"Renderer", "preload_textures"}, false};
const ConfigInfo<Settings::LayoutOption> LAYOUT_OPTION{{"Renderer", "layout_option"},
                                                       Settings::LayoutOption::Default};
const ConfigInfo<std::string> POST_PROCESSING_SHADER{{"Renderer", "pp_shader_name"}, ""};

// audio
const ConfigInfo<bool> ENABLE_DSP_LLE{{"Audio", "enable_dsp_lle"}, false};
const ConfigInfo<bool> DSP_LLE_MULTITHREAD{{"Audio", "enable_dsp_lle_multithread"}, true};
const ConfigInfo<bool> AUDIO_STRETCHING{{"Audio", "enable_audio_stretching"}, false};
const ConfigInfo<float> AUDIO_VOLUME{{"Audio", "volume"}, 1.0F};
const ConfigInfo<std::string> AUDIO_ENGINE{{"Audio", "output_engine"}, "auto"};
const ConfigInfo<std::string> AUDIO_DEVICE{{"Audio", "output_device"}, "auto"};

// mic
const ConfigInfo<Settings::MicInputType> MIC_INPUT_TYPE{{"Audio", "mic_input_type"},
                                                        Settings::MicInputType::None};
const ConfigInfo<std::string> MIC_INPUT_DEVICE{{"Audio", "mic_input_device"}, "Default"};

// camera
const ConfigInfo<std::string> CAMERA_DEVICE{{"Camera", "camera_type"}, "blank"};

// debug
const ConfigInfo<bool> ALLOW_SHADOW{{"Debug", "allow_shadow"}, false};
const ConfigInfo<u8> SHADER_TYPE{{"Debug", "shader_type"}, 1};
const ConfigInfo<bool> USE_PRESENT_THREAD{{"Debug", "use_present_thread"}, true};
const ConfigInfo<bool> CPU_USAGE_LIMIT{{"Debug", "cpu_usage_limit"}, false};
const ConfigInfo<std::string> LLE_MODULES{{"Debug", "lle_modules"}, ""};
const ConfigInfo<std::string> BAIDU_OCR_KEY{{"Debug", "baidu_ocr_key"}, ""};
const ConfigInfo<std::string> BAIDU_OCR_SECRET{{"Debug", "baidu_ocr_secret"}, ""};

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
const ConfigInfo<std::string> BUTTON_HOME{{"Controls", "button_home"}, "code:100"};
const ConfigInfo<std::string> CIRCLE_PAD_X{{"Controls", "circle_pad_x"}, "code:0"};
const ConfigInfo<std::string> CIRCLE_PAD_Y{{"Controls", "circle_pad_y"}, "code:1"};
const ConfigInfo<std::string> C_STICK_X{{"Controls", "c_stick_x"}, "code:11"};
const ConfigInfo<std::string> C_STICK_Y{{"Controls", "c_stick_y"}, "code:14"};

// custom layout
const ConfigInfo<bool> USE_CUSTOM_LAYOUT{{"Layout", "custom_layout"}, false};
const ConfigInfo<u16> CUSTOM_TOP_LEFT{{"Layout", "custom_top_left"}, 0};
const ConfigInfo<u16> CUSTOM_TOP_TOP{{"Layout", "custom_top_top"}, 0};
const ConfigInfo<u16> CUSTOM_TOP_RIGHT{{"Layout", "custom_top_right"}, 800};
const ConfigInfo<u16> CUSTOM_TOP_BOTTOM{{"Layout", "custom_top_bottom"}, 480};
const ConfigInfo<u16> CUSTOM_BOTTOM_LEFT{{"Layout", "custom_bottom_left"}, 80};
const ConfigInfo<u16> CUSTOM_BOTTOM_TOP{{"Layout", "custom_bottom_top"}, 480};
const ConfigInfo<u16> CUSTOM_BOTTOM_RIGHT{{"Layout", "custom_bottom_right"}, 720};
const ConfigInfo<u16> CUSTOM_BOTTOM_BOTTOM{{"Layout", "custom_bottom_bottom"}, 960};

} // namespace Config
