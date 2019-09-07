#pragma once

#include <string>

#include "config/config.h"

namespace Settings {
enum class LayoutOption;
} // namespace Settings

namespace Config {

// core
extern const ConfigInfo<bool> USE_CPU_JIT;
extern const ConfigInfo<bool> IS_NEW_3DS;
extern const ConfigInfo<bool> USE_VIRTUAL_SD;
extern const ConfigInfo<int> SYSTEM_REGION;

// renderer
extern const ConfigInfo<bool> USE_GLES;
extern const ConfigInfo<bool> SHOW_FPS;
extern const ConfigInfo<bool> USE_HW_RENDERER;
extern const ConfigInfo<bool> USE_HW_SHADER;
extern const ConfigInfo<bool> USE_SHADER_JIT;
extern const ConfigInfo<bool> SHADERS_ACCURATE_MUL;
extern const ConfigInfo<u16> RESOLUTION_FACTOR;
extern const ConfigInfo<bool> USE_FRAME_LIMIT;
extern const ConfigInfo<u16> FRAME_LIMIT;
extern const ConfigInfo<u8> FACTOR_3D;
extern const ConfigInfo<Settings::LayoutOption> LAYOUT_OPTION;
extern const ConfigInfo<std::string> POST_PROCESSING_SHADER;

// audio
extern const ConfigInfo<bool> ENABLE_DSP_LLE;
extern const ConfigInfo<bool> DSP_LLE_MULTITHREAD;
extern const ConfigInfo<bool> AUDIO_STRETCHING;
extern const ConfigInfo<float> AUDIO_VOLUME;
extern const ConfigInfo<std::string> AUDIO_ENGINE;
extern const ConfigInfo<std::string> AUDIO_DEVICE;

// debug
extern const ConfigInfo<bool> ALLOW_SHADOW;
extern const ConfigInfo<bool> DUMP_TEXTURES;

// controls
extern const ConfigInfo<std::string> BUTTON_A;
extern const ConfigInfo<std::string> BUTTON_B;
extern const ConfigInfo<std::string> BUTTON_X;
extern const ConfigInfo<std::string> BUTTON_Y;
extern const ConfigInfo<std::string> BUTTON_UP;
extern const ConfigInfo<std::string> BUTTON_DOWN;
extern const ConfigInfo<std::string> BUTTON_LEFT;
extern const ConfigInfo<std::string> BUTTON_RIGHT;
extern const ConfigInfo<std::string> BUTTON_L;
extern const ConfigInfo<std::string> BUTTON_R;
extern const ConfigInfo<std::string> BUTTON_START;
extern const ConfigInfo<std::string> BUTTON_SELECT;
extern const ConfigInfo<std::string> BUTTON_DEBUG;
extern const ConfigInfo<std::string> BUTTON_GPIO14;
extern const ConfigInfo<std::string> BUTTON_ZL;
extern const ConfigInfo<std::string> BUTTON_ZR;
extern const ConfigInfo<std::string> BUTTON_HOME;

} // namespace Config