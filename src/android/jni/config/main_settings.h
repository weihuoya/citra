#pragma once

#include <string>

#include "config/config.h"
#include "core/hle/service/cfg/cfg.h"

namespace Settings {
enum class LayoutOption;
enum class MicInputType;
enum class AccurateMul;
} // namespace Settings

namespace Config {

// core
extern const ConfigInfo<bool> USE_CPU_JIT;
extern const ConfigInfo<bool> IS_NEW_3DS;
extern const ConfigInfo<bool> USE_VIRTUAL_SD;
extern const ConfigInfo<int> SYSTEM_REGION;
extern const ConfigInfo<Service::CFG::SystemLanguage> SYSTEM_LANGUAGE;

// renderer
extern const ConfigInfo<bool> USE_GLES;
extern const ConfigInfo<bool> SHOW_FPS;
extern const ConfigInfo<bool> USE_HW_RENDERER;
extern const ConfigInfo<bool> USE_HW_SHADER;
extern const ConfigInfo<bool> USE_SHADER_JIT;
extern const ConfigInfo<Settings::AccurateMul> SHADERS_ACCURATE_MUL;
extern const ConfigInfo<u16> RESOLUTION_FACTOR;
extern const ConfigInfo<bool> USE_FRAME_LIMIT;
extern const ConfigInfo<u16> FRAME_LIMIT;
extern const ConfigInfo<u8> FACTOR_3D;
extern const ConfigInfo<bool> TEXTURE_LOAD_HACK;
extern const ConfigInfo<bool> CUSTOM_TEXTURES;
extern const ConfigInfo<bool> PRELOAD_TEXTURES;
extern const ConfigInfo<Settings::LayoutOption> LAYOUT_OPTION;
extern const ConfigInfo<std::string> POST_PROCESSING_SHADER;

// audio
extern const ConfigInfo<bool> ENABLE_DSP_LLE;
extern const ConfigInfo<bool> DSP_LLE_MULTITHREAD;
extern const ConfigInfo<bool> AUDIO_STRETCHING;
extern const ConfigInfo<float> AUDIO_VOLUME;
extern const ConfigInfo<std::string> AUDIO_ENGINE;
extern const ConfigInfo<std::string> AUDIO_DEVICE;

// mic
extern const ConfigInfo<Settings::MicInputType> MIC_INPUT_TYPE;
extern const ConfigInfo<std::string> MIC_INPUT_DEVICE;

// camera
extern const ConfigInfo<std::string> CAMERA_DEVICE;

// debug
extern const ConfigInfo<bool> ALLOW_SHADOW;
extern const ConfigInfo<u8> SHADER_TYPE;
extern const ConfigInfo<bool> USE_PRESENT_THREAD;

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
extern const ConfigInfo<std::string> CIRCLE_PAD_UP;
extern const ConfigInfo<std::string> CIRCLE_PAD_DOWN;
extern const ConfigInfo<std::string> CIRCLE_PAD_LEFT;
extern const ConfigInfo<std::string> CIRCLE_PAD_RIGHT;
extern const ConfigInfo<std::string> C_STICK_UP;
extern const ConfigInfo<std::string> C_STICK_DOWN;
extern const ConfigInfo<std::string> C_STICK_LEFT;
extern const ConfigInfo<std::string> C_STICK_RIGHT;

// custom layout
extern const ConfigInfo<bool> USE_CUSTOM_LAYOUT;
extern const ConfigInfo<u16> CUSTOM_TOP_LEFT;
extern const ConfigInfo<u16> CUSTOM_TOP_TOP;
extern const ConfigInfo<u16> CUSTOM_TOP_RIGHT;
extern const ConfigInfo<u16> CUSTOM_TOP_BOTTOM;
extern const ConfigInfo<u16> CUSTOM_BOTTOM_LEFT;
extern const ConfigInfo<u16> CUSTOM_BOTTOM_TOP;
extern const ConfigInfo<u16> CUSTOM_BOTTOM_RIGHT;
extern const ConfigInfo<u16> CUSTOM_BOTTOM_BOTTOM;

} // namespace Config