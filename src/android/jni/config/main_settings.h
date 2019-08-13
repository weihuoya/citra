#pragma once

#include <string>

#include "config/config.h"

namespace Settings {
enum class LayoutOption;
} // namespace Settings

namespace Config {

extern const ConfigInfo<bool> USE_CPU_JIT;
extern const ConfigInfo<bool> IS_NEW_3DS;
extern const ConfigInfo<bool> USE_VIRTUAL_SD;
extern const ConfigInfo<int> SYSTEM_REGION;

extern const ConfigInfo<bool> USE_GLES;
extern const ConfigInfo<bool> USE_HW_RENDERER;
extern const ConfigInfo<bool> USE_HW_SHADER;
extern const ConfigInfo<bool> USE_SHADER_JIT;
extern const ConfigInfo<bool> SHADERS_ACCURATE_MUL;
extern const ConfigInfo<bool> SHADERS_ACCURATE_GS;
extern const ConfigInfo<u16> RESOLUTION_FACTOR;
extern const ConfigInfo<bool> USE_FRAME_LIMIT;
extern const ConfigInfo<u16> FRAME_LIMIT;
extern const ConfigInfo<Settings::LayoutOption> LAYOUT_OPTION;

extern const ConfigInfo<bool> ENABLE_DSP_LLE;
extern const ConfigInfo<bool> DSP_LLE_MULTITHREAD;
extern const ConfigInfo<bool> AUDIO_STRETCHING;
extern const ConfigInfo<float> AUDIO_VOLUME;
extern const ConfigInfo<std::string> AUDIO_ENGINE;
extern const ConfigInfo<std::string> AUDIO_DEVICE;

} // namespace Config