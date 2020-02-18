// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <algorithm>
#include <list>
#include <map>

#include "config/config.h"
#include "config/config_loader.h"
#include "config/main_settings.h"

namespace Config {
static Layer s_layer(ConfigLoaders::GenerateConfigLoader());

Layer& GetLayer() {
    return s_layer;
}

void SaveDefault() {
    // core
    s_layer.Set(USE_CPU_JIT, USE_CPU_JIT.default_value);
    s_layer.Set(IS_NEW_3DS, IS_NEW_3DS.default_value);
    s_layer.Set(USE_VIRTUAL_SD, USE_VIRTUAL_SD.default_value);
    s_layer.Set(SYSTEM_REGION, SYSTEM_REGION.default_value);

    // renderer
    s_layer.Set(USE_GLES, USE_GLES.default_value);
    s_layer.Set(USE_HW_RENDERER, USE_HW_RENDERER.default_value);
    s_layer.Set(USE_HW_SHADER, USE_HW_SHADER.default_value);
    s_layer.Set(USE_SHADER_JIT, USE_SHADER_JIT.default_value);
    s_layer.Set(SHADERS_ACCURATE_MUL, SHADERS_ACCURATE_MUL.default_value);
    s_layer.Set(RESOLUTION_FACTOR, RESOLUTION_FACTOR.default_value);
    s_layer.Set(USE_FRAME_LIMIT, USE_FRAME_LIMIT.default_value);
    s_layer.Set(FRAME_LIMIT, FRAME_LIMIT.default_value);
    s_layer.Set(FACTOR_3D, FACTOR_3D.default_value);
    s_layer.Set(LAYOUT_OPTION, LAYOUT_OPTION.default_value);
    s_layer.Set(POST_PROCESSING_SHADER, POST_PROCESSING_SHADER.default_value);

    // audio
    s_layer.Set(ENABLE_DSP_LLE, ENABLE_DSP_LLE.default_value);
    s_layer.Set(DSP_LLE_MULTITHREAD, DSP_LLE_MULTITHREAD.default_value);
    s_layer.Set(AUDIO_STRETCHING, AUDIO_STRETCHING.default_value);
    s_layer.Set(AUDIO_VOLUME, AUDIO_VOLUME.default_value);
    s_layer.Set(AUDIO_ENGINE, AUDIO_ENGINE.default_value);
    s_layer.Set(AUDIO_DEVICE, AUDIO_DEVICE.default_value);

    // mic
    s_layer.Set(MIC_INPUT_TYPE, MIC_INPUT_TYPE.default_value);
    s_layer.Set(MIC_INPUT_DEVICE, MIC_INPUT_DEVICE.default_value);

    // debug
    s_layer.Set(ALLOW_SHADOW, ALLOW_SHADOW.default_value);

    s_layer.Save();
}

} // namespace Config
