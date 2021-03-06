// Copyright 2020 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <jni.h>
#include "audio_core/cubeb_input.h"
#include "jni_common.h"
#include "mic.h"

AndroidMicFactory::~AndroidMicFactory() = default;

std::unique_ptr<Frontend::Mic::Interface> AndroidMicFactory::Create(std::string mic_device_name) {
    if (NativeLibrary::CheckRecordPermission()) {
        return std::make_unique<AudioCore::CubebInput>(std::move(mic_device_name));
    } else {
        return std::make_unique<Frontend::Mic::NullMic>();
    }
}
