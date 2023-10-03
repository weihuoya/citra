// Copyright 2019 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <array>
#include "core/frontend/mic.h"
#include "core/settings.h"

namespace Frontend::Mic {

constexpr std::array<u8, 16> NOISE_SAMPLE_8_BIT = {0xFC, 0xFF, 0x7F, 0x8F, 0x80, 0xFF, 0xFF, 0xFF,
                                                   0xFF, 0xF5, 0x7F, 0x7F, 0x7F, 0xFF, 0x8E, 0xFF};

constexpr std::array<u8, 32> NOISE_SAMPLE_16_BIT = {
    0xFF, 0x7F, 0xFF, 0x7F, 0x0, 0x80, 0x0, 0x0,  0x20, 0xDD, 0x18, 0x16, 0xDC, 0x69, 0xD0, 0x7F,
    0xA0, 0x3C, 0xAC, 0xAD, 0x0, 0x80, 0x0, 0x80, 0x0,  0x80, 0x0,  0x80, 0x0,  0x80, 0x0,  0x80};

Interface::~Interface() = default;

void NullMic::StartSampling(const Parameters& params) {
    parameters = params;
    is_sampling = true;
}

void NullMic::StopSampling() {
    is_sampling = false;
}

void NullMic::AdjustSampleRate(u32 sample_rate) {
    parameters.sample_rate = sample_rate;
}

Samples NullMic::Read() {
    return {};
}

StaticMic::StaticMic()
    : CACHE_8_BIT{NOISE_SAMPLE_8_BIT.begin(), NOISE_SAMPLE_8_BIT.end()},
      CACHE_16_BIT{NOISE_SAMPLE_16_BIT.begin(), NOISE_SAMPLE_16_BIT.end()} {}

StaticMic::~StaticMic() = default;

void StaticMic::StartSampling(const Parameters& params) {
    sample_rate = params.sample_rate;
    sample_size = params.sample_size;

    parameters = params;
    is_sampling = true;
}

void StaticMic::StopSampling() {
    is_sampling = false;
}

void StaticMic::AdjustSampleRate(u32 sample_rate) {}

Samples StaticMic::Read() {
    return (sample_size == 8) ? CACHE_8_BIT : CACHE_16_BIT;
}

std::unique_ptr<Interface> NullFactory::Create([[maybe_unused]] std::string mic_device_name) {
    return std::make_unique<NullMic>();
}

static std::unique_ptr<RealMicFactory> g_factory = std::make_unique<NullFactory>();

void RegisterRealMicFactory(std::unique_ptr<RealMicFactory> factory) {
    g_factory = std::move(factory);
}

std::unique_ptr<Interface> CreateRealMic() {
    if (Settings::values.mic_input_type == Settings::MicInputType::Real) {
        return g_factory->Create(Settings::values.mic_input_device);
    } else if (Settings::values.mic_input_type == Settings::MicInputType::Static) {
        return std::make_unique<Frontend::Mic::StaticMic>();
    } else {
        return std::make_unique<Frontend::Mic::NullMic>();
    }
}

} // namespace Frontend::Mic
