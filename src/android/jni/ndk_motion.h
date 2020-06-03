// Copyright 2020 Citra Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "core/frontend/input.h"

inline std::atomic<int> screen_rotation;

class NDKMotion;

class NDKMotionFactory final : public Input::Factory<Input::MotionDevice> {
public:
    /**
     * Creates a motion device that obtains data from device sensors
     */
    std::unique_ptr<Input::MotionDevice> Create(const Common::ParamPackage& params) override;
};
