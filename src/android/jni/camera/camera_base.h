// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <string>
#include "core/frontend/camera/factory.h"

namespace Camera {

// Base class for camera interfaces of citra
class CameraBase : public CameraInterface {
public:
    CameraBase(const Service::CAM::Flip& flip);
    void SetResolution(const Service::CAM::Resolution&) override;
    void SetFlip(Service::CAM::Flip) override;
    void SetEffect(Service::CAM::Effect) override;
    void SetFormat(Service::CAM::OutputFormat) override;
    std::vector<u16> ReceiveFrame() override;
    virtual std::vector<u32>& DoReceiveFrame() = 0;

private:
    int width = 0;
    int height = 0;
    bool output_rgb = false;
    bool flip_horizontal, flip_vertical;
    bool basic_flip_horizontal, basic_flip_vertical;
};

// Base class for camera factories of citra_qt
class BaseCameraFactory : public CameraFactory {
    std::unique_ptr<CameraInterface> CreatePreview(const std::string& config, int width, int height,
                                                   const Service::CAM::Flip& flip) override;
};

} // namespace Camera
