// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <string>
#include "camera/camera_util.h"
#include "core/frontend/camera/factory.h"
#include "core/hle/service/cam/cam.h"

class StillImageCamera : public Camera::CameraInterface {
public:
    StillImageCamera(const std::string& config, const Service::CAM::Flip& flip);
    ~StillImageCamera();
    void StartCapture() override;
    void StopCapture() override;
    void SetResolution(const Service::CAM::Resolution&) override;
    void SetFlip(Service::CAM::Flip) override;
    void SetEffect(Service::CAM::Effect) override;
    void SetFormat(Service::CAM::OutputFormat) override;
    void SetFrameRate(Service::CAM::FrameRate frame_rate) override;
    std::vector<u16> ReceiveFrame() override;
    bool IsPreviewAvailable() override;

private:
    int width = 0;
    int height = 0;
    bool output_rgb = false;
    bool flip_horizontal, flip_vertical;
    bool basic_flip_horizontal, basic_flip_vertical;
    std::vector<u32> pixels;
};

class StillImageCameraFactory : public Camera::CameraFactory {
public:
    std::unique_ptr<Camera::CameraInterface> Create(const std::string& config,
                                                    const Service::CAM::Flip& flip) override;

private:
    friend class StillImageCamera;
};
