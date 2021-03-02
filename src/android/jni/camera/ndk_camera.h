// Copyright 2020 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <memory>

#include <camera/NdkCameraManager.h>

#include "core/frontend/camera/factory.h"
#include "core/frontend/camera/interface.h"

class NDKCameraInterface : public Camera::CameraInterface {
public:
    NDKCameraInterface(const std::string &config, const Service::CAM::Flip& flip);
    ~NDKCameraInterface() override;

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
    class Impl;
    std::shared_ptr<Impl> impl;
};

class NDKCameraFactory : public Camera::CameraFactory {
public:
    std::unique_ptr<Camera::CameraInterface> Create(const std::string& config,
                                            const Service::CAM::Flip& flip) override;
};
