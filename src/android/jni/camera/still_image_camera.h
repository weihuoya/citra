// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include "camera/camera_base.h"
#include "camera/camera_util.h"
#include "core/frontend/camera/interface.h"
#include "core/hle/service/cam/cam.h"

namespace Camera {

class StillImageCamera final : public CameraBase {
public:
    StillImageCamera(const std::string& config, const Service::CAM::Flip& flip);
    ~StillImageCamera();
    void StartCapture() override;
    void StopCapture() override;
    std::vector<u32>& DoReceiveFrame() override;
    bool IsPreviewAvailable() override;

private:
    std::vector<u32> pixels;
};

class StillImageCameraFactory final : public BaseCameraFactory {
public:
    std::unique_ptr<CameraInterface> Create(const std::string& config,
                                            const Service::CAM::Flip& flip) override;

private:
    friend class StillImageCamera;
};

} // namespace Camera
