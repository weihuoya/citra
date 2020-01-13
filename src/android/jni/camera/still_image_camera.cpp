// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "camera/still_image_camera.h"

namespace Camera {

StillImageCamera::StillImageCamera(const std::string& config, const Service::CAM::Flip& flip)
    : CameraBase(flip) {}

StillImageCamera::~StillImageCamera() = default;

void StillImageCamera::StartCapture() {}

void StillImageCamera::StopCapture() {}

std::vector<u32>& StillImageCamera::DoReceiveFrame() {
    return mPixels;
}

bool StillImageCamera::IsPreviewAvailable() {
    return !mPixels.empty();
}

std::unique_ptr<CameraInterface> StillImageCameraFactory::Create(const std::string& config,
                                                                 const Service::CAM::Flip& flip) {
    return std::make_unique<StillImageCamera>(config, flip);
}

} // namespace Camera
