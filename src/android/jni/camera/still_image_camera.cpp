// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <android/log.h>
#include "jni_common.h"
#include "camera/still_image_camera.h"

namespace Camera {

StillImageCamera::StillImageCamera(const std::string& config, const Service::CAM::Flip& flip)
    : CameraBase(flip) {}

StillImageCamera::~StillImageCamera() = default;

void StillImageCamera::StartCapture() {
    std::mutex mtx;
    std::condition_variable cv;
    std::unique_lock lck(mtx);
    g_image_loading_callback = [this, &cv](u32* data32, u32 w, u32 h) -> void {
        width = w;
        height = h;
        pixels.clear();
        pixels.insert(pixels.begin(), data32, data32 + w * h);
        cv.notify_one();
    };
    PickImage(width, height);
    cv.wait(lck);
}

void StillImageCamera::StopCapture() {
}

std::vector<u32>& StillImageCamera::DoReceiveFrame() {
    return pixels;
}

bool StillImageCamera::IsPreviewAvailable() {
    return !pixels.empty();
}

std::unique_ptr<CameraInterface> StillImageCameraFactory::Create(const std::string& config,
                                                                 const Service::CAM::Flip& flip) {
    return std::make_unique<StillImageCamera>(config, flip);
}

} // namespace Camera
