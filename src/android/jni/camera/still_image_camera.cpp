// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <android/log.h>
#include "jni_common.h"
#include "camera/still_image_camera.h"

StillImageCamera::StillImageCamera(const std::string& config, const Service::CAM::Flip& flip) {
    SetFlip(flip);
}

StillImageCamera::~StillImageCamera() = default;

void StillImageCamera::StartCapture() {
    std::mutex mtx;
    std::condition_variable cv;
    std::unique_lock lck(mtx);
    g_image_loading_callback = [this, &cv](u32* data32, u32 w, u32 h) -> void {
        if (w > 0 && h > 0) {
            width = w;
            height = h;
            pixels.clear();
            pixels.insert(pixels.begin(), data32, data32 + w * h);
        }
        cv.notify_one();
    };
    PickImage(width, height);
    cv.wait(lck);
}

void StillImageCamera::StopCapture() {
    // ignore
}

void StillImageCamera::SetFormat(Service::CAM::OutputFormat output_format) {
    output_rgb = output_format == Service::CAM::OutputFormat::RGB565;
}

void StillImageCamera::SetResolution(const Service::CAM::Resolution& resolution) {
    width = resolution.width;
    height = resolution.height;
}

void StillImageCamera::SetFlip(Service::CAM::Flip flip) {
    using namespace Service::CAM;
    flip_horizontal = basic_flip_horizontal ^ (flip == Flip::Horizontal || flip == Flip::Reverse);
    flip_vertical = basic_flip_vertical ^ (flip == Flip::Vertical || flip == Flip::Reverse);
}

void StillImageCamera::SetEffect(Service::CAM::Effect effect) {
    // ignore
}

void StillImageCamera::SetFrameRate(Service::CAM::FrameRate frame_rate) {
    // ignore
}

std::vector<u16> StillImageCamera::ReceiveFrame() {
    return CameraUtil::ProcessImage(pixels, width, height, output_rgb, flip_horizontal, flip_vertical);
}

bool StillImageCamera::IsPreviewAvailable() {
    return true;
}

std::unique_ptr<Camera::CameraInterface> StillImageCameraFactory::Create(const std::string& config,
                                                                 const Service::CAM::Flip& flip) {
    return std::make_unique<StillImageCamera>(config, flip);
}
