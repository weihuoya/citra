// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "camera/camera_base.h"
#include "camera/camera_util.h"

namespace Camera {

CameraBase::CameraBase(const Service::CAM::Flip& flip) {
    using namespace Service::CAM;
    flip_horizontal = basic_flip_horizontal = (flip == Flip::Horizontal) || (flip == Flip::Reverse);
    flip_vertical = basic_flip_vertical = (flip == Flip::Vertical) || (flip == Flip::Reverse);
}

void CameraBase::SetFormat(Service::CAM::OutputFormat output_format) {
    output_rgb = output_format == Service::CAM::OutputFormat::RGB565;
}

void CameraBase::SetResolution(const Service::CAM::Resolution& resolution) {
    width = resolution.width;
    height = resolution.height;
}

void CameraBase::SetFlip(Service::CAM::Flip flip) {
    using namespace Service::CAM;
    flip_horizontal = basic_flip_horizontal ^ (flip == Flip::Horizontal || flip == Flip::Reverse);
    flip_vertical = basic_flip_vertical ^ (flip == Flip::Vertical || flip == Flip::Reverse);
}

void CameraBase::SetEffect(Service::CAM::Effect effect) {
    if (effect != Service::CAM::Effect::None) {
        LOG_ERROR(Service_CAM, "Unimplemented effect {}", static_cast<int>(effect));
    }
}

std::vector<u16> CameraBase::ReceiveFrame() {
    return CameraUtil::ProcessImage(DoReceiveFrame(), width, height, output_rgb, flip_horizontal,
                                    flip_vertical);
}

std::unique_ptr<CameraInterface> BaseCameraFactory::CreatePreview(const std::string& config,
                                                                  int width, int height,
                                                                  const Service::CAM::Flip& flip) {
    std::unique_ptr<CameraInterface> camera = Create(config, flip);

    if (camera->IsPreviewAvailable()) {
        return camera;
    }

    LOG_ERROR(Service_CAM, "Couldn't load the camera: {}", config);
    return nullptr;
}

} // namespace Camera
