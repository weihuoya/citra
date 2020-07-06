#pragma once

#include <string>
#include <vector>

#include "core/frontend/image_interface.h"

class PNGHandler : public Frontend::ImageInterface {
public:
    ~PNGHandler() override;

    // Error logging should be handled by the frontend
    bool DecodePNG(std::vector<u8>& dst, u32& width, u32& height, const std::string& path) override;
    bool EncodePNG(const std::string& path, const std::vector<u8>& src, u32 width,
                   u32 height) override;
};
