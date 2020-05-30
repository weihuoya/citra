#include "jni_common.h"
#include "png_handler.h"

PNGHandler::~PNGHandler() = default;

bool PNGHandler::DecodePNG(std::vector<u8>& dst, u32& width, u32& height, const std::string& path) {
    LoadImageFromFile(path, dst, width, height);
    return true;
}

bool PNGHandler::EncodePNG(const std::string& path, const std::vector<u8>& src, u32 width,
                           u32 height) {
    SaveImageToFile(path, src, width, height);
    return true;
}
