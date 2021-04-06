#include "jni_common.h"
#include "png_handler.h"

PNGHandler::~PNGHandler() = default;

bool PNGHandler::DecodePNG(std::vector<u8>& dst, u32& width, u32& height, const std::string& path) {
    NativeLibrary::LoadImageFromFile(dst, width, height, path);
    return true;
}

bool PNGHandler::EncodePNG(const std::string& path, const std::vector<u8>& src, u32 width,
                           u32 height) {
    NativeLibrary::SaveImageToFile(path, src, width, height);
    return true;
}
