#pragma once

#include <string>

#include "common/file_util.h"

class AndroidIOFactory : public FileUtil::IOFactory {
public:
    ~AndroidIOFactory() override = default;
    std::unique_ptr<FileUtil::IOHandler> Open(const std::string& filename, const char openmode[]) override;
};

bool IsSafPath(const std::string& path);