#pragma once

#include <memory>
#include <string>
#include <vector>
#include "audio_core/sink.h"

namespace AudioCore {

class OboeSink final : public Sink {
public:
    explicit OboeSink(std::string_view device_id);
    ~OboeSink() override;

    unsigned int GetNativeSampleRate() const override;
    void SetCallback(std::function<void(s16*, std::size_t)> cb) override;

private:
    class Impl;
    std::unique_ptr<Impl> impl;
};

std::vector<std::string> ListOboeSinkDevices();

} // namespace AudioCore
