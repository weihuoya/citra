#include "oboe_sink.h"

#include <oboe/Oboe.h>

#include "audio_core/audio_types.h"
#include "common/logging/log.h"

namespace AudioCore {

class OboeSink::Impl : public oboe::AudioStreamCallback {
public:
    Impl() = default;
    ~Impl() override = default;

    oboe::DataCallbackResult onAudioReady(oboe::AudioStream *oboeStream, void *audioData, int32_t numFrames) override {
        s16* outputBuffer = static_cast<s16*>(audioData);
        if (mCallback) {
            mCallback(outputBuffer, static_cast<std::size_t>(numFrames));
        }
        return oboe::DataCallbackResult::Continue;
    }

    void onErrorAfterClose(oboe::AudioStream* /* oboeStream */, oboe::Result error) override {
        if (error == oboe::Result::ErrorDisconnected) {
            LOG_INFO(Audio_Sink, "Restarting AudioStream after disconnect");
            start();
        } else {
            LOG_CRITICAL(Audio_Sink, "Error after close: %d", error);
        }
    }

    bool start() {
        oboe::AudioStreamBuilder builder;
        auto result = builder.setSharingMode(oboe::SharingMode::Exclusive)
                ->setPerformanceMode(oboe::PerformanceMode::LowLatency)
                ->setFormat(oboe::AudioFormat::I16)
                ->setSampleRate(mSampleRate)
                ->setChannelCount(2)
                ->setCallback(this)
                ->openManagedStream(mStream);
        if (result != oboe::Result::OK) {
            LOG_CRITICAL(Audio_Sink, "Error creating playback stream: %s", oboe::convertToText(result));
            return false;
        }
        mSampleRate = mStream->getSampleRate();
        result = mStream->start();
        if (result != oboe::Result::OK) {
            LOG_CRITICAL(Audio_Sink, "Error starting playback stream: %s", oboe::convertToText(result));
            return false;
        }
        return true;
    }

    int32_t GetSampleRate() const {
        return mSampleRate;
    }

    void SetCallback(std::function<void(s16*, std::size_t)> cb) {
        mCallback = cb;
    }

private:
    oboe::ManagedStream mStream;
    std::function<void(s16*, std::size_t)> mCallback;
    int32_t mSampleRate = native_sample_rate;
};

OboeSink::OboeSink(std::string_view device_id) : impl(std::make_unique<Impl>()) {}
OboeSink::~OboeSink() = default;

unsigned int OboeSink::GetNativeSampleRate() const {
    return impl->GetSampleRate();
}

void OboeSink::SetCallback(std::function<void(s16*, std::size_t)> cb) {
    impl->SetCallback(cb);
    impl->start();
}

std::vector<std::string> ListOboeSinkDevices() {
    return {"auto"};
}

} // namespace AudioCore
