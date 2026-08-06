#pragma once
#include <atomic>
#include <functional>
#include <libobsensor/ObSensor.hpp>
#include <mutex>

class PipelineHolder {
public:
    using FrameCallback = std::function<void(std::shared_ptr<ob::FrameSet>)>;

    PipelineHolder(std::shared_ptr<ob::Device> device, int deviceIndex);
    ~PipelineHolder();

    void startStream();
    void stopStream();

    void setFrameCallback(FrameCallback cb);

    std::shared_ptr<ob::FrameSet> getLatestFrameSet();

    bool isStreaming() const {
        return streaming_.load();
    }

    std::shared_ptr<ob::Pipeline> getPipeline() const {
        return pipeline_;
    }

    std::string getSerialNumber() const {
        return deviceSN_;
    }

    int getDeviceIndex() const {
        return deviceIndex_;
    }

private:
    void onFrameSet(std::shared_ptr<ob::FrameSet> frameSet);

    std::shared_ptr<ob::Device>   device_;
    std::shared_ptr<ob::Pipeline> pipeline_;
    std::string                   deviceSN_;
    int                           deviceIndex_;

    std::atomic<bool> streaming_{ false };

    std::mutex                    frameMutex_;
    std::shared_ptr<ob::FrameSet> latestFrameSet_;

    FrameCallback userCallback_;
};
