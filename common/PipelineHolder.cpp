#include "PipelineHolder.hpp"
#include <iostream>

PipelineHolder::PipelineHolder(std::shared_ptr<ob::Device> device, int deviceIndex) : device_(device), deviceIndex_(deviceIndex) {
    if(device_) {
        pipeline_ = std::make_shared<ob::Pipeline>(device_);
        deviceSN_ = device_->getDeviceInfo()->serialNumber();
    }
}

PipelineHolder::~PipelineHolder() {
    stopStream();
}

void PipelineHolder::setFrameCallback(FrameCallback cb) {
    userCallback_ = cb;
}

void PipelineHolder::startStream() {
    if(streaming_.load() || !pipeline_) {
        return;
    }

    try {
        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
        config->enableStream(OB_SENSOR_DEPTH);
        config->enableStream(OB_SENSOR_COLOR);

        pipeline_->start(config, [this](std::shared_ptr<ob::FrameSet> frameSet) { onFrameSet(frameSet); });

        streaming_ = true;
        std::cout << "startStream: " << deviceSN_ << " (device #" << deviceIndex_ << ")" << std::endl;
    }
    catch(ob::Error &e) {
        std::cerr << "starting stream failed: " << deviceSN_ << std::endl;
        std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\nstatus:" << e.getStatus()
                  << "\ntype:" << e.getExceptionType() << std::endl;
    }
}

void PipelineHolder::onFrameSet(std::shared_ptr<ob::FrameSet> frameSet) {
    if(!frameSet) {
        return;
    }
    {
        std::lock_guard<std::mutex> lk(frameMutex_);
        latestFrameSet_ = frameSet;
    }
    if(userCallback_) {
        userCallback_(frameSet);
    }
}

std::shared_ptr<ob::FrameSet> PipelineHolder::getLatestFrameSet() {
    std::lock_guard<std::mutex> lk(frameMutex_);
    return latestFrameSet_;
}

void PipelineHolder::stopStream() {
    if(!streaming_.exchange(false) || !pipeline_) {
        return;
    }

    try {
        std::cout << "stopStream: " << deviceSN_ << " (device #" << deviceIndex_ << ")" << std::endl;
        pipeline_->stop();
    }
    catch(ob::Error &e) {
        std::cerr << "stopping stream failed: " << deviceSN_ << std::endl;
        std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\nstatus:" << e.getStatus()
                  << "\ntype:" << e.getExceptionType() << std::endl;
    }

    std::lock_guard<std::mutex> lk(frameMutex_);
    latestFrameSet_.reset();
}
