#include "FramePairingManager.hpp"
#include <cmath>
#include <iostream>
#include <sstream>

static constexpr size_t MAX_QUEUE_SIZE = 600;

FramePairingManager::FramePairingManager() {}

FramePairingManager::~FramePairingManager() {
    release();
}

void FramePairingManager::init(const std::vector<std::string> &deviceSNs, const std::string &outputDir) {
    std::lock_guard<std::mutex> colorLk(colorMutex_);
    std::lock_guard<std::mutex> depthLk(depthMutex_);
    deviceCount_ = deviceSNs.size();
    deviceSNs_   = deviceSNs;
    colorQueues_.assign(deviceCount_, std::deque<DeviceTimestamp>());
    colorCounters_.assign(deviceCount_, 0);
    depthQueues_.assign(deviceCount_, std::deque<DeviceTimestamp>());
    depthCounters_.assign(deviceCount_, 0);
    colorRowId_ = 0;
    depthRowId_ = 0;

    // Reserve accuracy sample vectors
    {
        std::lock_guard<std::mutex> lk(accuracyMtx_);
        colorGlobalRangeSamples_.clear();
        colorDeviceRangeSamples_.clear();
        depthGlobalRangeSamples_.clear();
        depthDeviceRangeSamples_.clear();
        colorGlobalRangeSamples_.reserve(MAX_ACCURACY_SAMPLES);
        colorDeviceRangeSamples_.reserve(MAX_ACCURACY_SAMPLES);
        depthGlobalRangeSamples_.reserve(MAX_ACCURACY_SAMPLES);
        depthDeviceRangeSamples_.reserve(MAX_ACCURACY_SAMPLES);
    }

    // Open color CSV
    {
        std::lock_guard<std::mutex> csvLk(colorCsvMutex_);
        std::string                 path = outputDir + "/sync_color_timestamps.csv";
        colorCsv_.open(path);
        if(colorCsv_.is_open()) {
            // Set larger stream buffer: 64KB
            constexpr size_t BUF_SIZE = 65536;
            static char      colorBuf[BUF_SIZE];
            colorCsv_.rdbuf()->pubsetbuf(colorBuf, BUF_SIZE);
            colorCsv_ << "D0_SN,D0_sw_frame_num,D0_hw_frame_num,D0_system_ts_us,D0_device_ts_us,D0_global_ts_us";
            for(size_t i = 1; i < deviceCount_; ++i) {
                colorCsv_ << ",D" << i << "_SN"
                          << ",D" << i << "_sw_frame_num"
                          << ",D" << i << "_hw_frame_num"
                          << ",D" << i << "_system_ts_us"
                          << ",D" << i << "_device_ts_us"
                          << ",D" << i << "_global_ts_us";
            }
            colorCsv_ << "\n";
            std::cout << "Color CSV: " << path << std::endl;
        }
    }

    // Open depth CSV
    {
        std::lock_guard<std::mutex> csvLk(depthCsvMutex_);
        std::string                 path = outputDir + "/sync_depth_timestamps.csv";
        depthCsv_.open(path);
        if(depthCsv_.is_open()) {
            constexpr size_t BUF_SIZE = 65536;
            static char      depthBuf[BUF_SIZE];
            depthCsv_.rdbuf()->pubsetbuf(depthBuf, BUF_SIZE);
            depthCsv_ << "D0_SN,D0_sw_frame_num,D0_hw_frame_num,D0_system_ts_us,D0_device_ts_us,D0_global_ts_us";
            for(size_t i = 1; i < deviceCount_; ++i) {
                depthCsv_ << ",D" << i << "_SN"
                          << ",D" << i << "_sw_frame_num"
                          << ",D" << i << "_hw_frame_num"
                          << ",D" << i << "_system_ts_us"
                          << ",D" << i << "_device_ts_us"
                          << ",D" << i << "_global_ts_us";
            }
            depthCsv_ << "\n";
            std::cout << "Depth CSV: " << path << std::endl;
        }
    }
}

void FramePairingManager::pushColorFrame(int deviceIndex, int64_t hwFrameNum, int64_t swFrameNum, int64_t systemTs, int64_t deviceTs, int64_t globalTs) {
    if(!recording_.load()) {
        return;
    }
    std::lock_guard<std::mutex> lk(colorMutex_);
    if(static_cast<size_t>(deviceIndex) >= colorQueues_.size()) {
        return;
    }

    auto &q = colorQueues_[deviceIndex];
    if(q.size() >= MAX_QUEUE_SIZE) {
        q.pop_front();
    }

    q.emplace_back();
    auto &b      = q.back();
    b.hwFrameNum = hwFrameNum;
    b.swFrameNum = swFrameNum;
    b.systemTs   = systemTs;
    b.deviceTs   = deviceTs;
    b.globalTs   = globalTs;
    ++colorCounters_[deviceIndex];
}

void FramePairingManager::pushDepthFrame(int deviceIndex, int64_t hwFrameNum, int64_t swFrameNum, int64_t systemTs, int64_t deviceTs, int64_t globalTs) {
    if(!recording_.load()) {
        return;
    }
    std::lock_guard<std::mutex> lk(depthMutex_);
    if(static_cast<size_t>(deviceIndex) >= depthQueues_.size()) {
        return;
    }

    auto &q = depthQueues_[deviceIndex];
    if(q.size() >= MAX_QUEUE_SIZE) {
        q.pop_front();
    }

    q.emplace_back();
    auto &b      = q.back();
    b.hwFrameNum = hwFrameNum;
    b.swFrameNum = swFrameNum;
    b.systemTs   = systemTs;
    b.deviceTs   = deviceTs;
    b.globalTs   = globalTs;
    ++depthCounters_[deviceIndex];
}

bool FramePairingManager::tryFlushColorRow() {
    return tryFlushRow(true);
}

bool FramePairingManager::tryFlushDepthRow() {
    return tryFlushRow(false);
}

bool FramePairingManager::tryFlushRow(bool isColor) {
    auto &mutex       = isColor ? colorMutex_ : depthMutex_;
    auto &queues      = isColor ? colorQueues_ : depthQueues_;
    auto &csvMutex    = isColor ? colorCsvMutex_ : depthCsvMutex_;
    auto &csv         = isColor ? colorCsv_ : depthCsv_;
    auto &rowId       = isColor ? colorRowId_ : depthRowId_;
    auto &rowsWritten = isColor ? colorRowsWritten_ : depthRowsWritten_;

    std::vector<DeviceTimestamp> snapshot;
    std::vector<std::string>     snSnapshot;
    uint64_t                     seq;

    {
        std::lock_guard<std::mutex> lk(mutex);
        for(size_t d = 0; d < deviceCount_; ++d) {
            if(queues[d].empty()) {
                return false;
            }
        }
        seq        = rowId++;
        snSnapshot = deviceSNs_;
        snapshot.reserve(deviceCount_);
        for(size_t d = 0; d < deviceCount_; ++d) {
            snapshot.push_back(queues[d].front());
            queues[d].pop_front();
        }
    }

    int64_t minGlobal = snapshot[0].globalTs;
    int64_t maxGlobal = snapshot[0].globalTs;
    int64_t minDevice = snapshot[0].deviceTs;
    int64_t maxDevice = snapshot[0].deviceTs;
    for(size_t d = 1; d < deviceCount_; ++d) {
        if(snapshot[d].globalTs < minGlobal) {
            minGlobal = snapshot[d].globalTs;
        }
        if(snapshot[d].globalTs > maxGlobal) {
            maxGlobal = snapshot[d].globalTs;
        }
        if(snapshot[d].deviceTs < minDevice) {
            minDevice = snapshot[d].deviceTs;
        }
        if(snapshot[d].deviceTs > maxDevice) {
            maxDevice = snapshot[d].deviceTs;
        }
    }

    int64_t globalRange = maxGlobal - minGlobal;
    int64_t deviceRange = maxDevice - minDevice;

    {
        std::lock_guard<std::mutex> csvLk(csvMutex);
        if(!csv.is_open()) {
            return false;
        }
        csv << snSnapshot[0] << "," << snapshot[0].swFrameNum << "," << snapshot[0].hwFrameNum << "," << snapshot[0].systemTs << ","
            << snapshot[0].deviceTs << "," << snapshot[0].globalTs;
        for(size_t d = 1; d < deviceCount_; ++d) {
            csv << "," << snSnapshot[d] << "," << snapshot[d].swFrameNum << "," << snapshot[d].hwFrameNum << "," << snapshot[d].systemTs << ","
                << snapshot[d].deviceTs << "," << snapshot[d].globalTs;
        }
        csv << "\n";
    }

    // Record accuracy samples
    {
        auto                       &gSamples = isColor ? colorGlobalRangeSamples_ : depthGlobalRangeSamples_;
        auto                       &dSamples = isColor ? colorDeviceRangeSamples_ : depthDeviceRangeSamples_;
        std::lock_guard<std::mutex> lk(accuracyMtx_);
        if(gSamples.size() < MAX_ACCURACY_SAMPLES) {
            gSamples.push_back(globalRange);
            dSamples.push_back(deviceRange);
        }
    }

    uint64_t written = ++rowsWritten;
    return true;
}

// Batch flush: drain all currently-paired rows in one go, write via
// ostringstream
uint64_t FramePairingManager::flushBatchToCsv(bool isColor, uint64_t maxRows) {
    if(deviceCount_ == 0) {
        return 0;
    }
    auto &mutex       = isColor ? colorMutex_ : depthMutex_;
    auto &queues      = isColor ? colorQueues_ : depthQueues_;
    auto &csvMutex    = isColor ? colorCsvMutex_ : depthCsvMutex_;
    auto &csv         = isColor ? colorCsv_ : depthCsv_;
    auto &rowId       = isColor ? colorRowId_ : depthRowId_;
    auto &rowsWritten = isColor ? colorRowsWritten_ : depthRowsWritten_;
    auto &gSamples    = isColor ? colorGlobalRangeSamples_ : depthGlobalRangeSamples_;
    auto &dSamples    = isColor ? colorDeviceRangeSamples_ : depthDeviceRangeSamples_;

    std::ostringstream batchSs;
    uint64_t           flushed = 0;

    while(flushed < maxRows) {
        std::vector<DeviceTimestamp> snapshot;
        std::vector<std::string>     snSnapshot;
        uint64_t                     seq;

        {
            std::lock_guard<std::mutex> lk(mutex);
            bool                        anyEmpty = false;
            for(size_t d = 0; d < deviceCount_; ++d) {
                if(queues[d].empty()) {
                    anyEmpty = true;
                    break;
                }
            }
            if(anyEmpty) {
                break;
            }

            seq        = rowId++;
            snSnapshot = deviceSNs_;
            snapshot.reserve(deviceCount_);
            for(size_t d = 0; d < deviceCount_; ++d) {
                snapshot.push_back(queues[d].front());
                queues[d].pop_front();
            }
        }

        int64_t minGlobal = snapshot[0].globalTs;
        int64_t maxGlobal = snapshot[0].globalTs;
        int64_t minDevice = snapshot[0].deviceTs;
        int64_t maxDevice = snapshot[0].deviceTs;
        for(size_t d = 1; d < deviceCount_; ++d) {
            if(snapshot[d].globalTs < minGlobal) {
                minGlobal = snapshot[d].globalTs;
            }
            if(snapshot[d].globalTs > maxGlobal) {
                maxGlobal = snapshot[d].globalTs;
            }
            if(snapshot[d].deviceTs < minDevice) {
                minDevice = snapshot[d].deviceTs;
            }
            if(snapshot[d].deviceTs > maxDevice) {
                maxDevice = snapshot[d].deviceTs;
            }
        }

        int64_t globalRange = maxGlobal - minGlobal;
        int64_t deviceRange = maxDevice - minDevice;

        batchSs << snSnapshot[0] << "," << snapshot[0].swFrameNum << "," << snapshot[0].hwFrameNum << "," << snapshot[0].systemTs << ","
                << snapshot[0].deviceTs << "," << snapshot[0].globalTs;
        for(size_t d = 1; d < deviceCount_; ++d) {
            batchSs << "," << snSnapshot[d] << "," << snapshot[d].swFrameNum << "," << snapshot[d].hwFrameNum << "," << snapshot[d].systemTs << ","
                    << snapshot[d].deviceTs << "," << snapshot[d].globalTs;
        }
        batchSs << "\n";

        // Record accuracy samples
        {
            std::lock_guard<std::mutex> lk(accuracyMtx_);
            if(gSamples.size() < MAX_ACCURACY_SAMPLES) {
                gSamples.push_back(globalRange);
                dSamples.push_back(deviceRange);
            }
        }

        ++rowsWritten;
        ++flushed;
    }

    if(flushed > 0) {
        std::lock_guard<std::mutex> csvLk(csvMutex);
        if(csv.is_open()) {
            csv << batchSs.str();
        }
    }

    return flushed;
}

void FramePairingManager::flushThreadFunc() {
    std::unique_lock<std::mutex> lk(flushCvMtx_);
    while(!flushThreadStop_.load()) {
        // Wait up to 100ms or until notified
        flushCv_.wait_for(lk, std::chrono::milliseconds(100));

        if(!recording_.load()) {
            continue;
        }

        flushBatchToCsv(true, 500);
        flushBatchToCsv(false, 500);
    }
}

void FramePairingManager::startBackgroundFlush() {
    if(flushThreadRunning_.exchange(true)) {
        return;
    }
    flushThreadStop_.store(false);
    flushThread_ = std::thread(&FramePairingManager::flushThreadFunc, this);
}

void FramePairingManager::stopBackgroundFlush() {
    if(!flushThreadRunning_.exchange(false)) {
        return;
    }
    flushThreadStop_.store(true);
    flushCv_.notify_all();
    if(flushThread_.joinable()) {
        flushThread_.join();
    }
    // Flush any remaining rows
    uint64_t remaining = 0;
    do {
        remaining = flushBatchToCsv(true, 500);
    } while(remaining > 0);
    do {
        remaining = flushBatchToCsv(false, 500);
    } while(remaining > 0);
}

void FramePairingManager::setRecording(bool v) {
    recording_.store(v);
}

void FramePairingManager::resetCounters() {
    std::lock_guard<std::mutex> colorLk(colorMutex_);
    std::lock_guard<std::mutex> depthLk(depthMutex_);
    for(auto &c: colorCounters_)
        c = 0;
    for(auto &c: depthCounters_)
        c = 0;
    colorRowsWritten_.store(0);
    depthRowsWritten_.store(0);
}

uint64_t FramePairingManager::getColorCapturedCount(size_t deviceIndex) const {
    std::lock_guard<std::mutex> lk(colorMutex_);
    if(deviceIndex >= colorCounters_.size()) {
        return 0;
    }
    return colorCounters_[deviceIndex];
}

uint64_t FramePairingManager::getDepthCapturedCount(size_t deviceIndex) const {
    std::lock_guard<std::mutex> lk(depthMutex_);
    if(deviceIndex >= depthCounters_.size()) {
        return 0;
    }
    return depthCounters_[deviceIndex];
}

uint64_t FramePairingManager::getColorRowsWritten() const {
    return colorRowsWritten_.load();
}

uint64_t FramePairingManager::getDepthRowsWritten() const {
    return depthRowsWritten_.load();
}

FramePairingManager::SyncAccuracy FramePairingManager::getColorAccuracy() const {
    std::lock_guard<std::mutex> lk(accuracyMtx_);
    SyncAccuracy                acc;
    acc.sampleCount = colorGlobalRangeSamples_.size();
    if(acc.sampleCount == 0) {
        return acc;
    }

    double sumG = 0, sumD = 0;
    double maxG = 0, maxD = 0;
    for(size_t i = 0; i < acc.sampleCount; ++i) {
        double g = static_cast<double>(colorGlobalRangeSamples_[i]);
        double d = static_cast<double>(colorDeviceRangeSamples_[i]);
        sumG += g;
        sumD += d;
        if(g > maxG) {
            maxG = g;
        }
        if(d > maxD) {
            maxD = d;
        }
    }
    acc.avgGlobalRangeUs = sumG / acc.sampleCount;
    acc.avgDeviceRangeUs = sumD / acc.sampleCount;
    acc.maxGlobalRangeUs = maxG;
    acc.maxDeviceRangeUs = maxD;

    double varG = 0, varD = 0;
    for(size_t i = 0; i < acc.sampleCount; ++i) {
        double dg = static_cast<double>(colorGlobalRangeSamples_[i]) - acc.avgGlobalRangeUs;
        double dd = static_cast<double>(colorDeviceRangeSamples_[i]) - acc.avgDeviceRangeUs;
        varG += dg * dg;
        varD += dd * dd;
    }
    acc.stddevGlobalRangeUs = std::sqrt(varG / acc.sampleCount);
    acc.stddevDeviceRangeUs = std::sqrt(varD / acc.sampleCount);
    return acc;
}

FramePairingManager::SyncAccuracy FramePairingManager::getDepthAccuracy() const {
    std::lock_guard<std::mutex> lk(accuracyMtx_);
    SyncAccuracy                acc;
    acc.sampleCount = depthGlobalRangeSamples_.size();
    if(acc.sampleCount == 0) {
        return acc;
    }

    double sumG = 0, sumD = 0;
    double maxG = 0, maxD = 0;
    for(size_t i = 0; i < acc.sampleCount; ++i) {
        double g = static_cast<double>(depthGlobalRangeSamples_[i]);
        double d = static_cast<double>(depthDeviceRangeSamples_[i]);
        sumG += g;
        sumD += d;
        if(g > maxG) {
            maxG = g;
        }
        if(d > maxD) {
            maxD = d;
        }
    }
    acc.avgGlobalRangeUs = sumG / acc.sampleCount;
    acc.avgDeviceRangeUs = sumD / acc.sampleCount;
    acc.maxGlobalRangeUs = maxG;
    acc.maxDeviceRangeUs = maxD;

    double varG = 0, varD = 0;
    for(size_t i = 0; i < acc.sampleCount; ++i) {
        double dg = static_cast<double>(depthGlobalRangeSamples_[i]) - acc.avgGlobalRangeUs;
        double dd = static_cast<double>(depthDeviceRangeSamples_[i]) - acc.avgDeviceRangeUs;
        varG += dg * dg;
        varD += dd * dd;
    }
    acc.stddevGlobalRangeUs = std::sqrt(varG / acc.sampleCount);
    acc.stddevDeviceRangeUs = std::sqrt(varD / acc.sampleCount);
    return acc;
}

void FramePairingManager::release() {
    stopBackgroundFlush();
    if(destroy_.exchange(true)) {
        return;
    }
    {
        std::lock_guard<std::mutex> lk(colorCsvMutex_);
        if(colorCsv_.is_open()) {
            colorCsv_.flush();
            colorCsv_.close();
        }
    }
    {
        std::lock_guard<std::mutex> lk(depthCsvMutex_);
        if(depthCsv_.is_open()) {
            depthCsv_.flush();
            depthCsv_.close();
        }
    }
}

FramePairingManager gTimestampBuffer;
