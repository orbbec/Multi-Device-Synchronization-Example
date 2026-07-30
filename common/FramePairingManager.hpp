#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct DeviceTimestamp {
    int64_t hwFrameNum = -1;
    int64_t swFrameNum = -1;
    int64_t systemTs   = 0;
    int64_t deviceTs   = 0;
    int64_t globalTs   = 0;
};

class FramePairingManager {
public:
    FramePairingManager();
    ~FramePairingManager();

    void init(const std::vector<std::string> &deviceSNs, const std::string &outputDir = ".");

    void pushColorFrame(int deviceIndex, int64_t hwFrameNum, int64_t swFrameNum, int64_t systemTs, int64_t deviceTs, int64_t globalTs);
    void pushDepthFrame(int deviceIndex, int64_t hwFrameNum, int64_t swFrameNum, int64_t systemTs, int64_t deviceTs, int64_t globalTs);

    bool tryFlushColorRow();
    bool tryFlushDepthRow();

    void     setRecording(bool v);
    void     resetCounters();
    uint64_t getColorCapturedCount(size_t deviceIndex) const;
    uint64_t getDepthCapturedCount(size_t deviceIndex) const;
    uint64_t getColorRowsWritten() const;
    uint64_t getDepthRowsWritten() const;

    struct SyncAccuracy {
        double   avgGlobalRangeUs    = 0.0;
        double   maxGlobalRangeUs    = 0.0;
        double   stddevGlobalRangeUs = 0.0;
        double   avgDeviceRangeUs    = 0.0;
        double   maxDeviceRangeUs    = 0.0;
        double   stddevDeviceRangeUs = 0.0;
        uint64_t sampleCount         = 0;
    };
    SyncAccuracy getColorAccuracy() const;
    SyncAccuracy getDepthAccuracy() const;

    void startBackgroundFlush();
    void stopBackgroundFlush();

    void release();

private:
    bool     tryFlushRow(bool isColor);
    void     flushThreadFunc();
    uint64_t flushBatchToCsv(bool isColor, uint64_t maxRows = 500);
    void     writeCsvHeader(std::ostream &csv);
    void     writeCsvRow(std::ostream &csv, const std::vector<DeviceTimestamp> &snapshot, const std::vector<std::string> &snSnapshot);

    size_t                   deviceCount_ = 0;
    std::vector<std::string> deviceSNs_;

    mutable std::mutex                       colorMutex_;
    std::vector<std::deque<DeviceTimestamp>> colorQueues_;
    std::vector<uint64_t>                    colorCounters_;
    std::atomic<uint64_t>                    colorRowsWritten_{ 0 };

    std::mutex    colorCsvMutex_;
    std::ofstream colorCsv_;
    uint64_t      colorRowId_ = 0;

    mutable std::mutex                       depthMutex_;
    std::vector<std::deque<DeviceTimestamp>> depthQueues_;
    std::vector<uint64_t>                    depthCounters_;
    std::atomic<uint64_t>                    depthRowsWritten_{ 0 };

    std::mutex    depthCsvMutex_;
    std::ofstream depthCsv_;
    uint64_t      depthRowId_ = 0;

    std::atomic<bool> recording_{ false };
    std::atomic<bool> destroy_{ false };

    std::thread             flushThread_;
    std::mutex              flushCvMtx_;
    std::condition_variable flushCv_;
    std::atomic<bool>       flushThreadRunning_{ false };
    std::atomic<bool>       flushThreadStop_{ false };

    mutable std::mutex      accuracyMtx_;
    std::vector<int64_t>    colorGlobalRangeSamples_;
    std::vector<int64_t>    colorDeviceRangeSamples_;
    std::vector<int64_t>    depthGlobalRangeSamples_;
    std::vector<int64_t>    depthDeviceRangeSamples_;
    static constexpr size_t MAX_ACCURACY_SAMPLES = 3000;
};

extern FramePairingManager gTimestampBuffer;
