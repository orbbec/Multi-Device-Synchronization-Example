
#include <algorithm>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cJSON.h"
#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"
#include "window.hpp"

// extern "C" {
// #include <libavcodec/avcodec.h>
// #include <libavutil/pixdesc.h>
// #include <libavutil/imgutils.h>
// #include <libswscale/swscale.h>
// #include <libavformat/avformat.h>
// #include <libavutil/opt.h>
// }

#include <chrono>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#else
#include <strings.h>
#endif

#define MAX_DEVICE_COUNT 4
#define CONFIG_FILE "./config/MultiDeviceSyncConfig.json"

#define MAX_INTERVAL_TIME 66

typedef struct DeviceConfigInfo_t {
  std::string deviceSN;
  OBMultiDeviceSyncConfig syncConfig;
} DeviceConfigInfo;

typedef struct PipelineHolderr_t {
  std::shared_ptr<ob::Pipeline> pipeline;
  OBSensorType sensorType;
  int deviceIndex;
  std::string deviceSN;
} PipelineHolder;
std::ostream &operator<<(std::ostream &os, const PipelineHolder &holder);
std::ostream &operator<<(std::ostream &os,
                         std::shared_ptr<PipelineHolder> holder) {
  return os << *holder;
}

// std::mutex frameMutex;
std::mutex frameMutex[MAX_DEVICE_COUNT];
std::mutex depthFrameMutex[MAX_DEVICE_COUNT];
std::mutex rgbFrameMutex[MAX_DEVICE_COUNT];
std::condition_variable colorCondition[MAX_DEVICE_COUNT];
std::condition_variable depthCondition[MAX_DEVICE_COUNT];
std::condition_variable rgbCondition[MAX_DEVICE_COUNT];

// std::map<uint8_t, std::shared_ptr<ob::Frame>> colorFrames;
std::map<uint8_t, std::shared_ptr<ob::Frame>> depthFrames;

std::map<uint8_t, std::queue<std::shared_ptr<ob::Frame>>> colorFrameQueues;
std::map<uint8_t, std::queue<std::shared_ptr<ob::Frame>>> RGBFrameQueues;

std::map<uint8_t, std::queue<std::shared_ptr<ob::Frame>>> depthFrameQueues;

std::vector<std::shared_ptr<ob::Device>> streamDevList;
std::vector<std::shared_ptr<ob::Device>> configDevList;
std::vector<std::shared_ptr<DeviceConfigInfo>> deviceConfigList;

std::vector<std::shared_ptr<PipelineHolder>> pipelineHolderList;

std::condition_variable waitRebootCompleteCondition;
std::mutex rebootingDevInfoListMutex;
std::vector<std::shared_ptr<ob::DeviceInfo>> rebootingDevInfoList;

std::queue<std::vector<std::shared_ptr<ob::Frame>>> framesVecQueue;

std::map<uint8_t, uint64_t> framesTimeStampLastMap;
std::map<uint8_t, int> framesCountMap;

OBFrameType mapFrameType(OBSensorType sensorType);

OBMultiDeviceSyncMode textToOBSyncMode(const char *text);
std::string readFileContent(const char *filePath);
bool loadConfigFile();
int configMultiDeviceSync();
int testMultiDeviceSync();
bool checkDevicesWithDeviceConfigs(
    const std::vector<std::shared_ptr<ob::Device>> &deviceList);
int strcmp_nocase(const char *str0, const char *str1);

std::shared_ptr<PipelineHolder> createPipelineHolder(
    std::shared_ptr<ob::Device> device, OBSensorType sensorType,
    int deviceIndex);
void startStream(std::shared_ptr<PipelineHolder> pipelineHolder);
void stopStream(std::shared_ptr<PipelineHolder> pipelineHolder);

void handleColorStream(int devIndex, std::shared_ptr<ob::Frame> frame);
void handleDepthStream(int devIndex, std::shared_ptr<ob::Frame> frame);

void MJPEGToRGB(unsigned char *data, unsigned int dataSize, unsigned char *outBuffer);

void decodeProcess(int deviceIndex);

ob::Context context;

void wait_any_key() { system("pause"); }

int main(int argc, char **argv) {

  std::cout<<"libobsensor version: "<<ob::Version::getVersion()<<std::endl;

  std::cout << "Please select options: " << std::endl;
  std::cout << " 0 --> config devices" << std::endl;
  std::cout << " 1 --> start stream" << std::endl;
  std::cout << "input: ";
  int index = -1;
  std::cin >> index;
  std::cout << std::endl;

  {
    for(int deviceIndex=0; deviceIndex < MAX_DEVICE_COUNT; deviceIndex++){
        framesTimeStampLastMap[deviceIndex] = 0;
    }
  }

  int exitValue = -1;
  if (index == 0) {
    exitValue = configMultiDeviceSync();
    // Only after the configuration is successful, the follow-up test is allowed
    // to continue
    if (exitValue == 0) {
      exitValue = testMultiDeviceSync();
    }
  } else if (index == 1) {
    exitValue = testMultiDeviceSync();
  } else {
    std::cout << "invalid index. " << std::endl;
    std::cout << "Please press any key to exit" << std::endl;
  }

  if (exitValue != 0) {
    wait_any_key();
  }

  return exitValue;
}

int configMultiDeviceSync() try {
  if (!loadConfigFile()) {
    std::cout << "load config failed" << std::endl;
    return -1;
  }

  if (deviceConfigList.empty()) {
    std::cout << "DeviceConfigList is empty. please check config file: "
              << CONFIG_FILE << std::endl;
    return -1;
  }

  // Query the list of connected devices
  auto devList = context.queryDeviceList();

  // Get the number of connected devices
  int devCount = devList->deviceCount();
  for (int i = 0; i < devCount; i++) {
    configDevList.push_back(devList->getDevice(i));
  }

  if (configDevList.empty()) {
    std::cerr << "Device list is empty. please check device connection state"
              << std::endl;
    return -1;
  }

  // write configuration to device
  for (auto config : deviceConfigList) {
    auto findItr = std::find_if(
        configDevList.begin(), configDevList.end(),
        [config](std::shared_ptr<ob::Device> device) {
          auto serialNumber = device->getDeviceInfo()->serialNumber();
          return strcmp_nocase(serialNumber, config->deviceSN.c_str()) == 0;
        });
    if (findItr != configDevList.end()) {
      auto device = (*findItr);

      auto curConfig = device->getMultiDeviceSyncConfig();

      // Update the configuration items of the configuration file, and keep the
      // original configuration for other items
      curConfig.syncMode = config->syncConfig.syncMode;
      curConfig.depthDelayUs = config->syncConfig.depthDelayUs;
      curConfig.colorDelayUs = config->syncConfig.colorDelayUs;
      curConfig.trigger2ImageDelayUs = config->syncConfig.trigger2ImageDelayUs;
      curConfig.triggerOutEnable = config->syncConfig.triggerOutEnable;
      curConfig.triggerOutDelayUs = config->syncConfig.triggerOutDelayUs;
      curConfig.framesPerTrigger = config->syncConfig.framesPerTrigger;

      device->setMultiDeviceSyncConfig(curConfig);
    }
  }

  // Reboot the device
  for (auto device : configDevList) {
    rebootingDevInfoList.push_back(device->getDeviceInfo());
    std::cout << "Device sn["
              << std::string(device->getDeviceInfo()->serialNumber())
              << "] is configured, rebooting..." << std::endl;
    try {
      device->reboot();
    } catch (ob::Error &e) {
      std::cout << "Device sn["
                << std::string(device->getDeviceInfo()->serialNumber())
                << "] is not configured, skipping...";
      // The early firmware versions of some models of devices will restart
      // immediately after receiving the restart command, causing the SDK to
      // fail to receive a response to the command request and throw an
      // exception
    }
  }
  configDevList.clear();

  // Register device change monitoring, used to assist the monitoring device to
  // reconnect after restarting
  context.setDeviceChangedCallback(
      [&](std::shared_ptr<ob::DeviceList> removedList,
          std::shared_ptr<ob::DeviceList> addedList) {
        if (addedList && addedList->deviceCount() > 0) {
          auto deviceCount = addedList->deviceCount();
          for (uint32_t i = 0; i < deviceCount; i++) {
            auto device = addedList->getDevice(i);
            auto deviceInfo = device->getDeviceInfo();
            std::cout << "addedList sn: "
                      << std::string(deviceInfo->serialNumber()) << std::endl;
            auto findItr = std::find_if(
                rebootingDevInfoList.begin(), rebootingDevInfoList.end(),
                [&deviceInfo](std::shared_ptr<ob::DeviceInfo> tmp) {
                  return strcmp_nocase(tmp->serialNumber(),
                                       deviceInfo->serialNumber()) == 0;
                });

            std::lock_guard<std::mutex> lk(rebootingDevInfoListMutex);
            if (findItr != rebootingDevInfoList.end()) {
              rebootingDevInfoList.erase(findItr);
              std::cout << "Device sn["
                        << std::string(deviceInfo->serialNumber())
                        << "] reboot complete." << std::endl;

              if (rebootingDevInfoList.empty()) {
                waitRebootCompleteCondition.notify_all();
              }
            }
          }
        }
      });

  // Wait for the device to restart
  {
    // wait 60s
    std::unique_lock<std::mutex> lk(rebootingDevInfoListMutex);
    waitRebootCompleteCondition.wait_for(
        lk, std::chrono::milliseconds(60000),
        [&]() { return rebootingDevInfoList.empty(); });

    // device restart failed
    if (!rebootingDevInfoList.empty()) {
      std::cerr << "Device not found after reboot. not found deviceCount: "
                << rebootingDevInfoList.size() << std::endl;
      for (auto devInfo : rebootingDevInfoList) {
        std::cout << "not found deviceSN: "
                  << std::string(devInfo->serialNumber()) << std::endl;
      }
      return -1;
    }

    // restart successfully
    std::cout << "All device update config and reboot complete." << std::endl;
  }

  // Logout callback to avoid affecting the next test multi-machine
  // synchronization
  context.setDeviceChangedCallback(nullptr);

  return 0;
} catch (ob::Error &e) {
  std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs()
            << "\nmessage:" << e.getMessage()
            << "\ntype:" << e.getExceptionType() << std::endl;
  wait_any_key();
  exit(EXIT_FAILURE);
}

int testMultiDeviceSync() try {
    
    std::vector<std::thread> threads;
    for (auto& th : threads) {
        if (th.joinable()) {
            th.join();
        }
    }

    for (int index = 0; index < MAX_DEVICE_COUNT; index++) {
        threads.push_back(std::thread(decodeProcess, index));
    }



    streamDevList.clear();
    // Query the list of connected devices
    auto devList = context.queryDeviceList();

    // Get the number of connected devices
    int devCount = devList->deviceCount();
    for (int i = 0; i < devCount; i++) {
        streamDevList.push_back(devList->getDevice(i));
    }

    if (streamDevList.empty()) {
        std::cerr << "Device list is empty. please check device connection state"
                << std::endl;
        return -1;
    }

    // traverse the device list and create the device
    std::vector<std::shared_ptr<ob::Device>> primary_devices;
    std::vector<std::shared_ptr<ob::Device>> secondary_devices;
    for (auto dev : streamDevList) {
        auto config = dev->getMultiDeviceSyncConfig();
        if (config.syncMode == OB_MULTI_DEVICE_SYNC_MODE_PRIMARY) {
        primary_devices.push_back(dev);
        } else {
        secondary_devices.push_back(dev);
        }
    }

    if (primary_devices.empty()) {
        std::cerr << "WARNING primary_devices is empty!!!" << std::endl;
    }

    // Start the multi-device time synchronization function
    context.enableDeviceClockSync(3600000);  // update and sync every hour

    std::cout << "Secondary devices start..." << std::endl;
    int deviceIndex = 0;  // Sencondary device display first
    for (auto itr = secondary_devices.begin(); itr != secondary_devices.end();
        itr++) {
        auto depthHolder = createPipelineHolder(*itr, OB_SENSOR_DEPTH, deviceIndex);
        pipelineHolderList.push_back(depthHolder);
        startStream(depthHolder);

        // auto colorHolder = createPipelineHolder(*itr, OB_SENSOR_COLOR, deviceIndex);
        // pipelineHolderList.push_back(colorHolder);
        // startStream(colorHolder);

        deviceIndex++;
    }

    // Delay and wait for 5s to ensure that the initialization of the slave device
    // is completed
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    std::cout << "Primary device start..." << std::endl;
    deviceIndex = secondary_devices
                        .size();  // Primary device display after primary devices.
    for (auto itr = primary_devices.begin(); itr != primary_devices.end();
        itr++) {
        // auto depthHolder = createPipelineHolder(*itr, OB_SENSOR_DEPTH, deviceIndex);
        // startStream(depthHolder);
        // pipelineHolderList.push_back(depthHolder);

        auto colorHolder = createPipelineHolder(*itr, OB_SENSOR_COLOR, deviceIndex);
        startStream(colorHolder);
        pipelineHolderList.push_back(colorHolder);

        deviceIndex++;
    }

    // Create a window for rendering and set the resolution of the window
    Window app("MultiDeviceSyncViewer", 1080, 720*2, RENDER_GRID);
    app.setShowInfo(false);

    while (app) {
        // // Get the key value of the key event
        auto key = app.waitKey();
        // if (key == 'S' || key == 's') {
        //     std::cout << "syncDevicesTime..." << std::endl;
        //     context.enableDeviceClockSync(3600000);  // Manual update synchronization
        // } else if (key == 'T' || key == 't') {
        //     // software trigger
        //     for (auto &dev : streamDevList) {
        //         auto multiDeviceSyncConfig = dev->getMultiDeviceSyncConfig();
        //         if (multiDeviceSyncConfig.syncMode ==
        //             OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING) {
        //         dev->triggerCapture();
        //         }
        //     }
        // }

        std::vector<std::shared_ptr<ob::Frame>> framesVec;
        {
            uint64_t baseDeviceTimeStamp = 0;
            // if(RGBFrameQueues[MAX_DEVICE_COUNT-1].size() > 0){
            //     // baseDeviceTimeStamp = RGBFrameQueues[MAX_DEVICE_COUNT-1].front()->timeStamp();
            //     baseDeviceTimeStamp = depthFrameQueues[MAX_DEVICE_COUNT-1].front()->timeStamp();
            // }
            if(depthFrameQueues[MAX_DEVICE_COUNT-1].size() > 0){
                baseDeviceTimeStamp = depthFrameQueues[MAX_DEVICE_COUNT-1].front()->timeStamp();
            }


            for (int i = 0; i < MAX_DEVICE_COUNT; i++) {
                std::unique_lock<std::mutex> depthLock(depthFrameMutex[i]);
                depthCondition[i].wait(depthLock, [i]{ return!depthFrameQueues[i].empty(); });
                
                std::cout << "xxxxxxxxxxxxxxx" << std::endl;
                auto depthFrame = depthFrameQueues[i].front();
                auto depthTimeStampMs = depthFrame->timeStamp();
                long long frameInternal = depthTimeStampMs - baseDeviceTimeStamp;
                if(frameInternal > MAX_INTERVAL_TIME){
                    continue;
                }else if(frameInternal < -MAX_INTERVAL_TIME){
                    depthFrameQueues[i].pop();
                    i--;
                    continue;
                }else{
                    framesVec.emplace_back(depthFrame);
                    depthFrameQueues[i].pop();
                }
            }
            

            // for (int i = 0; i < MAX_DEVICE_COUNT; i++) {
            //     std::unique_lock<std::mutex> lock(rgbFrameMutex[i]);
            //     rgbCondition[i].wait(lock, [i]{ return!RGBFrameQueues[i].empty(); });
                
            //     auto colorFrame = RGBFrameQueues[i].front();
            //     auto colorTimeStampMs = colorFrame->timeStamp();
            //     long long frameInternal = colorTimeStampMs - baseDeviceTimeStamp;
            //     if(frameInternal > MAX_INTERVAL_TIME){
            //         continue;
            //     }else if(frameInternal < -MAX_INTERVAL_TIME){
            //         RGBFrameQueues[i].pop();
            //         i--;
            //         continue;
            //     }else{
            //         framesVec.emplace_back(colorFrame);
            //         RGBFrameQueues[i].pop();
            //     }
            // }

            int franeVecSize = framesVec.size();
            std::cout << "********" << franeVecSize << std::endl;
            if(franeVecSize == MAX_DEVICE_COUNT){
                // app.addToRender(framesVec);

                auto framesVecQueueSize = framesVecQueue.size();
                if(framesVecQueueSize > 10){
                    framesVecQueue.pop();
                }
                framesVecQueue.push(framesVec);
            }
        }
        // Render a set of frame in the window, where the depth and color frames of
        // all devices will be rendered.
        app.addToRender(framesVec);
    }

    // close data stream
    for (auto itr = pipelineHolderList.begin(); itr != pipelineHolderList.end();
        itr++) {
        stopStream(*itr);
    }
    pipelineHolderList.clear();

    //   std::lock_guard<std::mutex> lock(frameMutex);
    //   depthFrames.clear();
    //   colorFrames.clear();

    // Release resource
    streamDevList.clear();
    configDevList.clear();
    deviceConfigList.clear();
    return 0;
} catch (ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs()
                << "\nmessage:" << e.getMessage()
                << "\ntype:" << e.getExceptionType() << std::endl;
    wait_any_key();
    exit(EXIT_FAILURE);
}

std::shared_ptr<PipelineHolder> createPipelineHolder(
    std::shared_ptr<ob::Device> device, OBSensorType sensorType,
    int deviceIndex) {
  PipelineHolder *pHolder = new PipelineHolder();
  pHolder->pipeline = std::shared_ptr<ob::Pipeline>(new ob::Pipeline(device));
  pHolder->sensorType = sensorType;
  pHolder->deviceIndex = deviceIndex;
  pHolder->deviceSN = std::string(device->getDeviceInfo()->serialNumber());

  return std::shared_ptr<PipelineHolder>(pHolder);
}

void startStream(std::shared_ptr<PipelineHolder> holder) {
  std::cout << "startStream. " << holder << std::endl;
  try {
    auto pipeline = holder->pipeline;
    // Configure which streams to enable or disable for the Pipeline by creating
    // a Config
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    
    if(holder->sensorType == OB_SENSOR_COLOR) {
      // config->enableVideoStream(OB_STREAM_COLOR, 3840, 2160, 25, OB_FORMAT_MJPG);
      // config->enableVideoStream(OB_STREAM_COLOR, 2560, 1440, 25, OB_FORMAT_MJPG);
      // config->enableVideoStream(OB_STREAM_COLOR, 1920, 1080, 30, OB_FORMAT_MJPG);
    }else if(holder->sensorType == OB_SENSOR_DEPTH){
      config->enableVideoStream(OB_STREAM_DEPTH, 512, 512, 30, OB_FORMAT_Y16);
    }else{
      // get Stream Profile.
      auto profileList = pipeline->getStreamProfileList(holder->sensorType);
      auto streamProfile = profileList->getProfile(OB_PROFILE_DEFAULT)
                              ->as<ob::VideoStreamProfile>();
      config->enableStream(streamProfile);
    }

    auto frameType = mapFrameType(holder->sensorType);
    auto deviceIndex = holder->deviceIndex;
    pipeline->start(config, [frameType, deviceIndex](
                                std::shared_ptr<ob::FrameSet> frameSet) {
      auto frame = frameSet->getFrame(frameType);
      // // std::cout << "frameIndex:" << deviceIndex << std::endl;
      if (frame) {
        if (frameType == OB_FRAME_COLOR) {
          handleColorStream(deviceIndex, frame);
        } else if (frameType == OB_FRAME_DEPTH) {
          handleDepthStream(deviceIndex, frame);
        }
      }
    });
  } catch (ob::Error &e) {
    std::cerr << "startStream failed. " << "function:" << e.getName()
              << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage()
              << "\ntype:" << e.getExceptionType() << std::endl;
  }
}

void stopStream(std::shared_ptr<PipelineHolder> holder) {
  try {
    std::cout << "stopStream " << holder << std::endl;
    holder->pipeline->stop();
  } catch (ob::Error &e) {
    std::cerr << "stopStream failed. " << "function:" << e.getName()
              << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage()
              << "\ntype:" << e.getExceptionType() << std::endl;
  }
}

void handleColorStream(int devIndex, std::shared_ptr<ob::Frame> frame) {
    std::lock_guard<std::mutex> lock(frameMutex[devIndex]);
//   std::cout << "Device#" << devIndex << ", color frame index=" << frame->index()
//             << ", timestamp=" << frame->timeStamp()
//             << ", system timestamp=" << frame->systemTimeStamp() << std::endl;

//   colorFrames[devIndex] = frame;
    if(colorFrameQueues[devIndex].size() < 20){
        colorFrameQueues[devIndex].push(frame);
    }else{
        std::cout << "colorFrameQueues overflow. devIndex=" << devIndex << std::endl;
        colorFrameQueues[devIndex].pop();
        colorFrameQueues[devIndex].push(frame);
    }
    colorCondition[devIndex].notify_one();
    
}

void handleDepthStream(int devIndex, std::shared_ptr<ob::Frame> frame) {
  std::lock_guard<std::mutex> lock(depthFrameMutex[devIndex]);
  // std::cout << "Device#" << devIndex << ", depth frame index=" << frame->index()
  //           << ", timestamp=" << frame->timeStamp()
  //           << ", system timestamp=" << frame->systemTimeStamp() << std::endl;
  if(depthFrameQueues[devIndex].size() < 20){
      depthFrameQueues[devIndex].push(frame);
  }else{
      std::cout << "depthFrameQueues overflow. devIndex=" << devIndex << std::endl;
      depthFrameQueues[devIndex].pop();
      depthFrameQueues[devIndex].push(frame);
  }
  depthCondition[devIndex].notify_one();

  // depthFrames[devIndex] = frame;
}

std::string readFileContent(const char *filePath) {
  std::ostringstream oss;

  long length = 0;
  long readSum = 0;
  int readSize = 0;
  char buf[512];
  bool isOpened = false;
  bool success = false;
  std::ifstream file;
  file.exceptions(std::fstream::badbit | std::fstream::failbit);
  try {
    file.open(filePath, std::fstream::in);
    isOpened = true;
    file.seekg(0, std::fstream::end);
    length = file.tellg();
    file.seekg(0, std::fstream::beg);

    while (!file.eof() && readSum < length) {
      readSize = (std::min)((long)512, length - readSum);
      file.read(buf, readSize);
      if (file.gcount() > 0) {
        oss << std::string(buf, file.gcount());
        readSum += file.gcount();
      }
    }
    success = true;
  } catch (std::fstream::failure e) {
    if ((file.rdstate() & std::fstream::failbit) != 0 &&
        (file.rdstate() & std::fstream::eofbit) != 0) {
      if (readSize > 0 && file.gcount() > 0) {
        oss << std::string(buf, file.gcount());
        readSum += file.gcount();
      }
      success = true;
    } else {
      std::string errorMsg = (nullptr != e.what() ? std::string(e.what()) : "");
      std::cerr << "open or reading file: " << std::string(filePath)
                << ", errorMsg: " << errorMsg << std::endl;
    }
  }

  if (isOpened) {
    try {
      file.close();
    } catch (std::fstream::failure e) {
      std::string errorMsg = (nullptr != e.what() ? std::string(e.what()) : "");
      std::cerr << "close file: " << std::string(filePath)
                << ", errorMsg: " << errorMsg << std::endl;
    }
  }

  return success ? oss.str() : "";
}

bool loadConfigFile() {
  auto content = readFileContent(CONFIG_FILE);
  if (content.empty()) {
    std::cerr << "load config file failed." << std::endl;
    return false;
  }

  int deviceCount = 0;

  cJSON *rootElem = cJSON_Parse(content.c_str());
  if (rootElem == nullptr) {
    const char *errMsg = cJSON_GetErrorPtr();
    std::cout << std::string(errMsg) << std::endl;
    cJSON_Delete(rootElem);
    return true;
  }

  std::shared_ptr<DeviceConfigInfo> devConfigInfo = nullptr;
  cJSON *deviceElem = nullptr;
  cJSON *devicesElem = cJSON_GetObjectItem(rootElem, "devices");
  cJSON_ArrayForEach(deviceElem, devicesElem) {
    devConfigInfo = std::make_shared<DeviceConfigInfo>();
    memset(&devConfigInfo->syncConfig, 0, sizeof(devConfigInfo->syncConfig));
    devConfigInfo->syncConfig.syncMode = OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;

    cJSON *snElem = cJSON_GetObjectItem(deviceElem, "sn");
    if (cJSON_IsString(snElem) && snElem->valuestring != nullptr) {
      devConfigInfo->deviceSN = std::string(snElem->valuestring);
    }

    cJSON *deviceConfigElem = cJSON_GetObjectItem(deviceElem, "syncConfig");
    if (cJSON_IsObject(deviceConfigElem)) {
      cJSON *numberElem = nullptr;
      cJSON *strElem = nullptr;
      cJSON *bElem = nullptr;
      strElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "syncMode");
      if (cJSON_IsString(strElem) && strElem->valuestring != nullptr) {
        devConfigInfo->syncConfig.syncMode =
            textToOBSyncMode(strElem->valuestring);
        std::cout << "config[" << (deviceCount++)
                  << "]: SN=" << std::string(devConfigInfo->deviceSN)
                  << ", mode=" << strElem->valuestring << std::endl;
      }

      numberElem =
          cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "depthDelayUs");
      if (cJSON_IsNumber(numberElem)) {
        devConfigInfo->syncConfig.depthDelayUs = numberElem->valueint;
      }

      numberElem =
          cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "colorDelayUs");
      if (cJSON_IsNumber(numberElem)) {
        devConfigInfo->syncConfig.colorDelayUs = numberElem->valueint;
      }

      numberElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem,
                                                    "trigger2ImageDelayUs");
      if (cJSON_IsNumber(numberElem)) {
        devConfigInfo->syncConfig.trigger2ImageDelayUs = numberElem->valueint;
      }

      numberElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem,
                                                    "triggerOutDelayUs");
      if (cJSON_IsNumber(numberElem)) {
        devConfigInfo->syncConfig.triggerOutDelayUs = numberElem->valueint;
      }

      bElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem,
                                               "triggerOutEnable");
      if (cJSON_IsBool(bElem)) {
        devConfigInfo->syncConfig.triggerOutEnable = (bool)bElem->valueint;
      }

      bElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem,
                                               "framesPerTrigger");
      if (cJSON_IsNumber(bElem)) {
        devConfigInfo->syncConfig.framesPerTrigger = bElem->valueint;
      }
    }

    if (OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN !=
        devConfigInfo->syncConfig.syncMode) {
      deviceConfigList.push_back(devConfigInfo);
    } else {
      std::cerr << "invalid sync mode of deviceSN: " << devConfigInfo->deviceSN
                << std::endl;
    }

    devConfigInfo = nullptr;
  }

  cJSON_Delete(rootElem);
  return true;
}

OBMultiDeviceSyncMode textToOBSyncMode(const char *text) {
  if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN") == 0) {
    return OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;
  } else if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_STANDALONE") == 0) {
    return OB_MULTI_DEVICE_SYNC_MODE_STANDALONE;
  } else if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_PRIMARY") == 0) {
    return OB_MULTI_DEVICE_SYNC_MODE_PRIMARY;
  } else if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_SECONDARY") == 0) {
    return OB_MULTI_DEVICE_SYNC_MODE_SECONDARY;
  } else if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED") == 0) {
    return OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED;
  } else if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING") ==
             0) {
    return OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING;
  } else if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING") ==
             0) {
    return OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING;
  } else {
    return OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;
  }
}

int strcmp_nocase(const char *str0, const char *str1) {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  return _strcmpi(str0, str1);
#else
  return strcasecmp(str0, str1);
#endif
}

OBFrameType mapFrameType(OBSensorType sensorType) {
  switch (sensorType) {
    case OB_SENSOR_COLOR:
      return OB_FRAME_COLOR;
    case OB_SENSOR_IR:
      return OB_FRAME_IR;
    case OB_SENSOR_IR_LEFT:
      return OB_FRAME_IR_LEFT;
    case OB_SENSOR_IR_RIGHT:
      return OB_FRAME_IR_RIGHT;
    case OB_SENSOR_DEPTH:
      return OB_FRAME_DEPTH;
    default:
      return OBFrameType::OB_FRAME_UNKNOWN;
  }
}

std::ostream &operator<<(std::ostream &os, const PipelineHolder &holder) {
  os << "deviceSN: " << holder.deviceSN << ", sensorType: ";
  if (holder.sensorType == OB_SENSOR_COLOR) {
    os << "OB_SENSOR_COLOR";
  } else if (holder.sensorType == OB_SENSOR_DEPTH) {
    os << "OB_SENSOR_DEPTH";
  } else {
    os << (int)holder.sensorType;
  }

  os << ", deviceIndex: " << holder.deviceIndex;

  return os;
}

// void MJPEGToRGB(unsigned char *data, unsigned int dataSize, unsigned char *outBuffer)
// {   
//     // 1. 将元数据装填到packet
//     AVPacket *avPkt = av_packet_alloc();
//     avPkt->size = dataSize;
//     avPkt->data = data;

//     // 2. 创建并配置codecContext
//     AVCodec *mjpegCodec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
//     AVCodecContext* codecCtx = avcodec_alloc_context3(mjpegCodec);
//     avcodec_get_context_defaults3(codecCtx, mjpegCodec);
//     avcodec_open2(codecCtx, mjpegCodec, nullptr);

//     // 3. 解码
//     //avcodec_decode_video2(codecCtx, &outFrame, &lineLength, &avPkt);  // 接口被弃用，使用下边接口代替
//     auto ret = avcodec_send_packet(codecCtx, avPkt);
//     if (ret >=0) {
//         AVFrame* YUVFrame = av_frame_alloc();
//         ret = avcodec_receive_frame(codecCtx, YUVFrame);
//         if (ret >= 0) { 

//             // 4.YUV转RGB24
//             AVFrame* RGB24Frame = av_frame_alloc();
//             struct SwsContext* convertCxt = sws_getContext(
//                 YUVFrame->width, YUVFrame->height, AV_PIX_FMT_YUV420P,
//                 YUVFrame->width, YUVFrame->height, AV_PIX_FMT_RGB24,
//                 SWS_POINT, NULL, NULL, NULL
//             );

//             // outBuffer将会分配给RGB24Frame->data,AV_PIX_FMT_RGB24格式只分配到RGB24Frame->data[0]
//             av_image_fill_arrays(
//                 RGB24Frame->data, RGB24Frame->linesize, outBuffer,  
//                 AV_PIX_FMT_RGB24, YUVFrame->width, YUVFrame->height,
//                 1
//             );
//             sws_scale(convertCxt, YUVFrame->data, YUVFrame->linesize, 0, YUVFrame->height, RGB24Frame->data, RGB24Frame->linesize);

//             // 5.清除各对象/context -> 释放内存
//             // free context and avFrame
//             sws_freeContext(convertCxt);
//             av_frame_free(&RGB24Frame);
//             // RGB24Frame.
//         }
//         // free context and avFrame
//         av_frame_free(&YUVFrame);
//     }
//     // free context and avFrame
//     av_packet_unref(avPkt);
//     av_packet_free(&avPkt);
//     avcodec_free_context(&codecCtx);
// }

// Get system mill timestamp.
int64_t get_milliseconds_timestamp()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

void decodeProcess(int deviceIndex){
    while(true){
        std::unique_lock<std::mutex> lock(frameMutex[deviceIndex]);
        colorCondition[deviceIndex].wait(lock, [deviceIndex]{ return!colorFrameQueues[deviceIndex].empty(); });

        auto frame = colorFrameQueues[deviceIndex].front();
        colorFrameQueues[deviceIndex].pop();
        lock.unlock();


        auto filterTimeLast = get_milliseconds_timestamp();
        // Create a format conversion Filter
        ob::FormatConvertFilter formatConvertFilter;
        formatConvertFilter.setFormatConvertType(FORMAT_MJPG_TO_RGB);
        auto colorFrame = formatConvertFilter.process(frame)->as<ob::ColorFrame>();
        auto filterTimeCurrent = get_milliseconds_timestamp();
        // std::cout << "filter time: " << filterTimeCurrent-filterTimeLast << std::endl;

        // auto colorFrame = frame->as<ob::ColorFrame>();
        // auto width = colorFrame->width();
        // auto height = colorFrame->height();
        // auto data = reinterpret_cast<uint8_t *>(colorFrame->data());
        // auto dataSize = colorFrame->dataSize();
        // auto rgb24DataFrame = ob::FrameHelper::createFrame(OB_FRAME_COLOR, OB_FORMAT_RGB, width, height, 0);
        // auto rgb24Data = static_cast<uint8_t *>(rgb24DataFrame->data());
        // MJPEGToRGB(data, dataSize, rgb24Data);
        
        {
            std::unique_lock<std::mutex> rgbLock(rgbFrameMutex[deviceIndex]);
            framesCountMap[deviceIndex] += 1;
            auto rgbQueueSize = RGBFrameQueues[deviceIndex].size();
            if(rgbQueueSize > 10){
                RGBFrameQueues[deviceIndex].pop();
            }
            auto frameTimeStampCurrent = get_milliseconds_timestamp();
            auto interval = frameTimeStampCurrent - framesTimeStampLastMap[deviceIndex];
            if(interval > 1000){
                double fps = framesCountMap[deviceIndex] * 1000.0 / interval;
                framesTimeStampLastMap[deviceIndex] = frameTimeStampCurrent;
                framesCountMap[deviceIndex] = 0;
                // std::cout << "deviceIndex: " << deviceIndex << ", fps: " << fps << std::endl;
            }
            RGBFrameQueues[deviceIndex].push(colorFrame);
            rgbCondition[deviceIndex].notify_one();
        }
    }
}