# Multi-Device Synchronization Example

Demonstrates multi-device synchronization. This sample supports network devices, USB devices, and GMSL devices.

- The primary purpose of this repository is to demonstrate the multi-device synchronization functionality of different types of Orbbec devices, including detailed instructions for hardware connections and software configuration. 
- The v2-main branch is based on the [multi device sync sample](https://github.com/orbbec/OrbbecSDK_v2/tree/main/examples/https://github.com/orbbec/OrbbecSDK_v2/tree/main/examples/3.advanced.multi_devices_sync) and [gmsl trigger sample](https://github.com/orbbec/OrbbecSDK_v2/tree/main/examples/3.advanced.multi_devices_sync_gmsltrigger) provided by [Orbbec SDK v2](https://github.com/orbbec/OrbbecSDK_v2). If there are any updates or changes to the samples in Orbbec SDK v2, please refer to the official implementation in Orbbec SDK v2 as the authoritative version.
- The legacy main branch is based on Orbbec SDK v1 and only demonstrates multi-device synchronization for Femto Mega.


## Supported Device Series


<table border="1" style="border-collapse: collapse; text-align: left; width: 100%;">
  <thead>
    <tr style="background-color: #1f4e78; color: white; text-align: center;">
      <th>Product Series</th>
      <th>Product</th>
      <th>Connection</th>
      <th>Sync Documentation</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align: center; font-weight: bold;">Gemini 305</td>
      <td>Gemini 305</td>
      <td>USB</td>
      <td><a href="docs/Gemini301Series.md">Gemini301Series.md </a></td>
    </tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Gemini 305g</td>
      <td>Gemini 305g</td>
      <td>USB/GMSL</td>
      <td><a href="docs/Gemini301Series.md">Gemini301Series.md</a></td>
    </tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Gemini 435Le</td>
      <td>Gemini 435Le</td>
      <td>Ethernet</td>
      <td><a href="docs/Gemini435Le.md">Gemini435Le.md </a></td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Gemini 330</td>
      <td>Gemini 335Le</td>
      <td>Ethernet</td>
      <td><a href="docs/Gemini330Series.md">Gemini330Series.md </a></td>
    </tr>
    <tr>
      <td>Gemini 335/335L/336/336L</td>
      <td> USB </td>
      <td><a href="docs/Gemini330Series.md">Gemini330Series.md</a></td>
    </tr>
    <tr>
      <td>Gemini 335Lg</td>
      <td> GMSL</td>
      <td><a href="docs/Gemini330Series.md">Gemini330Series.md </a></td>
    </tr>
    <tr>
      <td rowspan="1" style="text-align: center; font-weight: bold;">Gemini 2</td>
      <td>Gemini 2/2L/215/210</td>
      <td> USB </td>
      <td><a href="docs/Gemini2Series.md">Gemini2Series.md </a></td>
    </tr>
        <tr>
      <td rowspan="1" style="text-align: center; font-weight: bold;">Astra</td>
      <td>Astra 2</td>
      <td>USB</td>
      <td><a href="docs/Astra2.md">Astra2.md </a></td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Femto</td>
      <td>Femto Bolt</td>
      <td>USB</td>
      <td><a href="docs/FemtoSeries.md">FemtoSeries.md </a></td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td>USB/Ethernet</td>
      <td><a href="docs/FemtoSeries.md">FemtoSeries.md </a></td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td>Ethernet</td>
      <td><a href="docs/FemtoSeries.md">FemtoSeries.md </a></td>
    </tr>
  </tbody>
</table>

## Program Overview

This project builds two standalone executables:

| Program | Connection | Platform | Function |
| ------- | ---------- | -------- | -------- |
| **MultiDeviceSync** | USB / Ethernet /GMSL | Windows / Linux x64 / Linux ARM64 | Multi-device sync configuration, data stream acquisition, real-time preview |
| **MultiDeviceSyncGmslTrigger** | GMSL | Linux ARM64 (NVIDIA Jetson AGX Orin, NVIDIA Jetson Orin NX) | GMSL PWM hardware trigger signal source |

- Notes: **GMSL** devices require two applications: MultiDeviceSync and MultiDeviceSyncGmslTrigger. The MultiDeviceSyncGmslTrigger application is responsible for sending trigger signals, while **USB and Ethernet** devices only require the MultiDeviceSync application.

## Quick Start

### 1. Build

```bash
cd Multi-Device-Synchronization-Example
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

Build artifacts are in the `build/bin/` directory:

- **Windows**: `MultiDeviceSync.exe`
- **Linux**: `MultiDeviceSync`, `MultiDeviceSyncGmslTrigger`

### 2. Configuration

The program reads `./MultiDeviceSyncConfig.json` from the current working directory by default. Device serial numbers can be found using the OrbbecViewer tool or in the program log after connecting devices.

#### Configuration File Structure

The following example includes all optional fields; use as needed based on your device series:

```json
{
    "version": "1.0.1",
    "configTime": "2026/05/07",
    "devices": [
        {
            "sn": "CP123456789",
            "syncConfig": {
                "syncMode": "OB_MULTI_DEVICE_SYNC_MODE_PRIMARY",
                "depthDelayUs": 0,
                "colorDelayUs": 0,
                "trigger2ImageDelayUs": 0,
                "triggerOutEnable": true,
                "triggerOutDelayUs": 0,
                "framesPerTrigger": 1
            }
        }
    ]
}
```

**Field descriptions:**

| Field | Type | Required | Description |
| ----- | ---- | -------- | ----------- |
| `sn` | string | Yes | Device serial number, used to match connected devices |
| `syncConfig.syncMode` | string | Yes | Sync mode, see table below for available values |
| `syncConfig.depthDelayUs` | int | Yes | IR/Depth/ToF trigger signal input delay (microseconds), increment by 160us per device to achieve staggered exposure |
| `syncConfig.colorDelayUs` | int | Yes | RGB trigger signal input delay (microseconds), typically 0 |
| `syncConfig.trigger2ImageDelayUs` | int | Yes | Delay from trigger signal input to image capture (microseconds), typically 0 |
| `syncConfig.triggerOutEnable` | bool | Yes | Device trigger signal output enable switch. Set to true for PRIMARY/SECONDARY mode, false for SOFTWARE_TRIGGERING and HARDWARE_TRIGGERING mode |
| `syncConfig.triggerOutDelayUs` | int | Yes | Device trigger signal output delay (microseconds), typically 0 |
| `syncConfig.framesPerTrigger` | int | Yes | Number of frames captured per trigger, only effective in software and hardware trigger modes |

**`syncMode` available values:**

| Mode | Description |
| ---- | ----------- |
| `OB_MULTI_DEVICE_SYNC_MODE_PRIMARY` | Primary mode, outputs sync trigger signal |
| `OB_MULTI_DEVICE_SYNC_MODE_SECONDARY` | Secondary mode, receives trigger signal |
| `OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING` | Software trigger mode, PC controls capture timing |
| `OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING` | Hardware trigger mode, triggered by external hardware signal |
| `OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED` | Secondary synced variant, starts capturing immediately, auto-aligns when trigger signal is received |

### 3. Run MultiDeviceSync

```bash
./MultiDeviceSync
```

- Enter `0`: Configure device sync mode  and start streaming
- Enter `1`: Start streaming directly (use when devices are already configured)


**Keyboard shortcuts:**

| Key | Function |
| --- | -------- |
| `T` | Software trigger capture |
| `ESC` | Stop streaming and exit |

### 4. Run MultiDeviceSyncGmslTrigger

For GMSL2 connections to NVIDIA Jetson platforms. Devices must be configured as `HARDWARE_TRIGGERING` mode.

```bash
./MultiDeviceSyncGmslTrigger
```

- Enter `0`: Set trigger frequency (FPS)
- Enter `1`: Start PWM trigger signal
- Enter `2`: Stop PWM trigger signal
- Enter `3`: Exit

## Notes

1. Femto series and Gemini 2 series sync configurations are written to Flash and persist after power-off; frequent configuration will reduce Flash lifespan. Gemini 305 and Gemini 330 sync configurations do not persist after power-off and must be reconfigured each time the device is powered on.
2. **MultiDeviceSyncGmslTrigger** only provides PWM trigger signal source and does not include data stream acquisition or preview functionality.
3. When exiting MultiDeviceSyncGmslTrigger, select `3` to ensure the `/dev/camsync` device is properly closed.

## Project Structure

```text
├── CMakeLists.txt                              # Build configuration
├── MultiDeviceSync/                            # Standard USB/Ethernet multi-device sync
│   └── MultiDeviceSync.cpp                     #   Main program
├── MultiDeviceSyncGmslTrigger/                 # GMSL PWM hardware trigger
│   └── MultiDeviceSyncGmslTrigger.cpp          #   Main program
├── common/                                     # Shared core modules
│   ├── PipelineHolder.hpp/cpp                  #   Pipeline wrapper
│   ├── FramePairingManager.hpp/cpp             #   Cross-device frame pairing
│   └── utils/                                  #   Utility library
│       ├── cJSON.c/h                           #     JSON parser
│       ├── utils.cpp/hpp                       #     Basic utility functions
│       ├── utils_c.c/h                         #     C utility functions
│       ├── utils_opencv.cpp/hpp                #     OpenCV visualization windows
│       └── utils_types.h                       #     Common type definitions
├── config/
│   ├── OrbbecSDKConfig.xml                     # SDK device configuration
│   ├── MultiDeviceSyncConfig.json              # Current sync configuration
├── docs/                                       # Per-series documentation
│   ├── Astra2.md
│   ├── FemtoSeries.md
│   ├── Gemini2Series.md
│   ├── Gemini301Series.md
│   ├── Gemini330Series.md
│   └── Gemini435Le.md
├── res/                                        # Documentation image resources
└── 3rdparty/orbbecsdk/                         # OrbbecSDK
    ├── win_x64/
    ├── linux_x86_64/
    └── linux_arm64/
```
