# Multi-Device Synchronization Example

Demonstrates multi-device synchronization. This sample supports network devices, USB devices, and GMSL devices.

- The primary purpose of this repository is to demonstrate the multi-device synchronization functionality of different types of Orbbec devices, including detailed instructions for hardware connections and software configuration. 
- The v2-main branch is based on the [multi device sync sample](https://github.com/orbbec/OrbbecSDK_v2/tree/main/examples/3.advanced.multi_devices_sync) and [gmsl trigger sample](https://github.com/orbbec/OrbbecSDK_v2/tree/main/examples/3.advanced.multi_devices_sync_gmsltrigger) provided by [Orbbec SDK v2](https://github.com/orbbec/OrbbecSDK_v2). If there are any updates or changes to the samples in Orbbec SDK v2, please refer to the official implementation in Orbbec SDK v2 as the authoritative version.
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
      <td style="text-align: center; font-weight: bold;">Gemini 309g</td>
      <td>Gemini 309g</td>
      <td>USB/GMSL</td>
      <td><a href="docs/Gemini301Series.md">Gemini301Series.md </a></td>
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
            },
            "streamConfig": {
                "depth": {
                    "width": 640,
                    "height": 480,
                    "fps": 30,
                    "format": "OB_FORMAT_Y16"
                },
                "color": {
                    "width": 640,
                    "height": 480,
                    "fps": 30,
                    "format": "OB_FORMAT_MJPG"
                }
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
| `syncConfig.depthDelayUs` | int | Yes | IR/Depth/ToF trigger signal input delay (microseconds). Default 0; see the series-specific sync documentation for recommended per-device delay values |
| `syncConfig.colorDelayUs` | int | Yes | RGB trigger signal input delay (microseconds), typically 0 |
| `syncConfig.trigger2ImageDelayUs` | int | Yes | Delay from trigger signal input to image capture (microseconds), typically 0 |
| `syncConfig.triggerOutEnable` | bool | Yes | Device trigger signal output enable switch. With a star hub: enable on the primary device only. With a daisy-chain hub: enable on the primary and all secondary devices. Forcibly set to true in PRIMARY mode |
| `syncConfig.triggerOutDelayUs` | int | Yes | Device trigger signal output delay (microseconds), typically 0 |
| `syncConfig.framesPerTrigger` | int | Yes | Number of frames captured per trigger, only effective in software and hardware trigger modes |
| `streamConfig` | object | No | Stream profile overrides; omit the whole section to use the SDK default profile |
| `streamConfig.depth` / `streamConfig.color` | object | No | Per-sensor profile request. Each field is optional: `width`, `height`, `fps` (0 = not specified), `format` (empty = not specified, e.g. `"OB_FORMAT_Y16"`, `"OB_FORMAT_MJPG"`). Unspecified fields match any value; stream start fails if the device has no matching profile |

> **Note:** All devices must be configured with the same frame rate for each stream type. Multi-device frame grouping and pairing rely on a consistent frame interval; mixing frame rates across devices will lead to inaccurate synchronization and analysis results.

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
| `S` | Sync device clocks |
| `T` | Software trigger capture |
| `ESC` | Stop streaming and exit |

#### Headless Mode

Run without the OpenCV preview window, suitable for remote sessions (SSH) or headless servers:

```bash
./MultiDeviceSync --headless
```

- Timestamp recording and sync monitor output run exactly as in normal mode
- No preview window is created
- Press `Ctrl+C` to stop: the program catches SIGINT, flushes all CSV data, and shuts down streams before exiting

### 4. Run MultiDeviceSyncGmslTrigger

For GMSL2 connections to NVIDIA Jetson platforms. Devices can be configured as `HARDWARE_TRIGGERING` or `SECONDARY_SYNCED` mode.

```bash
sudo ./MultiDeviceSyncGmslTrigger
```

- Run with `sudo` (root permission is required to open `/dev/camsync`).

- Enter `0`: Set trigger frequency (FPS)
- Enter `1`: Start PWM trigger signal
- Enter `2`: Stop PWM trigger signal
- Enter `3`: Exit

**Recommended startup order (applicable only to `HARDWARE_TRIGGERING` and `SECONDARY_SYNCED` modes):** Start the streams first, then enable the trigger signal. Run **MultiDeviceSync** first (enter `0` to configure the sync mode and start streaming), followed by **MultiDeviceSyncGmslTrigger** to activate the PWM signal. Devices in other sync modes do not rely on external triggering and are unaffected by the PWM signal.

### 5. Timestamp CSV Output

During streaming, each device and each sensor (depth/color) records its own timestamp CSV file under `./output/`:

```text
output/
  sync_depth_dev0_<SN>.csv
  sync_color_dev0_<SN>.csv
  sync_depth_dev1_<SN>.csv
  sync_color_dev1_<SN>.csv
  ...
```

Each file contains one row per captured frame:

| Field | Description |
| ----- | ----------- |
| `row_id` | Sequential row index |
| `sw_frame_num` | Software frame number from the SDK |
| `hw_frame_num` | Hardware frame number; `-1` when the device does not support it |
| `system_ts_us` | Host system timestamp (microseconds) |
| `device_ts_us` | Device-side timestamp (microseconds) |
| `global_ts_us` | Global timestamp on the host clock domain (microseconds); `0` when unsupported |

> **Note:** Some devices do not provide a global timestamp (`global_ts_us` is all `0`) or a hardware frame number (`hw_frame_num` is `-1`). This is normal and expected.

### 6. Sync Accuracy Analysis (Python)

A standalone Python script evaluates synchronization accuracy from the recorded CSV files. It performs both **multi-device** checks (cross-device timestamp alignment) and **intra-device** checks (depth-color pairing offset and frame-drop detection):

- **Script:** `scripts/analyze_sync.py`
- **No third-party dependencies** (Python standard library only)

Run it from the directory containing the CSV files (typically `output/`):

```bash
cd build/bin/output
python analyze_sync.py
```

Or analyze any folder:

```bash
python analyze_sync.py /path/to/csv_dir [options]
```

**Options:**

| Option | Default | Description |
| ------ | ------- | ----------- |
| `[csv_dir]` | current directory | Directory with the `sync_*.csv` files |
| `--fps` | auto | Frame rate used to compute the grouping tolerance (half-frame interval); auto-detects from the median frame interval, falls back to `30` when there is too little data |
| `--threshold` | `5000` | In-group timestamp range (us) above which a matched group is flagged abnormal |
| `--ts-source` | `auto` | Time base for matching: `global` / `device` / `auto` (`auto` = global when valid, otherwise degrades to device) |
| `--frame-num-source` | `auto` | Frame number used for drop detection: `hw` / `sw` / `auto` (`auto` = hw when valid, otherwise degrades to sw) |
| `--output` | `<csv_dir>` | Base output directory; writes `analysis_multi_device/` and `analysis_per_device/` under it |

**Time base selection:**

- `global` (recommended): uses `global_ts_us`, all devices share the host clock domain.
- `device`: uses `device_ts_us`, for devices that do not support a global timestamp. The script prints a RISK note: without clock sync during capture, cross-device diffs include clock offset/drift and are indicative only.
- `auto`: uses `global` when valid; otherwise degrades to `device` with a note.

**Report sections:**

1. **Multi-device** (per sensor, requires >= 2 devices): matched-group completeness, abnormal in-group timestamp range, and per-group matched CSV.
2. **Intra-device** (per device): depth-color pairing count, global/device timestamp diff range, and per-stream dropped-frame counts (detailed drop positions go to `per_device_drops.csv`).

**Output files and field reference:**

| File | Location | Description |
| ---- | -------- | ----------- |
| `sync_matched_<sensor>.csv` | `analysis_multi_device/` | One row per matched moment: group id, `diffGlobalUs`/`diffDeviceUs` (max-min across devices; empty when that time base is invalid), and each device's frame numbers and timestamps |
| `sync_failed_<sensor>.csv` | `analysis_multi_device/` | One row per unmatched frame (anchor that found no partner within tolerance, or a frame skipped during a match); the frame itself was captured and is present in the original CSV, with full original columns: `sn, rowId, swNum, hwNum, systemUs, deviceUs, globalUs` |
| `per_device_dev<N>_<SN>.csv` | `analysis_per_device/` | One row per depth-color pair of device N: `diffGlobalUs`/`diffDeviceUs` and both frames' numbers and timestamps |
| `per_device_drops.csv` | `analysis_per_device/` | One row per detected dropped frame: `sn, sensor, frameNumSource, missingFrameNum` |

CSV column meanings:

`sync_matched_<sensor>.csv`:

| Column | Description |
| ------ | ----------- |
| `groupId` | Sequential index of the matched group |
| `diffGlobalUs` | Max-min spread of `global_ts_us` across the devices in the group; empty when the global time base is invalid |
| `diffDeviceUs` | Max-min spread of `device_ts_us` across the devices in the group; empty when the device time base is invalid |
| `devN_SwNum` | Software frame number of the matched frame on device N |
| `devN_HwNum` | Hardware frame number of the matched frame on device N; `-1` when the device does not support it |
| `devN_SystemUs` | Host system timestamp (microseconds) of the matched frame on device N |
| `devN_GlobalUs` | Global timestamp (microseconds) of the matched frame on device N |
| `devN_DeviceUs` | Device-side timestamp (microseconds) of the matched frame on device N |

`sync_failed_<sensor>.csv`:

| Column | Description |
| ------ | ----------- |
| `sn` | Serial number of the device the unmatched frame belongs to |
| `rowId` | Row index of the frame in the original capture CSV |
| `swNum` | Software frame number |
| `hwNum` | Hardware frame number; `-1` when the device does not support it |
| `systemUs` | Host system timestamp (microseconds) |
| `deviceUs` | Device-side timestamp (microseconds) |
| `globalUs` | Global timestamp (microseconds) |

`per_device_dev<N>_<SN>.csv`:

| Column | Description |
| ------ | ----------- |
| `groupId` | Sequential index of the depth-color pair |
| `diffGlobalUs` | `depthGlobalUs - colorGlobalUs` (depth minus color within the device); empty when the global time base is invalid |
| `diffDeviceUs` | `depthDeviceUs - colorDeviceUs` (depth minus color within the device); empty when the device time base is invalid |
| `depthSwNum` / `colorSwNum` | Software frame number of the depth / color frame in the pair |
| `depthHwNum` / `colorHwNum` | Hardware frame number of the depth / color frame in the pair; `-1` when the device does not support it |
| `depthSystemUs` / `colorSystemUs` | Host system timestamp (microseconds) of the depth / color frame |
| `depthGlobalUs` / `colorGlobalUs` | Global timestamp (microseconds) of the depth / color frame |
| `depthDeviceUs` / `colorDeviceUs` | Device-side timestamp (microseconds) of the depth / color frame |

`per_device_drops.csv`:

| Column | Description |
| ------ | ----------- |
| `sn` | Serial number of the device the dropped frame belongs to |
| `sensor` | Stream the frame belongs to (`depth` or `color`) |
| `frameNumSource` | Frame-number source actually used for drop detection (`hw` or `sw`) |
| `missingFrameNum` | The missing frame number detected in the sequence |

**Console report fields:**

The script prints a summary report to the terminal. Header block:

| Field | Description |
| ----- | ----------- |
| `Data dir` | Directory containing the analyzed CSV files |
| `Frame rate` | Frame rate used for grouping; shows how it was obtained (auto-detected from median frame interval, or a fallback value when auto-detect fails) |
| `Threshold` | In-group range threshold; a matched group whose max-min timestamp range reaches this value is counted as Abnormal |
| `Time base` | Timestamp source used for matching (`global` / `device`) |
| `Frame num` | Frame-number source used for drop detection (`hw` / `sw`) |

Per-sensor sections (`========== Sensor: DEPTH / COLOR ==========`):

| Field | Description |
| ----- | ----------- |
| `devN(<SN>): N frames` | Number of captured frames per device for this sensor |
| `Duration` | Time span covered by all frames of this sensor, and the approximate average fps |
| `Completeness` | Percentage of anchors that formed a complete group across all devices: `groups / (groups + unmatched)`. Unmatched frames are present in the capture CSV but failed cross-device matching; they are not dropped frames |
| `Abnormal (>= N us)` | Percentage (and count) of matched groups whose in-group timestamp range reaches the threshold |
| `Matched CSV` / `Failed CSV` | Paths of the written CSV files for this sensor |

Per-device sections (intra-device):

| Field | Description |
| ----- | ----------- |
| `ts source` | Time base actually used for this device's pairing (`global` or `device`, with the degradation reason) |
| `frame num` | Frame-number source used for drop detection (`HW` or `SW`) |
| `frames` | Captured frame count of depth and color |
| `pairs` | Number of matched depth-color pairs (`unmatched` = frames left without a partner) |
| `global diff` / `device diff` | Min-max range of the depth-color timestamp difference within the device (`depth - color`); `n/a` when the time base is invalid |
| `dropped frames` | Per-stream dropped-frame count and ratio; exact positions are in `per_device_drops.csv` |
| `Drops CSV` | Path of `per_device_drops.csv`; only printed when dropped frames are detected |
| `first csv row` | First recorded frame number per stream; a value > 1 means recording started mid-stream, those earlier frames are not counted as drops |
| `CSV` | Path of the written `per_device_dev<N>_<SN>.csv` |


## Notes

1. Femto series and Gemini 2 series sync configurations are written to Flash and persist after power-off; frequent configuration will reduce Flash lifespan. Gemini 305 and Gemini 330 sync configurations do not persist after power-off and must be reconfigured each time the device is powered on.
2. **MultiDeviceSyncGmslTrigger** only provides PWM trigger signal source and does not include data stream acquisition or preview functionality.
3. When exiting MultiDeviceSyncGmslTrigger, select `3` to ensure the `/dev/camsync` device is properly closed.
4. **GMSL** connections do not support the MJPG format. When the color stream is configured with `"format": "OB_FORMAT_MJPG"` in `MultiDeviceSyncConfig.json`, change it to a format supported by the device (e.g. `OB_FORMAT_YUYV`).
5. **Exposure alignment:** the timestamp reference point is controlled by the SDK property `OB_PROP_INTRA_CAMERA_SYNC_REFERENCE_INT` (enum `OBIntraCameraSyncReference`): `OB_START_OF_EXPOSURE` = 0, `OB_MIDDLE_OF_EXPOSURE` = 1, `OB_END_OF_EXPOSURE` = 2. For multi-device sync, `OB_START_OF_EXPOSURE` is recommended.

## Project Structure

```text
Multi-Device-Synchronization-Example/
|-- CMakeLists.txt                              # Build configuration
|-- MultiDeviceSync/                            # Standard USB/Ethernet multi-device sync
|   `-- MultiDeviceSync.cpp                     #   Main program
|-- MultiDeviceSyncGmslTrigger/                 # GMSL PWM hardware trigger
|   `-- MultiDeviceSyncGmslTrigger.cpp          #   Main program
|-- common/                                     # Shared core modules
|   |-- PipelineHolder.hpp/cpp                  #   Per-device pipeline wrapper
|   |-- FramePairingManager.hpp/cpp             #   Per-device per-sensor timestamp CSV logging
|   `-- utils/                                  #   Utility library
|       |-- cJSON.c/h                           #     JSON parser
|       |-- utils.cpp/hpp                       #     Basic utility functions
|       |-- utils_c.c/h                         #     C utility functions
|       |-- utils_opencv.cpp/hpp                #     OpenCV visualization windows
|       `-- utils_types.h                       #     Common type definitions
|-- scripts/                                    # Sync accuracy analyzer
|   `-- analyze_sync.py                         #   Standalone, no third-party dependencies
|-- config/
|   |-- OrbbecSDKConfig.xml                     # SDK device configuration
|   `-- MultiDeviceSyncConfig.json              # Current sync configuration
|-- docs/                                       # Per-series documentation
|   |-- Astra2.md
|   |-- FemtoSeries.md
|   |-- Gemini2Series.md
|   |-- Gemini301Series.md
|   |-- Gemini330Series.md
|   `-- Gemini435Le.md
|-- res/                                        # Documentation image resources
`-- 3rdparty/orbbecsdk/                         # OrbbecSDK
    |-- win_x64/
    |-- linux_x86_64/
    `-- linux_arm64/
```
