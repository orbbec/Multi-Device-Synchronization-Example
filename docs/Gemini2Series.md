# Gemini 2 Series

## 1. Supported Devices

| Product  |  Support sync mode |  Notes          |
| -------- |  ---- | ----- |
| Gemini 2 / Gemini 2L  |  Primary, Secondary-synced, Software Triggering, Hardware Triggering     |   The Sync mode settings are **retained** after power cycling                |
| Gemini 215 / Gemini 210  |  Primary, Secondary-synced    | The Sync mode settings are **not retained** after power cycling                 |



## 2. Hardware Connection


- USB devices must be connected to a sync hub (via the 8-pin port). Please refer to the [Multi-device Sync documentation](https://www.orbbec.com/docs-general/set-up-cameras-for-external-synchronization_v1-2/).


## 3 Sync Modes

### 3.1 Primary/Secondary-synced Mode

The standard hardware cable sync mode. One device acts as the Primary outputting sync trigger signals, while the other devices act as Secondary receiving trigger signals, achieving frame exposure synchronization across all devices.

**Use case:** Multi-device synchronized capture via USB connection, using the standard 8-pin sync cable + sync hub solution.

**How it works:**

1. The Primary device sends a hardware trigger signal via the 8-pin sync cable at each frame exposure.
2. The sync hub distributes the signal to secondary devices.
3. Secondary devices perform exposure capture upon receiving the trigger signal.

**Startup sequence:**

1. Start all Secondary devices first (wait for initialization to complete).
2. Start the Primary device last.

**Configuration example (Primary):**

```json
{
    "sn": "CP2194200060",
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
```

**Configuration example (Secondary-synced):**

```json
{
    "sn": "CP0Y8420004K",
    "syncConfig": {
        "syncMode": "OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED",
        "depthDelayUs": 0,
        "colorDelayUs": 0,
        "trigger2ImageDelayUs": 0,
        "triggerOutEnable": true,
        "triggerOutDelayUs": 0,
        "framesPerTrigger": 1
    }
}
```


### 3.2 SOFTWARE_TRIGGERING Mode

PC-side software triggering mode. After all devices are configured in SOFTWARE_TRIGGERING mode, the PC sends a software command to trigger all devices to simultaneously capture one frame.

**Use case:** Scenarios without hardware sync cables, or where precise control of capture timing is needed (e.g., static object photography, quality inspection, etc.).

**How it works:**

1. All devices are configured in SOFTWARE_TRIGGERING mode
2. Devices enter a standby state, waiting for the PC to issue a trigger command
3. The user presses the `T` key in the preview window; the program calls `device->triggerCapture()` to trigger capture
4. All devices expose and capture simultaneously upon receiving the trigger command

**Trigger frequency limit:**

Trigger frequency x `framesPerTrigger` must be less than the stream frame rate. For example, with a 30fps stream:

- `framesPerTrigger = 1`: manual trigger interval must be >= 33ms (approximately 30 triggers per second maximum)
- `framesPerTrigger = 5`: manual trigger interval must be >= 167ms (approximately 6 triggers per second maximum)

If triggering is too frequent, the device will not be able to process in time, which may result in frame drops or trigger failures.

**Configuration example:**

```json
{
    "sn": "CP2194200060",
    "syncConfig": {
        "syncMode": "OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING",
        "depthDelayUs": 0,
        "colorDelayUs": 0,
        "trigger2ImageDelayUs": 0,
        "triggerOutEnable": true,
        "triggerOutDelayUs": 0,
        "framesPerTrigger": 1
    }
}
```


### 3.3 HARDWARE_TRIGGERING Mode

Hardware trigger mode. The device captures a specified number of frames upon receiving an external hardware trigger signal via the sync port.


**Configuration example:**

```json
{
    "sn": "CP2194200060",
    "syncConfig": {
        "syncMode": "OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING",
        "depthDelayUs": 0,
        "colorDelayUs": 0,
        "trigger2ImageDelayUs": 0,
        "triggerOutEnable": true,
        "triggerOutDelayUs": 0,
        "framesPerTrigger": 1
    }
}
```

### 3.4 Software/Hardware Triggering 

In this mode, the primary device is configured in `OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING` and the secondary device(s) in `OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING`. The PC triggers the primary device via software command; the primary device then outputs a hardware trigger signal via the sync port to trigger the secondary device(s).

**Configuration example (Primary - Software Triggering):**

```json
{
    "sn": "CP2194200060",
    "syncConfig": {
        "syncMode": "OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING",
        "depthDelayUs": 0,
        "colorDelayUs": 0,
        "trigger2ImageDelayUs": 0,
        "triggerOutEnable": true,
        "triggerOutDelayUs": 0,
        "framesPerTrigger": 1
    }
}
```

**Configuration example (Secondary - Hardware Triggering):**

```json
{
    "sn": "CP0Y8420004K",
    "syncConfig": {
        "syncMode": "OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING",
        "depthDelayUs": 0,
        "colorDelayUs": 0,
        "trigger2ImageDelayUs": 0,
        "triggerOutEnable": false,
        "triggerOutDelayUs": 0,
        "framesPerTrigger": 1
    }
}
```

### 3.5 syncConfig Field Reference

| Field  | Description |
| ---- | ---- |
| `syncMode` | Sync mode. Available values: `OB_MULTI_DEVICE_SYNC_MODE_PRIMARY`, `OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED`, `OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING`, `OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING` |
| `depthDelayUs` | Configure depth trigger delay, unit: microseconds |
| `colorDelayUs` |  Color trigger signal input delay in microseconds. Typically set to 0 |
| `trigger2ImageDelayUs` |  Delay from trigger signal input to image capture in microseconds. Typically set to 0 |
| `triggerOutEnable` |  Device trigger signal output enable switch |
| `triggerOutDelayUs` | Device trigger signal output delay in microseconds. Typically set to 0 |
| `framesPerTrigger` | Number of frames captured per trigger. Only effective in SOFTWARE_TRIGGERING and HARDWARE_TRIGGERING modes, typically set to 1 |

### streamConfig Field Reference

Optional stream profile overrides for stream start. `depth`/`color` each accept `width`, `height`, `fps`, `format`.

| Field | Description |
| ---- | ---- |
| `streamConfig.depth.width` | Depth stream resolution width. 0 = use the SDK default profile |
| `streamConfig.depth.height` | Depth stream resolution height. 0 = use the SDK default profile |
| `streamConfig.depth.fps` | Depth stream frame rate. 0 = use the SDK default profile |
| `streamConfig.depth.format` | Depth stream format, e.g. `OB_FORMAT_Y16`. Empty = use the SDK default profile |
| `streamConfig.color.width` | Color stream resolution width. 0 = use the SDK default profile |
| `streamConfig.color.height` | Color stream resolution height. 0 = use the SDK default profile |
| `streamConfig.color.fps` | Color stream frame rate. 0 = use the SDK default profile |
| `streamConfig.color.format` | Color stream format, e.g. `OB_FORMAT_MJPG`. Empty = use the SDK default profile |


## 4 Operation Guide

### 4.1 Build the Project

Open a terminal and navigate to the project directory:

```bash
cd Multi-Device-Synchronization-Example
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

### 4.2 Modify the Configuration File


Edit the configuration file to match your actual device serial numbers. Device serial numbers can be viewed using the OrbbecViewer tool or found in the program log after connecting the devices.


### 4.3 Run Sample

#### Windows
The following demonstrates how to use the multi-device synchronization sample on Windows.
- Double-click MultiDeviceSync.exe. When the following dialog appears, select 0.

![windows_sync](../res/image/windows_sync.png)


0: Configure sync mode and start stream

1: Start stream: If the parameters for multi-device sync mode have been configured, you can start the stream directly.


- Multi-device synchronization test results are as follows
![windows sync result](../res/image/windows_sync_result.png)


Observe the timestamps. As shown in the figure below, the device timestamps of the two devices are identical, indicating that the two devices are successfully synchronized.

#### Linux/ARM64

- For USB device or Ethernet device multi-device synchronization, simply execute MultiDeviceSync. 
```
$ ./MultiDeviceSync

```

## Caution
- After starting the device, press 'ESC' in the image preview window to stop the data stream and exit the program. Abnormal program termination may cause incomplete shutdown of the device, leading to continuous triggering of the secondary device (restarting the device can resolve this).

- The same device can only be accessed by one application at a time. Opening the same device with multiple applications simultaneously may cause anomalies. Please use with caution.

- FPS boost: in trigger mode, the frame rate can be increased by enabling the SDK property `OB_PROP_FPS_BOOST_BOOL`. Notes: USB 2.0 connections provide no frame rate improvement; Ethernet connections are not supported; all other connections support frame rates of 30 fps and below. Support depends on the device firmware; verify at runtime with `device->isPropertySupported(OB_PROP_FPS_BOOST_BOOL, OB_PERMISSION_READ_WRITE)` before enabling it.

- Using AE (Auto Exposure) may result in synchronization delays due to significant environmental differences between cameras. It is recommended to use the SDK to call exposure control interfaces and set fixed exposures to mitigate this issue.

- For Linux computers, such as Ubuntu, the default kernel allocates only 16 MB of memory for USB controllers to handle USB transfers. This amount may be insufficient for high-resolution images or multiple streams and devices. To support multiple devices, the USB controller must have more memory allocated. Follow these steps to increase the allocated memory:

```
echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```
For Linux-x86 platforms, to make this change permanent:
Open the `/etc/default/grub` file, find and replace:
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
```
with this
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=128"
```

Update grub
```
sudo update-grub
```
Reboot and check
```
cat /sys/module/usbcore/parameters/usbfs_memory_mb
```

