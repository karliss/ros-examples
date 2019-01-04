# PX4 Snapdragon Machine Vision module
This repository contains px4 module which uses [Snapdragon Machine Vision SDK](https://developer.qualcomm.com/software/machine-vision-sdk) for VISLAM. It is based on [PX4/ros-examples](https://github.com/PX4/ros-examples) which is a fork of [ATLFlight/ros-examples](https://github.com/ATLFlight/ros-examples) with the required changes to work as px4 module. Compared to ros-examples it reads IMU data and publishes result directly to PX4 instead of going through mavlink,mavros and ros.


## High-Level Block Diagram
![SnapVislamRosNodeBlockDiagram](images/SnapVislamRosNodeBlockDiagram.jpg)

## Setup and build process

**NOTE:** These instructions are for VISLAM version 1.1.9. Snapdragon MV API has changed between 0.9.1 and 1.1.9. Using with  

### Summary of changes from previous release

| Item | Previous release - mv0.8 | Previous Release - mv0.9.1 | Current Release mv 1.0.2, 1.1.9 |
|----|----|----|----|
|MV_SDK environment variable| needed | Not needed.  The new mv installation puts the library files under /usr/lib | Not needed |
|MV License file installation | needed.  Should be placed in the /opt/qualcomm/mv/lib/mv/bin/lin/8x74/ | needed should be placed at /usr/lib | needed should be placed at /opt/qcom-licenses/ |
|MV link library Name| libmv.so | libmv1.so.  Update the respective make files to link against libmv1 instead of libmv.so | same as mv 0.9.1 |

### Pre-requisites

#### Platform BSP

These instructions were tested with version **Flight_3.1.3.1**. The latest version of the software can be downloaded from [here](http://support.intrinsyc.com/projects/snapdragon-flight/files) and  installed by following the instructions found [here](http://support.intrinsyc.com/projects/snapdragon-flight/wiki)


#### Install Snapdragon Machine Vision SDK

* Download the latest Snapdragon Machine Vision SDK from [here](https://developer.qualcomm.com/software/machine-vision-sdk)
* The package name will be mv\<version\>.deb.  
** Example: *mv1.1.9_8x74.deb*
* push the deb package to the target and install it.

```
adb push mv<version>.deb /home/linaro
adb shell 
dpkg -i /home/linaro/mv<version>.deb
```

* Unpack the .deb file to obtain headers and lib for cross-compilation. Set `MV_SDK` environment variable to the folder containing `usr` folder.

#### Machine Vision SDK License Installation

The Machine Vision SDK will need a license file to run.  Obtain a research and development license file from [here](https://developer.qualcomm.com/sdflight-key-req)

The license file needs to be placed in the following folder on target: /opt/qcom-licenses/

Push the license file to the target using the following command:

```
adb push snapdragon-flight-license.bin /opt/qcom-licenses/
adb shell sync
```

*NOTE:* Make sure to create the folder /opt/qcom-licenses/ if it is not present on the target.

### Clone and build sample code

* Clone this repository in one of the PX4/Firmware module folders - modules or examples.
* Add px4_snapdragon_mv to px4 module list in the cmake file for app processor boards/atlflight/eagle/default.cmake
* Add snap_vislam_status.msg to the uorb message list.
* Build and install as usual `make eagle_default upload`. See PX4 devguide snapdragon instructions for more information. 
  
*untested* Alternatively follow out of tree module [instructions](https://dev.px4.io/en/advanced/out_of_tree_modules.html).

## Configure
* Configure EKF2 to use visual position and height
* EKF2_AID_MASK 24
* EKF2_HGT_MODE 3
* EKF2_EV_DELAY ~5-30 (default is 175ms) adjust by comparing local position estimate and vision position/ODOMETRY
* MAV_ODOM_LP 1 - send vislam position over MAVlink as ODOMETRY, useful for debuging 

## Run
Start px4 module using `snapdragon_mv start` in PX4 shell or main configuration to start automatically.
