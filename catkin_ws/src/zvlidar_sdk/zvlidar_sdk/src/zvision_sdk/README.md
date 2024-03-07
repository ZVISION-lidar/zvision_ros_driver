# 1 Introduction

ZVISION SDK developed based on c++ provide a convenient way to use the zvision production. You can communicate with the device and get the device's info, including but not limited to pointcloud and configuration.

## Prerequisites
* Ubuntu Ubuntu 16.04/Ubuntu 18.04/Ubuntu 20.04/Ubuntu 22.04, both x86 and ARM (Nvidia TX2, Xavier, Orin, ect.)
* Windows 7/10/11, Visual Studio 2015 Update3
* C++11 compiler

# 2 ZVISION SDK

ZVISION SDK implement the main function to configure device's function.

# 3 SAMPLE

For user's reference, SAMPLE tell you how to set device's parameter, how to get pointcloud data from the device.

# 4 INSTALL SDK

## 4.1 Installation
### 4.1.1 Ubuntu LTS
#### Dependencies
[CMake 3.0.0+](https://cmake.org/) is required. You can install these packages using apt:
```
sudo apt install cmake pkg-config
```
#### Compile ZVISION SDK
In the ZVISION SDK directory, run the following commands to compile the project:
```
git clone https://github.com/zvision-lidar/zvision_sdk.git
cd zvision_sdk
```
```
mkdir build
cd build && cmake ..
make
sudo make install
```

### 4.1.2 Windows

#### Dependencies
ZVISION SDK supports Visual Studio 2015 and requires install [CMake 3.0.0+](https://cmake.org/) as dependencies.  

In the ZVISION SDK directory, run the following commands to create the Visual Studio solution file. 
Generate the 64-bit project:
```
cd zvision_sdk
cd build && \
cmake .. -G "Visual Studio 14 2015 Win64"
```
#### Compile ZVISION SDK
You can now compile the SDK in Visual Studio.

## 4.2 Run Sample
Two samples are provided in sample/lidar_config and Sample/pointcloud

### 4.2.1 Ubuntu LTS
For Ubuntun 18.04/16.04/14.04 LTS, run the *pointcloud_sample* if connect with the LiDAR
```
cd sample/pointcloud && ./pointcloud_sample
```
or run the *lidarconfig_sample* if connect with the LiDAR:
```
cd sample/lidar_config && ./lidarconfig_sample
```
### 4.2.2 Windows
After compiling the ZVISION SDK, you can find `pointcloud_sample.exe` or `lidarconfig_sample.exe` in the {zvision_sdk}\build\sample\pointcloud\Debug or {zvision_sdk-SDK}\build\sample\lidar_config\Debug folder, respectively, which can be run directly. 

