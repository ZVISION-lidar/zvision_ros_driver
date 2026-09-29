#!/bin/bash
# 判断目标文件夹是否存在
if [ -d "zvlidar_sdk/src/lidar_sdk" ]; then
    echo "lidar_sdk folder exists, do not replace."
else
    # 检查是否有符合条件的 .zip 文件
    if find . -maxdepth 1 -name "zvision_sdk_v*.zip" | grep -q .; then
        echo "zvision_sdk_v*.zip exists. Unzip and clear."
        unzip zvision_sdk_v*.zip -d zvlidar_sdk/src/lidar_sdk
        rm -rf zvision_sdk_v*.zip zvision_ros_driver*.zip
    else
        echo "zvision_sdk_v*.zip does not exist. Cloning from Git repository."
        git clone http://git.z.lan:10008/embedded/lidar_sdk
        mv lidar_sdk zvlidar_sdk/src/
    fi
fi

