
#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <signal.h>
#include<std_msgs/String.h>

#include<zvision_lidar_msgs/zvisionLidarInformation.h>

void mtr_sub_cb(const zvision_lidar_msgs::zvisionLidarInformation::ConstPtr& msg){
    zvision_lidar_msgs::zvisionLidarInformation info = *msg;
    ROS_INFO_STREAM("is_ptp:"<< (int)info.is_ptp);
    ROS_INFO_STREAM("lock_status:"<< (int)info.lock_status);
    ROS_INFO_STREAM("stamp(s):"<< info.stamp);
    ROS_INFO_STREAM("apd_bias(v):"<< info.apd_bias<< "\n");
}

void monitor_subscriber(int argc, char** argv){

    ros::init(argc, argv, "frame_info_node_sub");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
    ros::Subscriber mtr_sub = node.subscribe("zvision_lidar_frame_info", 20, &mtr_sub_cb);

    ros::Rate rate(40.0);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}

volatile sig_atomic_t flag = 1;
static void my_handler(int sig)
{
 flag = 0;
 ros::shutdown();
}

int main(int argc, char** argv){

    monitor_subscriber(argc, argv);
    return 0;
}