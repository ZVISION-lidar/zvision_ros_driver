#include<ros/ros.h>
#include<std_msgs/String.h>
#include <ros/package.h>
#include <signal.h>

#include "monitor.h"

void mtr_sub_cb(const zvision_lidar_msgs::zvisionLidarHeartbeat::ConstPtr& msg){
    zvision_lidar_msgs::zvisionLidarHeartbeat ht = *msg;
    ROS_INFO_STREAM("\n"<<ht.info.c_str() << "\n");
}

void monitor_subscriber(int argc, char** argv){

    ros::init(argc, argv, "monitor_node_sub");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
    ros::Subscriber mtr_sub = node.subscribe("zvision_lidar_heartbeat", 20, &mtr_sub_cb);

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
}
void monitor_publisher(int argc, char** argv){

    ros::init(argc, argv, "monitor_node_pub");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
    signal(SIGINT, my_handler);

    zvision_lidar_driver::zvisionLidarMonitor mtr(node, private_nh);
    while(ros::ok()){
        mtr.pollMsg();
        ros::spinOnce();
    }
}


int main(int argc, char** argv){

    // monitor_publisher(argc, argv);

    monitor_subscriber(argc, argv);
    return 0;
}