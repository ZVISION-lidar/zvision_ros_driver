#include<ros/ros.h>
#include<std_msgs/String.h>
#include <ros/package.h>
#include <signal.h>

#include "monitor.h"
volatile sig_atomic_t flag = 1;
static void my_handler(int sig)
{
 flag = 0;
 ros::shutdown();
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

    monitor_publisher(argc, argv);
    return 0;
}