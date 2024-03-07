/*
***********************************************************
* ######  #    #     #     ####      #     ####   #    #
*     #   #    #     #    #          #    #    #  ##   #
*    #    #    #     #     ####      #    #    #  # #  #
*   #     #    #     #         #     #    #    #  #  # #
*  #       #  #      #    #    #     #    #    #  #   ##
* ######    ##       #     ####      #     ####   #    #
***********************************************************
*
* Software License Agreement (Private License)
*
*  ZISION Tech - www.zvision.xyz.
*  Copyright(c) 2018-2022 ZVISION.Co.Ltd
*  All rights reserved.
*/

#include <iostream>
#include <string.h>
#include <signal.h>
#include <condition_variable>
#include <util/env_def.h>

#include <manager/processor_manager.h>
#include <common/print.h>

std::mutex g_mutex;
std::condition_variable g_cdv;
const std::string g_node_str = "zvlidar_driver_node";
static void my_handler(int sig)
{
    LOG_INFO("%s: [Stopping].\n", g_node_str.c_str());
#ifdef ROS_FOUND
    ros::shutdown();
#endif

#ifdef ROS2_FOUND
    rclcpp::shutdown();
#endif

    g_cdv.notify_all();
}

int main(int argc, char** argv)
{
#ifdef ROS_FOUND
    ros::init(argc, argv, g_node_str.c_str(), ros::init_options::NoSigintHandler);
#endif

#ifdef ROS2_FOUND
  rclcpp::init(argc, argv);
#endif
    
    LOG_INFO("%s: [Running].\n%s\n",g_node_str.c_str(),  zv_processor::get_zv_logo().c_str());
    // exit cb
    signal(SIGINT, my_handler);

    // start the driver
    std::shared_ptr<zv_processor::ProcessorManager> g_pm = nullptr;
#ifdef ENV_ROS
    std::shared_ptr<NodeHandle_> node = std::make_shared<NodeHandle_>("zvlidar_sdk");
    g_pm = std::make_shared<zv_processor::ProcessorManager>(node);
#else
    g_pm = std::make_shared<zv_processor::ProcessorManager>();
#endif
    // default yaml path
    std::string yaml_path = std::string(PROJECT_PATH) + "/config/config.yaml";
#ifdef ENV_ROS
    {
        std::string usr_path;
        #ifdef ROS_FOUND
            NodeHandle_ private_nh("~");
            private_nh.param("yaml_path", usr_path, std::string(""));
        #else
            node->declare_parameter("yaml_path","");
            usr_path = node->get_parameter("yaml_path").as_string();
        #endif
        if(!usr_path.empty())
           yaml_path = usr_path;
        std::cout<< "from manu :" << usr_path.c_str() <<std::endl;
    }
#endif
    // initialization
    g_pm->init(yaml_path);
    // start processor manager
    g_pm->start();
    std::cout<<std::endl;
    // wait for exit
#ifdef ROS_FOUND
    #ifdef ROS_FOUND
        ros::spin();
    #endif

    #ifdef ROS2_FOUND
        rclcpp::spin();
    #endif
#else
    std::unique_lock<std::mutex> _lock(g_mutex);
    g_cdv.wait(_lock);
#endif

    if(g_pm)
    {
        g_pm->stop();
        g_pm.reset();
    }
    return 0;
}


