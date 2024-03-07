/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2019, Zvision, Pengfei Cui
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver nodelet for the ZVISION LIDAR 3D LIDARs
 */

#include <signal.h>
#include <mutex>
#include <condition_variable>
#include <string>
#include <iostream>
#include <thread>
#include <signal.h>
#include <condition_variable>
#include <boost/thread.hpp>
#include <util/env_def.h>
#include <manager/processor_manager.h>

#ifdef ROS2_FOUND
    #include "rclcpp_components/register_node_macro.hpp"
#endif

std::condition_variable g_cdv;
static void my_handler(int sig)
{
#ifdef ROS_FOUND
    ros::shutdown();
#endif

#ifdef ROS2_FOUND
    rclcpp::shutdown();
#endif
    g_cdv.notify_all();
}

namespace zvlidar_sdk
{
    class ZVLidarNodelet
#ifdef ROS2_FOUND
    : public rclcpp::Node
#endif
    {
    public:   
#ifdef ROS_FOUND
        explicit ZVLidarNodelet(ros::NodeHandle& node, ros::NodeHandle& private_nh)
        : running_(false)
        , node_(node)
        , private_nh_(private_nh)
        {

        }
#elif defined ROS2_FOUND
        explicit ZVLidarNodelet(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
        : running_(false)
        , Node("zvlidar_sdk", options)
        {
            node_ = rclcpp::Node::SharedPtr(this);
            onInit();
        }
#endif
        ~ZVLidarNodelet()
        {   
            printf("shutting down driver thread");
            if (running_)
            {
                if(manager_)
                {
                    manager_->stop();
                    manager_.reset();
                }
                running_ = false;
                if(poll_thread_->joinable())
                    poll_thread_->join();
            }
            printf("driver thread stopped");
        }

    private:
        virtual void onInit(void);
        virtual void devicePoll(void);
        volatile bool running_;
        std::shared_ptr<std::thread> poll_thread_= nullptr;
        std::shared_ptr<zv_processor::ProcessorManager> manager_ = nullptr;

#ifdef ROS_FOUND
        ros::NodeHandle node_;
        ros::NodeHandle private_nh_;
#elif defined ROS2_FOUND
        rclcpp::Node::SharedPtr node_;
#endif

    };

    void ZVLidarNodelet::onInit()
    {
        signal(SIGINT, my_handler);

        running_ = true;
#ifdef ENV_ROS
        manager_ = std::make_shared<zv_processor::ProcessorManager>(node_);
#else
        manager_ = std::make_shared<zv_processor::ProcessorManager>();
#endif
        std::cout<<"2"<<std::endl;
        // default yaml path
        std::string yaml_path = std::string(PROJECT_PATH) + "/config/config.yaml";
#ifdef ENV_ROS
        {
            std::string usr_path;
            #ifdef ROS_FOUND
                node_.param("yaml_path", usr_path, std::string(""));
            #else
                node_->declare_parameter("yaml_path","");
                usr_path = node_->get_parameter("yaml_path").as_string();
            #endif
            if(!usr_path.empty())
            yaml_path = usr_path;
            
            std::cout<< "from manu :" << usr_path.c_str() <<std::endl;
        }
#endif
        std::cout<<"3"<<std::endl;
        // initialization
        manager_->init(yaml_path);
        std::cout<<"4"<<std::endl;
        // start
        manager_->start();
        // thread
        //poll_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&ZVLidarNodelet::devicePoll, this)));
    }

    void ZVLidarNodelet::devicePoll()
    {
        std::cout<<"5"<<std::endl;
#ifdef ROS_FOUND
        while (ros::ok() && running_)
        {
            ros::spinOnce();
        }
#else
        //while (rclcpp::ok() && running_)
        //{
        //    std::cout<<"5.1"<<std::endl;
        //    rclcpp::spin_some(node_);
        //    std::cout<<"6"<<std::endl;
        //}
        //rclcpp::spin();
#endif
        // if(manager_)
        // {
        //     manager_->stop();
        //     manager_.reset();
        // }
    }



//#endif
}

// Register this plugin with pluginlib.  Names must match nodelet.xml.
// parameters are: class type, base class type
#ifdef ROS_FOUND
    PLUGINLIB_EXPORT_CLASS(zvlidar_sdk::ZVLidarNodelet, nodelet::Nodelet)
#endif

#ifdef ROS2_FOUND
    RCLCPP_COMPONENTS_REGISTER_NODE(zvlidar_sdk::ZVLidarNodelet)
#endif