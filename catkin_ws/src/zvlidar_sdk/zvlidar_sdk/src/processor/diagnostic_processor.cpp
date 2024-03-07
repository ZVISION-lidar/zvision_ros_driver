#include "processor/diagnostic_processor.h"
#include <chrono>
#include <functional>
#include <commu/client.h>
#include <common/print.h>
#include <util/sync.hpp>
#include <util/translater.h>

namespace zv_processor{

    std::unique_ptr<DiagnosticProcessor> DiagnosticProcessor::instance_ = nullptr;
    DiagnosticProcessor::DiagnosticProcessor(/* args */)
    :is_running_(false)
    {
        init();
    }

    DiagnosticProcessor::~DiagnosticProcessor()
    {
#if defined(ROS2_FOUND) || defined(ROS_FOUND)
        if(node_ptr_)
            node_ptr_.reset();
#endif
    }

    DiagnosticProcessor* DiagnosticProcessor::GetInstance()
    {
        if(!DiagnosticProcessor::instance_)
            DiagnosticProcessor::instance_.reset( new DiagnosticProcessor);
        return instance_.get();
    }

    void DiagnosticProcessor::init()
    {
        if(!packets_)
            packets_.reset( new SynchronizedQueue<DiagnosticInfo> );

#ifdef ROS_FOUND
        if(!node_ptr_)
            node_ptr_.reset(new ros::NodeHandle());
            //node_ptr_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
#endif
    
#ifdef ROS2_FOUND
        if(!node_ptr_)
            node_ptr_.reset(new rclcpp::Node("zvlidar_diagnostic_node"));
#endif
        device_map_.clear();
    }

    bool DiagnosticProcessor::addDevice(zvision::DeviceType type, std::string ip)
    {
        /* check pub */
        if(type == zvision::DeviceType::LidarML30SA1)
        {
#ifdef ROS_FOUND
            if(!pub_diag_30s_)
                pub_diag_30s_ = std::make_shared<ros::Publisher>(node_ptr_->advertise<zvlidar_msgs::ML30SHeartbeatMsg>("zvlidar_diagnostic_30s", 20));
#endif
#ifdef ROS2_FOUND
            if(!pub_diag_30s_)
                pub_diag_30s_ = node_ptr_->create_publisher<zvlidar_msgs_::ML30SHeartbeatMsg>("zvlidar_diagnostic_30s", 20);
#endif
        }
        else if(type == zvision::DeviceType::LidarMl30SA1Plus)
        {
#ifdef ROS_FOUND
            if(!pub_diag_30s_plus_)
                pub_diag_30s_plus_ = std::make_shared<ros::Publisher>(node_ptr_->advertise<zvlidar_msgs::ML30SPlusHeartbeatMsg>("zvlidar_diagnostic_30s_plus", 20));
#endif
#ifdef ROS2_FOUND
            if(!pub_diag_30s_plus_)
                pub_diag_30s_plus_ = node_ptr_->create_publisher<zvlidar_msgs_::ML30SPlusHeartbeatMsg>("zvlidar_diagnostic_30s_plus", 20);
#endif
        }
        else if(type == zvision::DeviceType::LidarMl30SA1Factory)
        {
#ifdef ROS_FOUND
            if(!pub_diag_30s_factory_)
                pub_diag_30s_factory_ = std::make_shared<ros::Publisher>(node_ptr_->advertise<zvlidar_msgs::ML30SFactoryHeartbeatMsg>("zvlidar_diagnostic_30s_factory", 20));
#endif
#ifdef ROS2_FOUND
            if(!pub_diag_30s_factory_)
                pub_diag_30s_factory_ = node_ptr_->create_publisher<zvlidar_msgs_::ML30SFactoryHeartbeatMsg>("zvlidar_diagnostic_30s_factory", 20);
#endif
        }
        else if(type == zvision::DeviceType::LidarMLX)
        {
#ifdef ROS_FOUND
        //    if(!pub_diag_mlxs_)
        //        pub_diag_mlxs_ = std::make_shared<ros::Publisher>(node_ptr_->advertise<zvlidar_msgs::MLXSHeartbeatMsg>("zvlidar_diagnostic_mlxs", 20));
#endif
#ifdef ROS2_FOUND
        //    if(!pub_diag_mlxs_)
        //        pub_diag_mlxs_ = node_ptr_->create_publisher<zvlidar_msgs_::MLXSHeartbeatMsg>("zvlidar_diagnostic_mlxs", 20);
#endif
        }

        // update filter map
        device_map_[ip].insert(type);
    }

    void DiagnosticProcessor::start()
    {
        if(is_running_)
            return;

        /* check config */
        if(!input_thre_)
            input_thre_.reset(new std::thread(std::bind(&DiagnosticProcessor::input, this)));
        if(!input_thre_)
            process_thre_.reset(new std::thread(std::bind(&DiagnosticProcessor::process, this)));

        is_running_ = true;
    }

    void DiagnosticProcessor::stop()
    {
        if(!is_running_)
            return;
        is_running_ = false;

        /* input */
        if(input_thre_)
        {
            if(input_thre_->joinable())
            {
                input_thre_->join();
            }
            input_thre_.reset();
        }
        /* process */
        if(process_thre_)
        {
            if(process_thre_->joinable())
            {
                process_thre_->join();
            }
            process_thre_.reset();
        }

        /* claear data */
        if(packets_)
            packets_->stopQueue();

        device_map_.clear();
    }

    void DiagnosticProcessor::input()
    {
        const int heart_beat_port = 55000;
        zvision::UdpReceiver recv(heart_beat_port, 100);
        std::string data;
        const int heart_beat_len = 48;

        int len = 0;
        uint32_t ip = 0;
        int ret = 0;
        std::string str_30s_mlxs = "ZVSHEARTBEAT";
        std::string str_30s_fac = "ZVSHTFACTORY";
        std::string str_30s_plus = "zvision";
        std::string str_exciton = "zvision";
        LOG_DEBUG("### Start monitor heartbeat on port %d. ###\n", heart_beat_port);
        while (is_running_)
        {
            if (0 != (ret = recv.SyncRecv(data, len, ip)))
            {
                LOG_WARN("### Get heartbeat packet faialed. ###\n");
                continue;
            }

            /* check */
            if(len < heart_beat_len)
                continue;

            /* get lidar type */
            zvision::DeviceType type = zvision::DeviceType::LidarUnknown;
            if((len == 138) && (0 == data.compare(0, str_30s_mlxs.size(), data.substr(0, str_30s_mlxs.size()))))
            {
                type = zvision::DeviceType::LidarML30SA1;
            }
            else if((len == 512) && (0 == data.compare(0, str_30s_mlxs.size(), data.substr(0, str_30s_mlxs.size()))))
            {
                type = zvision::DeviceType::LidarMLX;
            }
            else if((len == 512) && (0 == data.compare(0, str_30s_plus.size(), data.substr(0, str_30s_plus.size()))))
            {
                type = zvision::DeviceType::LidarMl30SA1Plus;
            }
            else if((len == 512) && (0 == data.compare(0, str_30s_fac.size(), data.substr(0, str_30s_fac.size()))))
            {
                type = zvision::DeviceType::LidarMl30SA1Factory;
            }
            else if((len == 164) && (0 == data.compare(0, str_exciton.size(), data.substr(0, str_exciton.size()))))
            {
                type = zvision::DeviceType::LidarExciton;
            }
			if (type != zvision::DeviceType::LidarUnknown)
			{
                std::string ip_string = zvision::IpToString(ip);
                DiagnosticInfo info;
                info.dev_type = type;
                info.ip_str = ip_string;
                info.packet = data;
                info.stamp_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>
                    (std::chrono::system_clock::now()).time_since_epoch().count();

                if(packets_)
                    packets_->enqueue(info);
            }
        }
    }

    void DiagnosticProcessor::process()
    {
        while(is_running_){
            if(packets_)
            {
                DiagnosticInfo diag;
                packets_->dequeue(diag);
                processPacket(diag);
            }
        }
    }

    void DiagnosticProcessor::processPacket(const DiagnosticInfo& diag)
        {
           
           /* filter device */
            if(device_map_.find(diag.ip_str) == device_map_.end() || \
                device_map_[diag.ip_str].find(diag.dev_type) == device_map_[diag.ip_str].end())
            {
                return;
            }

            /* parsing heartbeat packet */
            DiagnosticInfo info = diag;
            if(!parsingDiagnosticInfo(info))
            {
                LOG_WARN("### Unknown heartbeat packet type, ignore ...\n ");
                return;
            }

            /* to message */
#ifdef ENV_ROS
            if(diag.dev_type == zvision::DeviceType::LidarML30SA1)
            {
                zvlidar_msgs_::ML30SHeartbeatMsg msg = convertDiagToML30SMsg(diag);
                pub_diag_30s_->publish(msg);
            }
            else if(diag.dev_type == zvision::DeviceType::LidarMl30SA1Plus)
            {
                zvlidar_msgs_::ML30SPlusHeartbeatMsg msg = convertDiagToML30SPlusMsg(diag);
                pub_diag_30s_plus_->publish(msg);
            }
            else if(diag.dev_type == zvision::DeviceType::LidarMl30SA1Factory)
            {
                zvlidar_msgs_::ML30SFactoryHeartbeatMsg msg = convertDiagToML30SFactoryMsg(diag);
                pub_diag_30s_factory_->publish(msg);
            }
            else if(diag.dev_type == zvision::DeviceType::LidarMLX)
            {
                //zvlidar_msgs_::MLXSHeartbeatMsg msg = convertDiagToMLXSMsg(diag);
                //pub_diag_mlxs_->publish(msg);
            }
#endif
        }
}