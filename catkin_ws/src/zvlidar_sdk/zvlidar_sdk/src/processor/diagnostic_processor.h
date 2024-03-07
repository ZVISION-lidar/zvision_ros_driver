/*


*/
#pragma once
#include<string>
#include <queue>
#include <thread>
#include <time.h>
#include <mutex>
#include <unordered_map>
#include<set>

#include <common/define.h>
#include <util/env_def.h>
#include <util/config.hpp>

#ifdef ROS2_FOUND
    #include<zvlidar_msgs/msg/ml30_s_heartbeat_msg.hpp>
    #include<zvlidar_msgs/msg/ml30_s_factory_heartbeat_msg.hpp>
    #include<zvlidar_msgs/msg/ml30_s_plus_heartbeat_msg.hpp>

#elif defined ROS_FOUND
    #include<zvlidar_msgs/ML30SHeartbeatMsg.h>
    #include<zvlidar_msgs/ML30SFactoryHeartbeatMsg.h>
    #include<zvlidar_msgs/ML30SPlusHeartbeatMsg.h>
#endif

namespace zv_processor{

    template <typename T>
    class SynchronizedQueue;
    
    class DiagnosticProcessor
    {
    public:
         ~DiagnosticProcessor();
        /* add diagnostic type */
        bool addDevice(zvision::DeviceType type, std::string ip);
        /* start diagnostic monitor */
        void start();
        /* stop diagnostic monitor */
        void stop();
        /* get diagnostic  processor instance */
        static DiagnosticProcessor* GetInstance();
    private:
        DiagnosticProcessor(/* args */);


        /* init diagnostic monitor */
        void init();
        /* receive */
        void input();
        /* process */
        void process();
        /* process packet */
        void processPacket(const DiagnosticInfo& info);

    private:

        static std::unique_ptr<DiagnosticProcessor> instance_;
        bool is_running_;
        std::shared_ptr<std::thread> input_thre_;
        std::shared_ptr<std::thread> process_thre_;
        std::shared_ptr<SynchronizedQueue<DiagnosticInfo> > packets_;
        std::unordered_map<std::string, std::set<int>> device_map_;

        /* node , publisher*/
#ifdef ROS_FOUND
        std::shared_ptr<ros::NodeHandle> node_ptr_ = nullptr;
        std::shared_ptr<ros::Publisher> pub_diag_30s_;
        std::shared_ptr<ros::Publisher> pub_diag_30s_plus_;
        std::shared_ptr<ros::Publisher> pub_diag_30s_factory_;
        //std::shared_ptr<ros::Publisher> pub_diag_mlxs_;
#endif ROS_FOUND

#ifdef ROS2_FOUND
        std::shared_ptr<rclcpp::Node> node_ptr_ = nullptr;
        rclcpp::Publisher<zvlidar_msgs_::ML30SHeartbeatMsg>::SharedPtr pub_diag_30s_;
        rclcpp::Publisher<zvlidar_msgs_::ML30SPlusHeartbeatMsg>::SharedPtr pub_diag_30s_plus_;
        rclcpp::Publisher<zvlidar_msgs_::ML30SFactoryHeartbeatMsg>::SharedPtr pub_diag_30s_factory_;
        //rclcpp::Publisher<zvlidar_msgs_::MLXSHeartbeatMsg>::SharedPtr pub_diag_mlxs_;
#endif ROS_FOUND
    };

};