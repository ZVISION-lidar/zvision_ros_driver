#pragma once
#include <string>
#include <thread>
#include <common/define.h>
#include <point_cloud.h>

#include <util/env_def.h>
#include <util/config.hpp>
#include <plugin/filter_pointcloud.h>

#ifdef ROS2_FOUND
    #include<zvlidar_msgs/msg/zvision_lidar_scan.hpp>
#elif defined ROS_FOUND
    #include<zvlidar_msgs/ZvisionLidarScan.h>
#endif

namespace zv_processor
{
    /* Lidar device Processor */
    class Processor
    {
    public:
        Processor(/* args */);
        ~Processor();
#ifdef ENV_ROS
        void init(std::shared_ptr<NodeHandle_> node, const ProcessorConfig& cfg);
#else
        void init(const ProcessorConfig& cfg);
#endif


        void start();

        void stop();

    private:
        /*  pointcloud*/
        void Consumer();
        /* pro */
       void process(const zvision::PointCloud& pointcloud);
    private:

        /* **filter ***/
        FilterPointcloud pointcloud_filter_;

        /*** player ***/
        zvision::CalibrationDataSinCosTable cal_lut_;
        std::shared_ptr<zvision::PointCloudProducer> online_player_;
        std::shared_ptr<zvision::OfflinePointCloudProducer> offline_player_;
        int offline_frames_;
        std::shared_ptr<std::thread> consumer_thre_;
        
        /*** processor***/
        /* config parameter */
        ProcessorConfig config_;
        zvision::DeviceType device_type_;
        /* processor state */
        bool is_running_;

        /*** ros message ***/
#ifdef ROS_FOUND
        /* lidar packets */
        std::shared_ptr<ros::Publisher> pub_packets_;
        /* pointcloud type xyzrgba */
        std::shared_ptr<ros::Publisher> pub_pointcloud_xyzrgba_;
        /* pointcloud type xyzi */
        std::shared_ptr<ros::Publisher> pub_pointcloud_xyzi_;
        /* pointcloud type xyzirt */
        std::shared_ptr<ros::Publisher> pub_pointcloud_xyzirt_;
        /* pointcloud type blooming */
        std::shared_ptr<ros::Publisher> pub_pointcloud_blooming_;
#endif

#ifdef ROS2_FOUND
        /* lidar packets */
        rclcpp::Publisher<zvlidar_msgs_::ZvisionLidarScan>::SharedPtr pub_packets_;
        /* pointcloud type xyzrgba */
        rclcpp::Publisher<PointCloud2_>::SharedPtr pub_pointcloud_xyzrgba_;
        /* pointcloud type xyzi */
        rclcpp::Publisher<PointCloud2_>::SharedPtr pub_pointcloud_xyzi_;
        /* pointcloud type xyzirt */
        rclcpp::Publisher<PointCloud2_>::SharedPtr pub_pointcloud_xyzirt_;
        /* pointcloud type blooming */
        rclcpp::Publisher<PointCloud2_>::SharedPtr pub_pointcloud_blooming_;
#endif
    };



} // namespace zvision