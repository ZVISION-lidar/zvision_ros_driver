#pragma once
#include <string>
#include <vector>

#include <point_cloud.h>
#include <util/env_def.h>
#include <util/config.hpp>
#ifdef ROS2_FOUND
    #include <zvlidar_msgs/msg/zvision_lidar_scan.hpp>
#elif defined ROS_FOUND
    #include<zvlidar_msgs/ZvisionLidarScan.h>
#endif

namespace zv_processor
{
    enum RingType: uint8_t
    {
        RingTypeUnknown = 0,
        FovId = 1,
        LineId
    };
    enum TimeStampType: uint8_t
    {
        UseFirstUdpPacket = 1,
        UseSystemClock
    };
    enum PointCloudType: uint8_t
    {
        PointCloudXYZI = 1,
        PointCloudXYZIRT,
        PointCloudXYZRGBA,
    };

    struct MsgConvertParam
    {
        MsgConvertParam()
        :ring_type(RingType::RingTypeUnknown)
        ,timestamp_type(TimeStampType::UseFirstUdpPacket){}

        RingType ring_type;
        TimeStampType timestamp_type;
        std::string frame_id;

        // for pointcloud
        PointCloudType pointcloud_type;
    };

    /** \brief Get config parameters from yaml file.
    * \param[in] path               path to the yaml config file
    * \param[out] cfgs                config parameters
    * \return   0 for ok, others for failure.
    */
    bool getProcessorsConfig(std::string path, std::vector<ProcessorConfig>& cfgs, std::string& infos);

    /** \brief Parsing heartbeat packet data.
    * \param[inout] info               path to the yaml config file
    * \return   0 for ok, others for failure.
    */
    bool parsingDiagnosticInfo(DiagnosticInfo& info);

#ifdef ENV_ROS
    /** \brief Convert pointcloud to packet message.
    * \param[in] src            pointcloud data
    * \param[in] param     convert parameter
    * \param[out] msg      packet message
    * \return   ros message.
    */
    void convertPoints2PacketMsg(const zvision::PointCloud& src, const MsgConvertParam& param, zvlidar_msgs_::ZvisionLidarScan& msg);

    /** \brief Convert pointcloud to points message.
    * \param[in] src            pointcloud data
    * \param[in] param     convert parameter
    * \param[out] msg      points message
    * \return   ros message.
    */
    void convertPoints2PointCloudMsg(const zvision::PointCloud& src, const MsgConvertParam& param, PointCloud2_& msg, ProcessorConfig& cfg);
    void convertImudata2ImuMsg(const zvision::imu_data_t & src, Imu_msg_& msg,const MsgConvertParam& param,ProcessorConfig& cfg);
#endif
}  // namespace zvision