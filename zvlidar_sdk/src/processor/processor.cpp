#include<processor/processor.h>
#include<common/print.h>
#include<tcp_tools.h>
#include<tcp_tools.h>
#include<util/translater.h>

#include "point_cloud.h"

namespace zv_processor
{
    zvision::DeviceType getDeviceTypeByString(std::string type)
    {
        if(type == "ez6_b2"){
            return zvision::DeviceType::LidarEZ6_B2;
        }
        else if(type == "nz1_a2"){
            return zvision::DeviceType::LidarNZ1_A2;
        }
        else if(type == "mrz16"){
            return zvision::DeviceType::LidarMRZ16;
        }
        else
        {
            return zvision::DeviceType::LidarUnknown;
        }
    }

    std::string getStringByDeviceType(zvision::DeviceType type)
    {
        if(type == zvision::DeviceType::LidarEZ6_B2){
            return "EZ6_B2";
        }
        else if(type == zvision::DeviceType::LidarNZ1_A2){
            return "NZ1_A2";
        }
        else if(type == zvision::DeviceType::LidarMRZ16){
            return "MRZ16";
        }
        else
        {
            return "Unknown";
        }
    }

    Processor::Processor(/* args */)
    :offline_frames_(0),
    is_running_(false)
    {
    }

    Processor::~Processor()
    {
    }

#ifdef ENV_ROS
    void Processor::init(std::shared_ptr<NodeHandle_> node, const ProcessorConfig& cfg)
#else
    void Processor::init(const ProcessorConfig& cfg)
#endif
    {
        // config
        config_ = cfg;

        device_type_ = getDeviceTypeByString(config_.lidar_type);
#ifdef ROS_FOUND
        if(config_.pub_lidar_packet)
            pub_packets_ = std::make_shared<ros::Publisher>(node->advertise<zvlidar_msgs_::ZvisionLidarScan>(config_.topic_tag_name + "_raw_packets", 20));

        if(config_.pub_pointcloud_xyzi)
            pub_pointcloud_xyzi_ = std::make_shared<ros::Publisher>(node->advertise<PointCloud2_>(config_.topic_tag_name + "_points_xyzi", 20));

        if(config_.pub_pointcloud_xyzirt)
            pub_pointcloud_xyzirt_ = std::make_shared<ros::Publisher>(node->advertise<PointCloud2_>(config_.topic_tag_name + "_points_xyzirt", 20));

        if(config_.pub_pointcloud_xyzrgba)
            pub_pointcloud_xyzrgba_ = std::make_shared<ros::Publisher>(node->advertise<PointCloud2_>(config_.topic_tag_name + "_points_xyzrgba", 20));

        if(config_.pub_imu_msg)
        {
            pub_imu_msg_ = std::make_shared<ros::Publisher>(node->advertise<Imu_msg_>(config_.topic_tag_name + "_imu_msg", 20));
        }

#endif

#ifdef ROS2_FOUND
        if(config_.pub_lidar_packet)
            pub_packets_ = node->create_publisher<zvlidar_msgs_::ZvisionLidarScan>(config_.topic_tag_name + "_raw_packets", 20);

        if(config_.pub_pointcloud_xyzi)
            pub_pointcloud_xyzi_ = node->create_publisher<PointCloud2_>(config_.topic_tag_name + "_points_xyzi", 20);

        if(config_.pub_pointcloud_xyzirt)
            pub_pointcloud_xyzirt_ = node->create_publisher<PointCloud2_>(config_.topic_tag_name + "_points_xyzirt", 20);

        if(config_.pub_pointcloud_xyzrgba)
            pub_pointcloud_xyzrgba_ = node->create_publisher<PointCloud2_>(config_.topic_tag_name + "_points_xyzrgba", 20);

        if(config_.pub_imu_msg)
        {
            pub_imu_msg_ = node->create_publisher<Imu_msg_>(config_.topic_tag_name + "_imu_msg", 20);
        }
#endif
     }

    void Processor::start()
    {
        if(is_running_)
        {
            return;
        }

        /* init player */
        // zvision official SDK: one class zvision::PointCloudProducer with two
        // constructors - UDP (EZ6_B2 / NZ1_A2) or MRZ16 serial ("EE FF",
        // dual UART). transport in config.yaml picks which constructor to use.
        if(config_.online)
        {
            if(config_.transport == "serial")
            {
                /* MRZ16 (EE-FF) dual serial */
                const std::string cmd_port =
                    config_.port_send.empty() ? "/dev/ttyACM2" : config_.port_send;
                const std::string data_port =
                    config_.port_recv.empty() ? "/dev/ttyACM0" : config_.port_recv;
                const int baud_cmd  = config_.baud_send > 0 ? config_.baud_send : 9600;
                const int baud_data = config_.baud_recv > 0 ? config_.baud_recv : 3125000;

                device_type_ = zvision::DeviceType::LidarMRZ16;
                online_player_.reset(new zvision::PointCloudProducer(
                    cmd_port, data_port, config_.angle_path,
                    zvision::DeviceType::LidarMRZ16, baud_cmd, baud_data));
                LOG_INFO("Start MRZ16 serial player: cmd[%s@%d] data[%s@%d] "
                         "angle_file[%s] imu_pub[%s].\n",
                         cmd_port.c_str(), baud_cmd, data_port.c_str(), baud_data,
                         config_.angle_path.empty() ? "<none, auto via $LDCMD>"
                                                    : config_.angle_path.c_str(),
                         get_switch_str(config_.pub_imu_msg));
            }
            else
            {
                /* UDP (EZ6_B2 / NZ1_A2) - original behaviour */
                const std::string ip =
                    config_.device_ip.empty() ? "192.168.10.108" : config_.device_ip;
                online_player_.reset(new zvision::PointCloudProducer(
                    config_.udp_port, ip, config_.angle_path,
                    config_.multicast_en, config_.multicast_ip, device_type_));
                LOG_INFO("Start UDP player: ip[%s] port[%d] type[%s].\n",
                         ip.c_str(), config_.udp_port,
                         getStringByDeviceType(device_type_).c_str());
            }

            online_player_->SetPointcloudBufferEnable(true);
            /* start player*/
            int ret = online_player_->Start();
            while(ret != 0)
            {
                LOG_ERROR("Start online player[%s] failed, Msg:%s", config_.device_ip.c_str(), zvision::get_return_code_string(zvision::ReturnCode(ret)).c_str());
                Sleep(3000);
                /* restart*/
                ret = online_player_->Start();
            }
        }

        /*  start pointcloud thread */
        is_running_ = true;
        consumer_thre_.reset(new std::thread(std::bind(&Processor::Consumer,this)));
    }

    void Processor::stop()
    {

        if(!is_running_)
            return;
        
        // stop porcessing thread
        is_running_ = false;
        if(consumer_thre_){
            if(consumer_thre_->joinable())
                consumer_thre_->join();
            
            consumer_thre_.reset();
        }

        if(online_player_)
        {
            online_player_->Stop();
        }
    }

    void Processor::Consumer()
    {
        int ret = 0;
        
        while(is_running_)
        {
            /*  processing pointcloud */
            zvision::PointCloud points;
            if(this->online_player_)
            {    
                ret = this->online_player_->GetPointCloud(points, 1);
            }
            if(ret == 0)
            {
                process(points);
            }
            
            /* processing imu data */
            if(config_.pub_imu_msg)
            {
	            zvision::imu_data_t imu_data;
                if(this->online_player_)
                {    
                    ret = this->online_player_->GetImuData(imu_data, 1);
                }

                if(ret == 0)
                {
                    process_imu(imu_data);
                }
            }
        }
    }

    void Processor::process(const zvision::PointCloud& pointcloud)
    {
        if(pointcloud.dev_type != device_type_)
        {
            LOG_WARN("Player type was not matched with pointcloud type. player[%s]/pointcloud[%s]\n", \
                    config_.lidar_type.c_str(), getStringByDeviceType(pointcloud.dev_type).c_str());
            return;
        }

        zvision::PointCloud cloud = pointcloud;

        // LOG_INFO("in process\n");

#ifdef ENV_ROS
        /* convert to msg*/ 
        MsgConvertParam conv_param;
        conv_param.ring_type = config_.use_lidar_line_id?RingType::LineId:RingType::FovId;
        conv_param.timestamp_type = config_.use_lidar_time?TimeStampType::UseFirstUdpPacket:TimeStampType::UseSystemClock;
        conv_param.frame_id = config_.frame_id;
        /*publish lidar packet data*/
#if 0
        if(config_.pub_lidar_packet && pub_packets_ && cloud.packets.size())
        {
            zvlidar_msgs_::ZvisionLidarScan scan;
            convertPoints2PacketMsg(cloud, conv_param, scan);
            pub_packets_->publish(scan);
        }
#endif

        if(!cloud.points.size())
            return;

        /* pub pointcloud */
        // pub pointcloud xyzrgba
        if(config_.pub_pointcloud_xyzrgba && pub_pointcloud_xyzrgba_)
        {
            PointCloud2_ msg;
            conv_param.pointcloud_type = PointCloudType::PointCloudXYZRGBA;
            convertPoints2PointCloudMsg(cloud, conv_param, msg, config_);
            pub_pointcloud_xyzrgba_->publish(msg);
        }
        // pub pointcloud xyzi
        if(config_.pub_pointcloud_xyzi && pub_pointcloud_xyzi_)
        {
            PointCloud2_ msg;
            conv_param.pointcloud_type = PointCloudType::PointCloudXYZI;
            convertPoints2PointCloudMsg(cloud, conv_param, msg, config_);
            // std::cout << msg << std::endl;
            pub_pointcloud_xyzi_->publish(msg);
        }
        // pub pointcloud xyzirt
        if(config_.pub_pointcloud_xyzirt && pub_pointcloud_xyzirt_)
        {
            PointCloud2_ msg;
            conv_param.pointcloud_type = PointCloudType::PointCloudXYZIRT;
            convertPoints2PointCloudMsg(cloud, conv_param, msg, config_);
            pub_pointcloud_xyzirt_->publish(msg);
        }
        
#else
        LOG_INFO("2-Get PointCloud OK, npoints:%d\n", cloud.points.size());
#endif
    }

    void Processor::process_imu(const zvision::imu_data_t &imu_data)
    {
        zvision::imu_data_t imu_data1 = imu_data;

#ifdef ENV_ROS
        /* pub pointcloud */
        // pub imu msg
        MsgConvertParam conv_param;
        conv_param.timestamp_type = config_.use_lidar_time?TimeStampType::UseFirstUdpPacket:TimeStampType::UseSystemClock;
        if(config_.pub_imu_msg && pub_imu_msg_)
        {
            Imu_msg_ msg;
            convertImudata2ImuMsg(imu_data1, msg, conv_param,config_);
            pub_imu_msg_->publish(msg);
        }
#else
        LOG_INFO("2-Get imu data OK, timestamp:%lu\n",imu_data1.timestamp_us);
#endif
    }

} // namespace zvision
