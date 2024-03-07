#include<processor/processor.h>
#include<common/print.h>
#include<lidar_tools.h>
#include<processor/diagnostic_processor.h>
#include<util/translater.h>

namespace zv_processor
{

    zvision::DeviceType getDeviceTypeByString(std::string type)
    {
        if(type == "ML30S"){
            return zvision::DeviceType::LidarML30SA1;
        }
        else if(type == "ML30SPlus"){
            return zvision::DeviceType::LidarMl30SA1Plus;
        }
        else if(type == "ML30SPlusB1_40" || type == "ML30SPlusB1_90")
        {
            return zvision::DeviceType::LidarMl30SB1Plus;
        }
        else if(type == "MLXS"){
            return zvision::DeviceType::LidarMLX;
        }
        else if(type == "Exciton"){
            return zvision::DeviceType::LidarExciton;
        }
        else if(type == "30spb1IS1"){
            return zvision::DeviceType::LidarMl30SpB1IS1;
        }
        else
        {
            return zvision::DeviceType::LidarUnknown;
        }
    }

    std::string getStringByDeviceType(zvision::DeviceType type)
    {
        if(type == zvision::DeviceType::LidarML30SA1){
            return "ML30S";
        }
        else if(type == zvision::DeviceType::LidarMl30SA1Plus){
            return "ML30SPlus";
        }
        else if(type == zvision::DeviceType::LidarMl30SB1Plus)
        {
            if(zvision::is_ml30splus_b1_ep_mode_enable())
                return "ML30SPlusB1_40";
            else
                return "ML30SPlusB1_90";
        }
        else if(type == zvision::DeviceType::LidarMLX){
            return "MLXS";
        }
        else if(type == zvision::DeviceType::LidarExciton){
            return "Exciton";
        }
        else if(type == zvision::DeviceType::LidarMl30SpB1IS1){
            return "30spb1IS1";
        }
        else
        {
            return "Unknown";
        }
    }

    Processor::Processor(/* args */)
    :is_running_(false)
    ,offline_frames_(0)
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
        // trick
        if(config_.lidar_type == "ML30SPlusB1_40")
        {
            zvision::set_ml30splus_b1_ep_mode_enable(true);
        }
        if(config_.lidar_type == "30spb1IS1")
        {
            zvision::set_ml30splus_b1_ep_mode_enable(true);
        }


        device_type_ = getDeviceTypeByString(config_.lidar_type);
        // filter
        pointcloud_filter_.set_filter_param(config_.filter_param);
        // msg
        if(config_.pub_diagnostic && config_.online)
        {
            // diagnostic
            auto diag = DiagnosticProcessor::GetInstance();
            diag->addDevice(device_type_, config_.device_ip);
        }
#ifdef ROS_FOUND
        // std::cout << "ROS FOUND" << config_.pub_lidar_packet << config_.pub_pointcloud_xyzi << config_.pub_pointcloud_xyzirt << config_.pub_pointcloud_xyzrgba
        //     << config_.pub_pointcloud_blooming << std::endl;
        // std::cout << "Su ---------> " << config_.topic_tag_name << std::endl;
        if(config_.pub_lidar_packet)
            pub_packets_ = std::make_shared<ros::Publisher>(node->advertise<zvlidar_msgs_::ZvisionLidarScan>(config_.topic_tag_name + "_raw_packets", 20));

        if(config_.pub_pointcloud_xyzi)
            pub_pointcloud_xyzi_ = std::make_shared<ros::Publisher>(node->advertise<PointCloud2_>(config_.topic_tag_name + "_points_xyzi", 20));

        if(config_.pub_pointcloud_xyzirt)
            pub_pointcloud_xyzirt_ = std::make_shared<ros::Publisher>(node->advertise<PointCloud2_>(config_.topic_tag_name + "_points_xyzirt", 20));

        if(config_.pub_pointcloud_xyzrgba)
            pub_pointcloud_xyzrgba_ = std::make_shared<ros::Publisher>(node->advertise<PointCloud2_>(config_.topic_tag_name + "_points_xyzrgba", 20));

        if(config_.pub_pointcloud_blooming)
            pub_pointcloud_blooming_= std::make_shared<ros::Publisher>(node->advertise<PointCloud2_>(config_.topic_tag_name + "_points_blooming", 20));
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

        if(config_.pub_pointcloud_blooming)
            pub_pointcloud_blooming_= node->create_publisher<PointCloud2_>(config_.topic_tag_name + "_points_blooming", 20);
#endif
     }

    void Processor::start()
    {
        if(is_running_)
        {
            return;
        }

        /* init player */
        // online player
        if(config_.online)
        {
            online_player_.reset(new zvision::PointCloudProducer(config_.udp_port, \
                                                                config_.device_ip, \
                                                                config_.angle_path, \
                                                                config_.multicast_en, \
                                                                config_.multicast_ip, \
                                                                device_type_));
            online_player_->SetPointcloudBufferEnable(true);
            if(config_.pub_pointcloud_blooming)
                online_player_->SetProcessInternalPacketsEnable(true);

            /* start player*/
            int ret = online_player_->Start();
            while(ret != 0)
            {
                LOG_ERROR("Start online player[%s] failed, Msg:%s", config_.device_ip.c_str(), zvision::get_return_code_string(zvision::ReturnCode(ret)).c_str());
                Sleep(3000);
                /* restart*/
                ret = online_player_->Start();
            }

            //  get calibration data
            online_player_->GetCalibrationData(cal_lut_);
        }
        // offline player
        else
        {
            offline_player_.reset(new zvision::OfflinePointCloudProducer(config_.pcap_path, \
                                                                                                                                        config_.angle_path, \
                                                                                                                                        config_.device_ip, \
                                                                                                                                        config_.udp_port));
            
            offline_player_->SetInternalFrameMatchMethod(false);
            
            /* start player*/
            int frames = 0;
            zvision::DeviceType type;
            int ret = 0;
            if (ret = offline_player_->GetPointCloudInfo(frames, type))
            {
                LOG_ERROR("OfflinePointCloudProducer GetPointCloudInfo failed, ret = %d.\n", ret);
                return;
            }
            else
            {
                LOG_INFO("OfflinePointCloudProducer GetPointCloudInfo ok, frames: %d, type: %d\n", frames, type);
                if (0 == frames)
                {
                    LOG_ERROR("No frames found for lidar %s:%d.\n", config_.device_ip.c_str(), config_.udp_port);
                    return;
                }
            }

           offline_frames_ = frames;
            //  get calibration data
            offline_player_->GetCalibrationDataSinCosTable(cal_lut_);
        }

        // set filter calibration data
        if(cal_lut_.data.size() \
            && cal_lut_.device_type != zvision::DeviceType::LidarUnknown \
            && cal_lut_.scan_mode != zvision::ScanMode::ScanUnknown)
        {
            pointcloud_filter_.setCalibrationdata(cal_lut_);
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
        // stop player
        ///offline_player_
        if(online_player_)
        {
            online_player_->Stop();
        }
    }

    void Processor::Consumer()
    {
        int ret = 0;
        int frame_id = 0;
        time_t frame_ms = 1000.0f / config_.pcap_rate;
        std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> last_pub_time = \
		    std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());

        while(is_running_)
        {
            zvision::PointCloud points;
            if(this->online_player_)
            {    
                // std::cout << "Processor::Consumer" << std::endl;

                ret = this->online_player_->GetPointCloud(points, 200);
                points.dev_type = zvision::DeviceType::LidarMl30SpB1IS1;
            }
            else if(this->offline_player_)
            {
                ret = this->offline_player_->GetPointCloud(frame_id, points);
                frame_id ++;
                if(frame_id >= offline_frames_)
                {
                    frame_id = 0;
                }
            }
            if(ret)
            {
                LOG_ERROR("%s - %s GetPointCloud error, ret = %d.\n",config_.lidar_type.c_str(), config_.device_ip.c_str(), ret);
                continue;
            }
            /*  processing pointcloud*/
            process(points);

            // set pcap file play rate
            if(this->offline_player_)
            {
                //  get current timestamp
                auto cur_time = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
                auto span = cur_time.time_since_epoch().count() \
                                        - last_pub_time.time_since_epoch().count();
                if(span > 0)
                {
                    auto val = frame_ms - span;
                    if(val > 0)
                        std::this_thread::sleep_for(std::chrono::milliseconds(val));
                }
                last_pub_time = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
            }
        }
    }

    void Processor::process(const zvision::PointCloud& pointcloud)
    {
        if(pointcloud.dev_type != device_type_)
        {
            if((pointcloud.dev_type != zvision::DeviceType::LidarMl30SA1Plus) && (device_type_ != zvision::DeviceType::LidarMl30SB1Plus)
             && (pointcloud.dev_type != zvision::DeviceType::LidarMl30SpB1IS1))
            {
                LOG_WARN("Player type was not matched with pointcloud type. player[%s]/pointcloud[%s]\n", \
                    config_.lidar_type.c_str(), getStringByDeviceType(pointcloud.dev_type).c_str());
                return;
            }
        }

        zvision::PointCloud cloud = pointcloud;

           /*filter pointcloud */
        if(config_.filter_param.filter_mode != FilterMode::None) {
            pointcloud_filter_.run(pointcloud, cloud);
        }

#ifdef ENV_ROS
        /* convert to msg*/ 
        MsgConvertParam conv_param;
        conv_param.ring_type = config_.use_lidar_line_id?RingType::LineId:RingType::FovId;
        conv_param.timestamp_type = config_.use_lidar_time?TimeStampType::UseFirstUdpPacket:TimeStampType::UseSystemClock;
        conv_param.frame_id = config_.frame_id;
        /*publish lidar packet data*/
        // std::cout << "111111" << "---------" << config_.pub_lidar_packet << "---------" << pub_packets_  << "---------" << cloud.packets.size() << std::endl;
        if(config_.pub_lidar_packet && pub_packets_ && cloud.packets.size())
        {
            zvlidar_msgs_::ZvisionLidarScan scan;
            convertPoints2PacketMsg(cloud, conv_param, scan);
            pub_packets_->publish(scan);
        }

        if(!cloud.points.size())
            return;

        /* pub pointcloud */
        // pub pointcloud xyzrgba
        if(config_.pub_pointcloud_xyzrgba && pub_pointcloud_xyzrgba_)
        {
            PointCloud2_ msg;
            conv_param.pointcloud_type = PointCloudType::PointCloudXYZRGBA;
            convertPoints2PointCloudMsg(cloud, conv_param, msg);
            pub_pointcloud_xyzrgba_->publish(msg);
        }
        // pub pointcloud xyzi
        if(config_.pub_pointcloud_xyzi && pub_pointcloud_xyzi_)
        {
            PointCloud2_ msg;
            conv_param.pointcloud_type = PointCloudType::PointCloudXYZI;
            convertPoints2PointCloudMsg(cloud, conv_param, msg);
            pub_pointcloud_xyzi_->publish(msg);
        }
        // pub pointcloud xyzirt
        if(config_.pub_pointcloud_xyzirt && pub_pointcloud_xyzirt_)
        {
            PointCloud2_ msg;
            conv_param.pointcloud_type = PointCloudType::PointCloudXYZIRT;
            convertPoints2PointCloudMsg(cloud, conv_param, msg);
            pub_pointcloud_xyzirt_->publish(msg);
        }
        // pub pointcloud blooming
        if(config_.pub_pointcloud_blooming && pub_pointcloud_blooming_&& cloud.use_blooming)
        {
            PointCloud2_ msg;
            conv_param.pointcloud_type = PointCloudType::PointCloudBlooming;
            convertPoints2PointCloudMsg(cloud, conv_param, msg);
            pub_pointcloud_blooming_->publish(msg);
        }
        
#else
        LOG_INFO("2-Get PointCloud OK, npoints:%d\n", cloud.points.size());
#endif
    }

} // namespace zvision