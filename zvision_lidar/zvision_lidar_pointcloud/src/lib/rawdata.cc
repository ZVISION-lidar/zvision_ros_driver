/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *  Copyright (C) 2019 Zvision
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  ZVISION 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw ZVISION LIDAR packets into useful
 *  formats.
 *
 */
#include "rawdata.h"
#include "tools.h"

namespace zvision_lidar_rawdata
{
RawData::RawData(const rclcpp::NodeOptions &options)
:rclcpp::Node("zvision_rawdata_node", options)
{
  this->cal_init_ok_ = false;
  this->use_lidar_time_ = false; /* default is local timestamp */
  this->cal_.reset(new zvision::CalibrationData());
  this->cal_lut_.reset(new zvision::PointCalibrationTable());
  point_line_number_.resize(MAX_POINTS, 0);
  x_trans = 0.0;
  y_trans = 0.0;
  z_trans = 0.0;
  x_rotation = 0.0;
  y_rotation = 0.0;
  z_rotation = 0.0;
}
RawData::~RawData()
{
    if(online_calibration_thread_)
        online_calibration_thread_->join();
}
//import cali
void RawData::loadConfigFile()
{
  std::string anglePath;
  std::string model;
  double xt, yt, zt, xr, yr, zr;
  this->use_lidar_time_ = this->declare_parameter("use_lidar_time",false);
  anglePath = this->declare_parameter("angle_path",std::string(""));
  model = this->declare_parameter("model",std::string("ML30SA1"));
  
  if(model == std::string("ML30B1"))
      device_type_ = zvision::ML30B1;
  else if(model == std::string("ML30SA1"))
      device_type_ = zvision::ML30SA1;
  else if(model == std::string("MLX"))
      device_type_ = zvision::MLX;
  else if(model == std::string("MLXA1"))
      device_type_ = zvision::MLXA1;
  else if(model == std::string("MLXS"))
      device_type_ = zvision::MLXS;
  else
      device_type_ = zvision::Unknown;

  xt = this->declare_parameter("x_tra",0.0);
  yt = this->declare_parameter("y_tra",0.0);
  zt = this->declare_parameter("z_tra",0.0);
  xr = this->declare_parameter("x_rot",0.0);
  yr = this->declare_parameter("y_rot",0.0);
  zr = this->declare_parameter("z_rot",0.0);

  this->x_trans = xt;
  this->y_trans = yt;
  this->z_trans = zt;
  this->x_rotation = xr / 180.0 * M_PI;;
  this->y_rotation = yr / 180.0 * M_PI;;
  this->z_rotation = zr / 180.0 * M_PI;;

  RCLCPP_INFO(this->get_logger(),"Coordinate transformation\n xt:%6.3f yt:%6.3f zt:%6.3f\n xr:%6.3f yr:%6.3f zr:%6.3f",
           xt, yt, zt, xr, yr, zr);

  RCLCPP_INFO(this->get_logger(),"Using lidar time [%d]\n", this->use_lidar_time_);


  if(!this->cal_)
      this->cal_.reset(new zvision::CalibrationData());

  if(!this->cal_lut_)
      this->cal_lut_.reset(new zvision::PointCalibrationTable());

  if(!anglePath.empty())
  {
      //read calibration file
      if(0 != zvision::LidarTools::ReadCalibrationFile(anglePath, *(this->cal_.get())))
      {
          RCLCPP_ERROR(this->get_logger(),"Read calibration file error, %s", anglePath.c_str());
      }

      //compute sin cos
      zvision::LidarTools::ComputeCalibrationData(*(this->cal_.get()), *(this->cal_lut_.get()));
      if((device_type_ == zvision::Unknown) || (device_type_ != this->cal_lut_->model))
          RCLCPP_ERROR(this->get_logger(),"Device %s's calibration not matched, device type is %s, cal type is %s", this->dev_ip_.c_str(), zvision::LidarTools::GetDeviceTypeString(device_type_).c_str(),
                zvision::LidarTools::GetDeviceTypeString(this->cal_lut_->model).c_str());
      else
      {
          RCLCPP_INFO(this->get_logger(),"Load device %s's calibration file ok, device type is %s", this->dev_ip_.c_str(), zvision::LidarTools::GetDeviceTypeString(device_type_).c_str());
          zvision::LidarTools::ComputePointLineNumber(*this->cal_lut_, this->point_line_number_);
          this->cal_init_ok_ = true;
      }
  }
  else
  {
      // raw data output topic
      std::cout << "<<<<<<<<<<<<<<<<<<<" <<std::endl;
      online_calibration_thread_.reset(
          new std::thread(&RawData::PollCalibrationData, this));
  }
}


bool RawData::isCalibrationInitOk()
{
    return this->cal_init_ok_;
}

//------------------------------------------------------------

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack(const zvision_lidar_msgs::msg::ZvisionLidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
{
  float azimuth;
  int intensity;
  pcl::PointXYZI point;
  const unsigned char *data = (const unsigned char *)(&(pkt.data[0]));
  int udp_seq = 0;
  udp_seq = (int)(data[3]) + ((data[2] & 0xF) << 8);


  if(device_type_ == zvision::ML30SA1)
  {
	  for (int group = 0; group < GROUP_PER_PACKET; group++)   // 1 packet:40 data group
	  {
		  const unsigned char *pc = data + 4 + group * 8 * 4;/*group address*/
          int group_index = udp_seq * GROUP_PER_PACKET * POINT_PER_GROUP + group * POINT_PER_GROUP;/*group number, to search the angle file *.cal*/

          for (int laser_id = 0; laser_id < POINT_PER_GROUP; laser_id++)/*8 Points per laser*/
		  {
              int point_num = group_index + laser_id;
              int disTemp = 0;


			  //dis_high
			  unsigned char Dis_High = (u_char)(pc[0 + 4 * laser_id]);
			  unsigned char Dis_Low  = (u_char)(pc[1 + 4 * laser_id]);
			  unsigned char Int_High = (u_char)(pc[2 + 4 * laser_id]);
			  unsigned char Int_Low  = (u_char)(pc[3 + 4 * laser_id]);


			  disTemp = (((Dis_High << 8) + Dis_Low) << 3) + (int)((Int_High & 0xE0) >> 5);
			  double distantce_real_live = disTemp * 0.0015;/*distance from udp*/
			  intensity = (((Int_High & 0x1F) << 8) + (Int_Low));/*reflectivity from udp*/
              intensity = intensity & 0x3FF;

              zvision::PointCalibrationData& cal = this->cal_lut_->data[point_num];
              point.x = distantce_real_live * cal.cos_ele * cal.sin_azi;/*x*/
              point.y = distantce_real_live * cal.cos_ele * cal.cos_azi;/*y*/
              point.z = distantce_real_live * cal.sin_ele;/*z*/
              point.intensity = intensity;

			  /*apply transform*/
			  double Ax = x_rotation;
			  double Ay = y_rotation;
			  double Az = z_rotation;
			  double x, y, z;
			  pcl::PointXYZI sourPoint   = point;
			  pcl::PointXYZI xTransPoint = point;
			  pcl::PointXYZI yTransPoint = point;
			  pcl::PointXYZI zTransPoint = point;

			  x = sourPoint.x;/*X rotation*/
			  y = sourPoint.y;
			  z = sourPoint.z;
			  xTransPoint.x = x;

			  xTransPoint.y = y * std::cos(Ax) - z * std::sin(Ax);
			  xTransPoint.z = y * std::sin(Ax) + z * std::cos(Ax);

			  x = xTransPoint.x;/*Y rotation*/
			  y = xTransPoint.y;
			  z = xTransPoint.z;

			  yTransPoint.x = x * std::cos(Ay) + z * std::sin(Ay);
			  yTransPoint.y = y;
			  yTransPoint.z = x * (-std::sin(Ay)) + z * std::cos(Ay);

			  x = yTransPoint.x;/*Z rotation*/
			  y = yTransPoint.y;
			  z = yTransPoint.z;

			  zTransPoint.x = x * std::cos(Az) + y * (-std::sin(Az));
			  zTransPoint.y = x * std::sin(Az) + y * std::cos(Az);
			  zTransPoint.z = z;

			  point.x = zTransPoint.x + x_trans;
			  point.y = zTransPoint.y + y_trans;
			  point.z = zTransPoint.z + z_trans;

              pointcloud->at(point_num) = point;/*not filled when pkt loss*/
		  }
	  }
  }
  else if(device_type_ == zvision::ML30B1)
  {
	  for (int group = 0; group < GROUP_PER_PACKET_ML30; group++)   // 1 packet:80 data group
	  {
		  const unsigned char *pc = data + 4 + group * 8 * 2;/*group address*/
          int group_index= udp_seq * GROUP_PER_PACKET_ML30 * POINT_PER_GROUP_ML30 + group * POINT_PER_GROUP_ML30;/*group number, to search the angle file *.cal*/

		  for (int laser_id = 0; laser_id < POINT_PER_GROUP_ML30; laser_id++)/*3 Points per laser*/
          {
              int point_num = group_index + laser_id;
              int disTemp = 0;

			  //dis_high
			  unsigned char Dis_High = (u_char)(pc[4 + 4 * laser_id]);
			  unsigned char Dis_Low  = (u_char)(pc[5 + 4 * laser_id]);
			  unsigned char Int_High = (u_char)(pc[6 + 4 * laser_id]);
			  unsigned char Int_Low  = (u_char)(pc[7 + 4 * laser_id]);

			  disTemp = (((Dis_High << 8) + Dis_Low) << 3) + (int)((Int_High & 0xE0) >> 5);
			  double distantce_real_live = disTemp * 0.0015;/*distance from udp*/
			  intensity = (((Int_High & 0x1F) << 8) + (Int_Low));/*reflectivity from udp*/
			  intensity = intensity & 0x3FF;

              zvision::PointCalibrationData& cal = this->cal_lut_->data[point_num];
              point.x = distantce_real_live * cal.cos_ele * cal.sin_azi;/*x*/
              point.y = distantce_real_live * cal.cos_ele * cal.cos_azi;/*y*/
              point.z = distantce_real_live * cal.sin_ele;/*z*/
			  point.intensity = intensity;

			  /*apply transform*/
			  double Ax = x_rotation;
			  double Ay = y_rotation;
			  double Az = z_rotation;
			  double x, y, z;
			  pcl::PointXYZI sourPoint   = point;
			  pcl::PointXYZI xTransPoint = point;
			  pcl::PointXYZI yTransPoint = point;
			  pcl::PointXYZI zTransPoint = point;

			  x = sourPoint.x;/*X rotation*/
			  y = sourPoint.y;
			  z = sourPoint.z;
			  xTransPoint.x = x;
			  xTransPoint.y = y * std::cos(Ax) - z * std::sin(Ax);
			  xTransPoint.z = y * std::sin(Ax) + z * std::cos(Ax);

			  x = xTransPoint.x;/*Y rotation*/
			  y = xTransPoint.y;
			  z = xTransPoint.z;

			  yTransPoint.x = x * std::cos(Ay) + z * std::sin(Ay);
			  yTransPoint.y = y;
			  yTransPoint.z = x * (-std::sin(Ay)) + z * std::cos(Ay);

			  x = yTransPoint.x;/*Z rotation*/
			  y = yTransPoint.y;
			  z = yTransPoint.z;

			  zTransPoint.x = x * std::cos(Az) + y * (-std::sin(Az));
			  zTransPoint.y = x * std::sin(Az) + y * std::cos(Az);
			  zTransPoint.z = z;

			  point.x = zTransPoint.x + x_trans;
			  point.y = zTransPoint.y + y_trans;
			  point.z = zTransPoint.z + z_trans;

              pointcloud->at(point_num) = point;/*not filled when pkt loss*/
		  }
	  }
  }
  else if(device_type_ == zvision::MLX)
  {
      for (int group = 0; group < GROUP_PER_PACKET_MLX; group++)   // 1 packet:80 data group
      {
          const unsigned char *pc = data + group * 8 * 2;/*group address*/
          int group_index = udp_seq * GROUP_PER_PACKET_MLX * POINT_PER_GROUP_MLX + group * POINT_PER_GROUP_MLX;/*group number, to search the angle file *.cal*/

          for (int laser_id = 0; laser_id < POINT_PER_GROUP_MLX; laser_id++)/*3 Points per laser*/
          {
              int point_num = group_index + laser_id;
              int disTemp = 0;

              //dis_high
              unsigned char Dis_High = (u_char)(pc[4 + 4 * laser_id]);
              unsigned char Dis_Low  = (u_char)(pc[5 + 4 * laser_id]);
              unsigned char Int_High = (u_char)(pc[6 + 4 * laser_id]);
              unsigned char Int_Low  = (u_char)(pc[7 + 4 * laser_id]);

              disTemp = (((Dis_High << 8) + Dis_Low) << 3) + (int)((Int_High & 0xE0) >> 5);
              double distantce_real_live = disTemp * 0.0015;/*distance from udp*/
              intensity = (((Int_High & 0x1F) << 8) + (Int_Low));/*reflectivity from udp*/
              intensity = intensity & 0x3FF;

              zvision::PointCalibrationData& cal = this->cal_lut_->data[point_num];
              point.x = distantce_real_live * cal.cos_ele * cal.sin_azi;/*x*/
              point.y = distantce_real_live * cal.cos_ele * cal.cos_azi;/*y*/
              point.z = distantce_real_live * cal.sin_ele;/*z*/
              point.intensity = intensity;

              /*apply transform*/
              double Ax = x_rotation;
              double Ay = y_rotation;
              double Az = z_rotation;
              double x, y, z;
              pcl::PointXYZI sourPoint   = point;
              pcl::PointXYZI xTransPoint = point;
              pcl::PointXYZI yTransPoint = point;
              pcl::PointXYZI zTransPoint = point;

              x = sourPoint.x;/*X rotation*/
              y = sourPoint.y;
              z = sourPoint.z;
              xTransPoint.x = x;
              xTransPoint.y = y * std::cos(Ax) - z * std::sin(Ax);
              xTransPoint.z = y * std::sin(Ax) + z * std::cos(Ax);

              x = xTransPoint.x;/*Y rotation*/
              y = xTransPoint.y;
              z = xTransPoint.z;

              yTransPoint.x = x * std::cos(Ay) + z * std::sin(Ay);
              yTransPoint.y = y;
              yTransPoint.z = x * (-std::sin(Ay)) + z * std::cos(Ay);

              x = yTransPoint.x;/*Z rotation*/
              y = yTransPoint.y;
              z = yTransPoint.z;

              zTransPoint.x = x * std::cos(Az) + y * (-std::sin(Az));
              zTransPoint.y = x * std::sin(Az) + y * std::cos(Az);
              zTransPoint.z = z;

              point.x = zTransPoint.x + x_trans;
              point.y = zTransPoint.y + y_trans;
              point.z = zTransPoint.z + z_trans;

              pointcloud->at(point_num) = point;/*not filled when pkt loss*/
          }
      }
  }
  else if(device_type_ == zvision::MLXA1)
  {
      for (int group = 0; group < GROUP_PER_PACKET_MLX; group++)   // 1 packet:80 data group
      {
          const unsigned char *pc = data + group * 8 * 2 + 4;/*group address*/
          int group_index = udp_seq * GROUP_PER_PACKET_MLX * POINT_PER_GROUP_MLX + group * POINT_PER_GROUP_MLX;/*group number, to search the angle file *.cal*/

          for (int laser_id = 0; laser_id < POINT_PER_GROUP_MLX; laser_id++)/*3 Points per laser*/
          {
              int point_num = group_index + laser_id;
              int disTemp = 0;

              //dis_high
              unsigned char Dis_High = (u_char)(pc[4 + 4 * laser_id]);
              unsigned char Dis_Low  = (u_char)(pc[5 + 4 * laser_id]);
              unsigned char Int_High = (u_char)(pc[6 + 4 * laser_id]);
              unsigned char Int_Low  = (u_char)(pc[7 + 4 * laser_id]);

              disTemp = (((Dis_High << 8) + Dis_Low) << 3) + (int)((Int_High & 0xE0) >> 5);
              double distantce_real_live = disTemp * 0.0015;/*distance from udp*/
              intensity = (((Int_High & 0x1F) << 8) + (Int_Low));/*reflectivity from udp*/
              intensity = intensity & 0x3FF;

              zvision::PointCalibrationData& cal = this->cal_lut_->data[point_num];
              point.x = distantce_real_live * cal.cos_ele * cal.sin_azi;/*x*/
              point.y = distantce_real_live * cal.cos_ele * cal.cos_azi;/*y*/
              point.z = distantce_real_live * cal.sin_ele;/*z*/
              point.intensity = intensity;

              /*apply transform*/
              double Ax = x_rotation;
              double Ay = y_rotation;
              double Az = z_rotation;
              double x, y, z;
              pcl::PointXYZI sourPoint   = point;
              pcl::PointXYZI xTransPoint = point;
              pcl::PointXYZI yTransPoint = point;
              pcl::PointXYZI zTransPoint = point;

              x = sourPoint.x;/*X rotation*/
              y = sourPoint.y;
              z = sourPoint.z;
              xTransPoint.x = x;
              xTransPoint.y = y * std::cos(Ax) - z * std::sin(Ax);
              xTransPoint.z = y * std::sin(Ax) + z * std::cos(Ax);

              x = xTransPoint.x;/*Y rotation*/
              y = xTransPoint.y;
              z = xTransPoint.z;

              yTransPoint.x = x * std::cos(Ay) + z * std::sin(Ay);
              yTransPoint.y = y;
              yTransPoint.z = x * (-std::sin(Ay)) + z * std::cos(Ay);

              x = yTransPoint.x;/*Z rotation*/
              y = yTransPoint.y;
              z = yTransPoint.z;

              zTransPoint.x = x * std::cos(Az) + y * (-std::sin(Az));
              zTransPoint.y = x * std::sin(Az) + y * std::cos(Az);
              zTransPoint.z = z;

              point.x = zTransPoint.x + x_trans;
              point.y = zTransPoint.y + y_trans;
              point.z = zTransPoint.z + z_trans;

              pointcloud->at(point_num) = point;/*not filled when pkt loss*/
          }
      }
  }
  else if(device_type_ == zvision::MLXS)
  {
      for (int group = 0; group < GROUP_PER_PACKET_MLXS; group++)   // 1 packet:80 data group
      {
          const unsigned char *pc = data + group * 8 * 2 + 4;/*group address*/
          int group_index = udp_seq * GROUP_PER_PACKET_MLXS * POINT_PER_GROUP_MLXS + group * POINT_PER_GROUP_MLXS;/*group number, to search the angle file *.cal*/

          for (int laser_id = 0; laser_id < POINT_PER_GROUP_MLXS; laser_id++)/*3 Points per laser*/
          {
              int point_num = group_index + laser_id;
              int disTemp = 0;

              //dis_high
              unsigned char Dis_High = (u_char)(pc[4 + 4 * laser_id]);
              unsigned char Dis_Low  = (u_char)(pc[5 + 4 * laser_id]);
              unsigned char Int_High = (u_char)(pc[6 + 4 * laser_id]);
              unsigned char Int_Low  = (u_char)(pc[7 + 4 * laser_id]);

              disTemp = (((Dis_High << 8) + Dis_Low) << 3) + (int)((Int_High & 0xE0) >> 5);
              double distantce_real_live = disTemp * 0.0015;/*distance from udp*/
              intensity = (((Int_High & 0x1F) << 8) + (Int_Low));/*reflectivity from udp*/
              intensity = intensity & 0x3FF;

              zvision::PointCalibrationData& cal = this->cal_lut_->data[point_num];
              point.x = distantce_real_live * cal.cos_ele * cal.sin_azi;/*x*/
              point.y = distantce_real_live * cal.cos_ele * cal.cos_azi;/*y*/
              point.z = distantce_real_live * cal.sin_ele;/*z*/
              point.intensity = intensity;

              /*apply transform*/
              double Ax = x_rotation;
              double Ay = y_rotation;
              double Az = z_rotation;
              double x, y, z;
              pcl::PointXYZI sourPoint   = point;
              pcl::PointXYZI xTransPoint = point;
              pcl::PointXYZI yTransPoint = point;
              pcl::PointXYZI zTransPoint = point;

              x = sourPoint.x;/*X rotation*/
              y = sourPoint.y;
              z = sourPoint.z;
              xTransPoint.x = x;
              xTransPoint.y = y * std::cos(Ax) - z * std::sin(Ax);
              xTransPoint.z = y * std::sin(Ax) + z * std::cos(Ax);

              x = xTransPoint.x;/*Y rotation*/
              y = xTransPoint.y;
              z = xTransPoint.z;

              yTransPoint.x = x * std::cos(Ay) + z * std::sin(Ay);
              yTransPoint.y = y;
              yTransPoint.z = x * (-std::sin(Ay)) + z * std::cos(Ay);

              x = yTransPoint.x;/*Z rotation*/
              y = yTransPoint.y;
              z = yTransPoint.z;

              zTransPoint.x = x * std::cos(Az) + y * (-std::sin(Az));
              zTransPoint.y = x * std::sin(Az) + y * std::cos(Az);
              zTransPoint.z = z;

              point.x = zTransPoint.x + x_trans;
              point.y = zTransPoint.y + y_trans;
              point.z = zTransPoint.z + z_trans;

              pointcloud->at(point_num) = point;/*not filled when pkt loss*/
          }
      }
  }
  else{
  	std::cout<< "WRONG MECHINE TYPE !"<<std::endl;
  }
}

//------------------------------------------------------------

/** @brief get timestamp from raw udp packet
 *
 *  @param pkt raw packet to get time info
 *  @param unix_microsec unix time(micro seconds)
 */
void RawData::getTimeStampFromUdpPkt(const zvision_lidar_msgs::msg::ZvisionLidarPacket& pkt, long long int& unix_sec, long long int& unix_microsec)
{
    const unsigned char *data = (const unsigned char *)(&(pkt.data[0]));
    int gps_ptp_status_2_bits_ = ((data[2] & 0x30) >> 4);

    if (gps_ptp_status_2_bits_ & 0x2)//PTP
    {
        uint64_t seconds = 0;
        seconds = 0;
        seconds += ((uint64_t)data[1304 - 20 + 0] << 40);
        seconds += ((uint64_t)data[1304 - 20 + 1] << 32);
        seconds += ((uint64_t)data[1304 - 20 + 2] << 24);
        seconds += ((uint64_t)data[1304 - 20 + 3] << 16);
        seconds += ((uint64_t)data[1304 - 20 + 4] << 8);
        seconds += ((uint64_t)data[1304 - 20 + 5]);

        int MillS = (int)(data[1304 - 20 + 6] << 8) + data[1304 - 20 + 7];
        int MicroS = (int)(data[1304 - 20 + 8] << 8) + data[1304 - 20 + 9];

        unix_sec = seconds;
        unix_microsec = MillS * 1000 + MicroS;
        //return (uint64_t)seconds * 1000000000 + MillS * 1000000 + MicroS * 1000;
    }
    else//GPS
    {
        struct tm tm_;
        tm_.tm_year = data[1304 - 20 + 0] + 2000 - 1900;
        tm_.tm_mon = data[1304 - 20 + 1] - 1;
        tm_.tm_mday = data[1304 - 20 + 2];
        tm_.tm_hour = data[1304 - 20 + 3];
        tm_.tm_min = data[1304 - 20 + 4];
        tm_.tm_sec = data[1304 - 20 + 5];
        tm_.tm_isdst = 0;

        time_t seconds = timegm(&tm_);
        int MillS = (int)(data[1304 - 20 + 6] << 8) + data[1304 - 20 + 7];
        int MicroS = (int)(data[1304 - 20 + 8] << 8) + data[1304 - 20 + 9];

        unix_sec = seconds;
        unix_microsec = MillS * 1000 + MicroS;
        //return (uint64_t)seconds * 1000000000 + MillS * 1000000 + MicroS * 1000;
    }
}

void RawData::PollCalibrationData(void) {
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if(this->cal_init_ok_)
        break;
    zvision::CalibrationData cal;
    int ret = 0;
    if(0 != (ret = zvision::LidarTools::GetOnlineCalibrationData( this->dev_ip_, cal)))
        RCLCPP_WARN(this->get_logger(),"Unable to get online calibration from %s.", this->dev_ip_.c_str());
    else {
        zvision::LidarTools::ComputeCalibrationData(cal, *(cal_lut_.get()));
        zvision::LidarTools::ComputePointLineNumber(*(cal_lut_.get()), this->point_line_number_);

        this->cal_init_ok_ = true;
        RCLCPP_INFO(this->get_logger(),"Get online calibration from %s ok.", this->dev_ip_.c_str());
    }
  }
}

}  // namespace zvision__pointcloud
