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
#include "tools/tools.h"

namespace zvision_lidar_rawdata
{
RawData::RawData()
{
  this->cal_init_ok_ = false;
  this->use_lidar_time_ = false; /* default is local timestamp */
  this->cal_.reset(new zvision::CalibrationData());
  this->cal_lut_.reset(new zvision::PointCalibrationTable());
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
void RawData::loadConfigFile(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  std::string anglePath;
  std::string model;
  double xt, yt, zt, xr, yr, zr;
  private_nh.param("use_lidar_time", this->use_lidar_time_, false);/*default is local timestamp*/
  private_nh.param("angle_path", anglePath, std::string(""));/*angle file *.cal*/
  private_nh.param("model", model, std::string("ML30S-A1"));/*device type*/
  private_nh.param("device_ip", this->dev_ip_, std::string(""));/*device ip*/

  if(model == std::string("ML30B1"))
      device_type_ = zvision::ML30B1;
  else if(model == std::string("ML30SA1"))
      device_type_ = zvision::ML30SA1;
  else if(model == std::string("MLX"))
      device_type_ = zvision::MLX;
  else
      device_type_ = zvision::Unknown;

  //private_nh.param("model", model, std::string("ML30S-A1"));/*device type*/
  private_nh.param("x_tra", xt, 0.0);/*x trans*/
  private_nh.param("y_tra", yt, 0.0);/*y trans*/
  private_nh.param("z_tra", zt, 0.0);/*z trans*/
  private_nh.param("x_rot", xr, 0.0);/*x rotation*/
  private_nh.param("y_rot", yr, 0.0);/*y rotation*/
  private_nh.param("z_rot", zr, 0.0);/*z rotation*/

  this->x_trans = xt;
  this->y_trans = yt;
  this->z_trans = zt;
  this->x_rotation = xr / 180.0 * M_PI;;
  this->y_rotation = yr / 180.0 * M_PI;;
  this->z_rotation = zr / 180.0 * M_PI;;

  ROS_INFO("Coordinate transformation\n xt:%6.3f yt:%6.3f zt:%6.3f\n xr:%6.3f yr:%6.3f zr:%6.3f",
           xt, yt, zt, xr, yr, zr);

  ROS_INFO("Using lidar time [%d]\n", this->use_lidar_time_);


  if(!this->cal_)
      this->cal_.reset(new zvision::CalibrationData());

  if(!this->cal_lut_)
      this->cal_lut_.reset(new zvision::PointCalibrationTable());

  if(!anglePath.empty())
  {
      //read calibration file
      if(0 != zvision::LidarTools::ReadCalibrationFile(anglePath, *(this->cal_.get())))
      {
          ROS_ERROR("Read calibration file error, %s", anglePath);
      }

      //compute sin cos
      zvision::LidarTools::ComputeCalibrationData(*(this->cal_.get()), *(this->cal_lut_.get()));
      if((device_type_ == zvision::Unknown) || (device_type_ != this->cal_lut_->model))
          ROS_ERROR("Device %s's calibration not matched, device type is %s, cal type is %s", this->dev_ip_.c_str(), zvision::LidarTools::GetDeviceTypeString(device_type_).c_str(),
                zvision::LidarTools::GetDeviceTypeString(this->cal_lut_->model).c_str());
      else
      {
          ROS_INFO("Load device %s's calibration file ok, device type is %s", this->dev_ip_.c_str(), zvision::LidarTools::GetDeviceTypeString(device_type_).c_str());
          this->cal_init_ok_ = true;
      }
  }
  else
  {
      // raw data output topic
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
void RawData::unpack(const zvision_lidar_msgs::zvisionLidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
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
void RawData::getTimeStampFromUdpPkt(const zvision_lidar_msgs::zvisionLidarPacket& pkt, long long int& unix_sec, long long int& unix_microsec)
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
        tm_.tm_mon = data[1304 - 20 + 1];
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
  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if(this->cal_init_ok_)
        break;
    zvision::CalibrationData cal;
    int ret = 0;
    if(0 != (ret = zvision::LidarTools::GetOnlineCalibrationData( this->dev_ip_, cal)))
        ROS_WARN("Unable to get online calibration from %s.", this->dev_ip_.c_str());
    else {
        zvision::LidarTools::ComputeCalibrationData(cal, *(cal_lut_.get()));
        this->cal_init_ok_ = true;
        ROS_INFO("Get online calibration from %s ok.", this->dev_ip_.c_str());
    }
  }
}

}  // namespace zvision__pointcloud
