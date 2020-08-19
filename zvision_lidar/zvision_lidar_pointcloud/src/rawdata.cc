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

namespace zvision_lidar_rawdata
{
RawData::RawData()
{
  this->is_init_angle_ = false;
  timestamp_type = 0; /* default is local timestamp */
  pdElevationAngle = NULL;
  pdAzimuthAngle = NULL;
  x_trans = 0.0;
  y_trans = 0.0;
  z_trans = 0.0;
  x_rotation = 0.0;
  y_rotation = 0.0;
  z_rotation = 0.0;
}
RawData::~RawData()
{
    delete pdElevationAngle;
    delete pdAzimuthAngle;
}
//import cali
void RawData::loadConfigFile(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  std::string anglePath;
  std::string model;
  double xt, yt, zt, xr, yr, zr;
  private_nh.param("timestamp_type", timestamp_type, 0);/*default is local timestamp*/
  private_nh.param("angle_path", anglePath, std::string(""));/*angle file *.cal*/
  private_nh.param("model", model, std::string("ML30S-A1"));/*device type*/
  if(model == std::string("ML30"))
	  type = LIDAR_TYPE::ML30;
  else
  	  type = LIDAR_TYPE::ML30S_A1;

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

  ROS_INFO("Timestamp Type[%d]:0 local time, 1 udp gps time\n", timestamp_type);

  if (model == "ML30S-A1")//ML30S-A1
  {
    if(!pdElevationAngle)
    {
      pdElevationAngle = new double[51200];/*320 * 80 * 2 */
    }
    if(!pdAzimuthAngle)
    {
      pdAzimuthAngle = new double[51200];
    }
  }
  else if(model == "ML30"){
      if(!pdElevationAngle)
      {
          pdElevationAngle = new double[30000];/*10000 * 3*/
      }
      if(!pdAzimuthAngle)
      {
          pdAzimuthAngle = new double[30000];
      }
  }
  else
  {
    if(!pdElevationAngle)
    {
      pdElevationAngle = new double[30000];
    }
    if(!pdAzimuthAngle)
    {
      pdAzimuthAngle = new double[30000];
    }
  }

  //=============================================================
  FILE* f_angle = fopen(anglePath.c_str(), "r");
  if (!f_angle)
  {
    ROS_ERROR_STREAM(anglePath << " does not exist angle file");
  }
  else
  {
      if(model == "ML30")
      {
          int group_id = 0;
          double  ath0, ath1, ath2, ele0,ele1, ele2;
          for(int it = 0; it<10000; it++){
              int tmp = fscanf(f_angle, "%d %lf %lf %lf %lf %lf %lf\n", &group_id, &ath0, &ath1, &ath2, &ele0, &ele1, &ele2);
              if(7 != tmp){
                  printf("Load Angle FIle Error, current group :%d!\r\n", it);
                  break;
              }
              else{

                  pdAzimuthAngle[it * 3 + 0] = ath0 / 180.0 * M_PI;
                  pdAzimuthAngle[it * 3 + 1] = ath1 / 180.0 * M_PI;
                  pdAzimuthAngle[it * 3 + 2] = ath2 / 180.0 * M_PI;

                  pdElevationAngle[it * 3 + 0] = ele0 / 180.0 * M_PI;
                  pdElevationAngle[it * 3 + 1] = ele1 / 180.0 * M_PI;
                  pdElevationAngle[it * 3 + 2] = ele2 / 180.0 * M_PI;

              }
          }
          fclose(f_angle);
      }
      if(model == "ML30S-A1")
      {
          int group_id = 0;
          double ath0, ath1, ath2, ath3, ath4, ath5, ath6, ath7, ele0, ele1, ele2, ele3, ele4, ele5, ele6, ele7;
          for(int it = 0; it < 6400; it++){
              int tmp = fscanf(f_angle, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                               &group_id, &ath0, &ele0, &ath2, &ele2, &ath4, &ele4, &ath6, &ele6, &ath5, &ele5,
                               &ath7, &ele7, &ath1, &ele1, &ath3, &ele3);
              if(17 != tmp)
              {
                  printf("Load Angle FIle Error, current group :%d!\r\n", it);
                  break;
              }
              else
              {

                  pdAzimuthAngle[it * 8 + 0] = ath0 / 180.0 * M_PI;
                  pdAzimuthAngle[it * 8 + 1] = ath1 / 180.0 * M_PI;
                  pdAzimuthAngle[it * 8 + 2] = ath2 / 180.0 * M_PI;
                  pdAzimuthAngle[it * 8 + 3] = ath3 / 180.0 * M_PI;
                  pdAzimuthAngle[it * 8 + 4] = ath4 / 180.0 * M_PI;
                  pdAzimuthAngle[it * 8 + 5] = ath5 / 180.0 * M_PI;
                  pdAzimuthAngle[it * 8 + 6] = ath6 / 180.0 * M_PI;
                  pdAzimuthAngle[it * 8 + 7] = ath7/ 180.0 * M_PI;


                  pdElevationAngle[it * 8 + 0] = ele0 / 180.0 * M_PI;
                  pdElevationAngle[it * 8 + 1] = ele1 / 180.0 * M_PI;
                  pdElevationAngle[it * 8 + 2] = ele2 / 180.0 * M_PI;
                  pdElevationAngle[it * 8 + 3] = ele3 / 180.0 * M_PI;
                  pdElevationAngle[it * 8 + 4] = ele4 / 180.0 * M_PI;
                  pdElevationAngle[it * 8 + 5] = ele5 / 180.0 * M_PI;
                  pdElevationAngle[it * 8 + 6] = ele6 / 180.0 * M_PI;
                  pdElevationAngle[it * 8 + 7] = ele7 / 180.0 * M_PI;
              }
          }
          fclose(f_angle);
      }
  }
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


  if(type == LIDAR_TYPE::ML30S_A1)
  {
	  for (int group = 0; group < GROUP_PER_PACKET; group++)   // 1 packet:40 data group
	  {
		  const unsigned char *pc = data + 4 + group * 8 * 4;/*group address*/
		  int index = udp_seq * GROUP_PER_PACKET + group;/*group number, to search the angle file *.cal*/
		  for (int laser_id = 0; laser_id < POINT_PER_GROUP; laser_id++)/*8 Points per laser*/
		  {
			  double azi = pdAzimuthAngle[index * POINT_PER_GROUP + laser_id];/*Ath data from angle file*/
			  double ele = pdElevationAngle[index * POINT_PER_GROUP + laser_id];/*Ele data from angle file*/
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

			  point.x = distantce_real_live * cos(ele) * sin(azi);/*x*/
			  point.y = distantce_real_live * cos(ele) * cos(azi);/*y*/
			  point.z = distantce_real_live * sin(ele);/*z*/
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

			  pointcloud->at(laser_id * 6400 + index) = point;/*not filled when pkt loss*/
		  }
	  }
  }
  else if(type == LIDAR_TYPE::ML30)
  {
	  for (int group = 0; group < GROUP_PER_PACKET_ML30; group++)   // 1 packet:80 data group
	  {
		  const unsigned char *pc = data + 4 + group * 8 * 2;/*group address*/
		  int index = udp_seq * GROUP_PER_PACKET_ML30 + group;/*group number, to search the angle file *.cal*/

		  for (int laser_id = 0; laser_id < POINT_PER_GROUP_ML30; laser_id++)/*3 Points per laser*/
		  {
			  double azi = pdAzimuthAngle[index * POINT_PER_GROUP_ML30 + laser_id];/*Ath data from angle file*/
			  double ele = pdElevationAngle[index * POINT_PER_GROUP_ML30 + laser_id];/*Ele data from angle file*/
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

			  point.x = distantce_real_live * cos(ele) * sin(azi);/*x*/
			  point.y = distantce_real_live * cos(ele) * cos(azi);/*y*/
			  point.z = distantce_real_live * sin(ele);/*z*/
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

			  pointcloud->at(laser_id * 10000 + index) = point;/*not filled when pkt loss*/
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
	int Y = data[1304 - 20 + 0];
	int Mon = data[1304 - 20 + 1];
	int D = data[1304 - 20 + 2];
	int H = data[1304 - 20 + 3];
	int M = data[1304- 20 + 4];
	int S = data[1304 - 20 + 5];
	int MillS = (int)(data[1304 - 20 + 6] << 8) + data[1304 - 20 + 7];
	int MicroS = (int)(data[1304 - 20 + 8] << 8) + data[1304 - 20 + 9];

	struct tm t_;
	struct tm *t_gm_ = NULL;
	time_t unix_t_;
	t_.tm_year = 2000 + Y -1900;
	t_.tm_mon= Mon-1;
	t_.tm_mday=D;
	t_.tm_hour=H;
	t_.tm_min=M;
	t_.tm_sec=S;
	t_.tm_isdst=0;
	//unix_t_ = mktime(&t_);
	//t_gm_ = gmtime(&unix_t_);/*to gmt time*/
	//unix_sec = mktime(t_gm_);
	unix_sec = timegm(&t_);
	unix_microsec = MillS * 1000 + MicroS;
}

}  // namespace zvision__pointcloud
