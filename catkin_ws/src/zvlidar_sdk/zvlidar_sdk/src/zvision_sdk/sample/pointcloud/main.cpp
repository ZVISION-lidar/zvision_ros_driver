// MIT License
//
// Copyright(c) 2019 ZVISION. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifdef USING_PCL_VISUALIZATION
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#endif


#include "print.h"
#include "lidar_tools.h"
#include "point_cloud.h"
#include "loguru.hpp"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <map>
#include <boost/filesystem.hpp>

class ParamResolver
{
public:

    static int GetParameters(int argc, char* argv[], std::map<std::string, std::string>& paras, std::string& appname)
    {
        paras.clear();
        if (argc >= 1)
            appname = std::string(argv[0]);

        std::string key;
        std::string value;
        for (int i = 1; i < argc; i++)
        {
            std::string str(argv[i]);
            if ((str.size() > 1) && ('-' == str[0]))
            {
                key = str;
                if (i == (argc - 1))
                    value = "";
                else
                {
                    value = std::string(argv[i + 1]);
                    if ('-' == value[0])
                    {
                        value = "";
                    }
                    else
                    {
                        i++;
                    }
                }
                paras[key] = value;
            }
        }
        return 0;
    }

};

#ifdef USING_PCL_VISUALIZATION
/*to pcl pointcloud*/
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_convert(zvision::PointCloud& in_point)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (auto& p : in_point.points)
    {
        pcl::PointXYZRGBA pcl_p;
        pcl_p.x = p.x;
        pcl_p.y = p.y;
        pcl_p.z = p.z;
        pcl_p.r = p.reflectivity % 255;
        pcl_p.g = p.reflectivity % 255;
        pcl_p.b = 255 - (p.reflectivity % 255);
        pcl_p.a = 255;
        cloud->push_back(pcl_p);
    }

    return cloud;
}

#endif

struct PlayParam {
    /** bref
    param: ip                       lidar ipaddress
    param: port                     lidar pointcloud udp destination port
    param: calibration              (optional) calibration filename, if pcapfilename is empty, online cal will be used.
    param: mc_enable                enable to join multicast group, if enable, mc_ip will be used.
    param: mcg_ip                   multicast group ip address.If you dont't known the mc_ip, set to "" , we get the mc_ip by tcp connection.
    param: tp                       lidar type
    param: mode                     none, 1 / 2 or 1 / 4, optional, only valid for ScanML30SA1_160.
    param: downsample_cfg_file      downsample config file path, optional, only valid for ScanML30SA1_160.
    */
    PlayParam(std::string ip, int port, std::string calibration, bool mc_enable, std::string mcg_ip, zvision::DeviceType tp, zvision::DownSampleMode mode, std::string downsample_cfg_file)
        :lidar_ip_(ip),
        port_(port),
        calfilename_(calibration),
        mc_en_(mc_enable),
        mc_ip_(mcg_ip),
        tp_(tp),
        downsample_(mode),
        downsample_cfg_file_(downsample_cfg_file)
    {}

    PlayParam() {
        lidar_ip_ = "192.168.10.108";
        port_ = 2368;
        calfilename_ = "";
        mc_en_ = false;
        mc_ip_ = "";
        tp_ = zvision::DeviceType::LidarUnknown;
        downsample_ = zvision::DownSampleMode::DownsampleUnknown;
        downsample_cfg_file_ = "";
    }

    std::string lidar_ip_;
    int port_;
    std::string calfilename_;
    bool mc_en_;
    std::string mc_ip_;
    zvision::DeviceType tp_;
    zvision::DownSampleMode downsample_;
    std::string downsample_cfg_file_;
};

//Pointcloud callback function.
void sample_pointcloud_callback(zvision::PointCloud& pc, int& status)
{
     LOG_F(2, "PointCloud callback, size %ld status %d.", pc.points.size(), status);
}

//sample 0 : get online pointcloud. You can get poincloud from online device.
//parameter param   lidar play parameters
void sample_online_pointcloud(const PlayParam& param)
{
    
    //Step 1 : Init a online player.
    //If you want to specify the calibration file for the pointcloud, cal_filename is used to load the calibtation data.
    //Otherwise, the PointCloudProducer will connect to lidar and get the calibtation data by tcp connection.
    zvision::PointCloudProducer player(param.port_, param.lidar_ip_, param.calfilename_, param.mc_en_, param.mc_ip_, param.tp_);

    // manu set downsample mode if necessary
    player.SetDownsampleMode(param.downsample_, param.downsample_cfg_file_);

    //Step 2 (Optioncal): Regist a callback function.
    //If a callback function registered, the callback function will be called when a new pointcloud is ready.
    //Otherwise, you can call PointCloudProducer's member function "GetPointCloud" to get the pointcloud.
    player.RegisterPointCloudCallback(sample_pointcloud_callback);

    // we need blooming frame?
    bool use_lidar_time = false;
    player.SetProcessInternalPacketsEnable(true, use_lidar_time);
    //Step 3 : Start to receive the pointcloud packet and process it.
    int ret = player.Start();
#ifdef USING_PCL_VISUALIZATION
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer.reset(new pcl::visualization::PCLVisualizer("cloudviewtest"));
#endif

    while (1)
    {
        int ret = 0;
        zvision::PointCloud cloud;

        //Step 3 : Wait the pointcloud for 200 ms. this function return when get poincloud ok or timeout. 
        if (ret = player.GetPointCloud(cloud, 20000))
             LOG_F(ERROR, "GetPointCloud error, ret = %d.", ret);
        else
        {
             LOG_F(2, "GetPointCloud ok.");
#ifdef USING_PCL_VISUALIZATION

             if (cloud.use_blooming)
             {
                 if (use_lidar_time)
                     LOG_F(2, "use_lidar_time:[%d], matched: %f, %f ", use_lidar_time, cloud.timestamp, cloud.blooming_frame->timestamp);
                 else
                     LOG_F(2, "use_lidar_time:[%d], matched: %f, %f ", use_lidar_time, cloud.sys_stamp, cloud.blooming_frame->sys_stamp);
             }

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  pcl_cloud = point_cloud_convert(cloud);
            if (!(viewer->updatePointCloud(pcl_cloud, "cloud")))
            {
                viewer->addPointCloud(pcl_cloud, "cloud", 0);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
            }
            viewer->spinOnce(10);
#endif
        }
    }
    getchar();
}

//sample 1 : get offline pointcloud. You can get poincloud from pcap file.
//parameter lidar_ip                lidar ipaddress
//parameter port                    lidar pointcloud udp destination port
//parameter calfilename             calibration filename
//parameter pcapfilename(required)  pcap filename
void sample_offline_pointcloud(std::string lidar_ip = "192.168.10.108", int port = 2368, std::string calfilename = "", std::string pcapfilename = "")
{
    int ret = 0;
    //Step 1 : Specify a pcap file, which contain the lidar's pointcloud packet.
    std::string pcap_filename = pcapfilename;

    //Specify a calibration file, which contain the lidar's calibration data.
    std::string cal_filename = calfilename;

    //Step 2 : Specify the pcap file, calibtation file to play.
    //The ip address and udp destination port is used to filter the pcap file to play the special lidar data.
    zvision::OfflinePointCloudProducer player(pcap_filename, cal_filename,
                                              lidar_ip, port);

    // auto detect blooming frame
    bool use_lidar_time = false;
    player.SetInternalFrameMatchMethod(use_lidar_time);

    int size = 0;
    zvision::DeviceType type = zvision::LidarMl30SpB1IS1;
    
    //Step 3 : Read pointcloud info from file.
    if (ret = player.GetPointCloudInfo(size, type))
    {
         LOG_F(ERROR, "OfflinePointCloudProducer GetPointCloudInfo failed, ret = %d.", ret);
    }
    else
    {
         LOG_F(2, "OfflinePointCloudProducer GetPointCloudInfo ok, count is %d, type is %d.", size, type);
        
        if (0 == size)
        {
             LOG_F(ERROR, "No pointcloud found for lidar %s:%d.", lidar_ip.c_str(), port);
            return;
        }
#ifdef USING_PCL_VISUALIZATION
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer.reset(new pcl::visualization::PCLVisualizer("cloudviewtest"));
#endif
        //Step 4: Iterate the pointcloud.
        while (1)
        {
            for (int i = 0; i < size-1; ++i)
            {
                zvision::PointCloud pointcloud;
                if (ret = player.GetPointCloud(i, pointcloud))
                {
                     LOG_F(ERROR, "GetPointCloud error, frame number is %d, ret = %d.", i, ret);
                }
                else
                {
                    int point_valid = 0;
                    for (auto& n : pointcloud.points)
                    {
                        if (n.valid)
                            point_valid++;
                    }
                     LOG_F(2, "GetPointCloud ok, frame number is %d, valid points %d.", i, point_valid);
                     if (pointcloud.use_blooming)
                     {
                         if (use_lidar_time)
                             LOG_F(2, "use_lidar_time:[%d], matched: %f, %f ", use_lidar_time, pointcloud.timestamp, pointcloud.blooming_frame->timestamp);
                         else
                             LOG_F(2, "use_lidar_time:[%d], matched: %f, %f ", use_lidar_time, pointcloud.sys_stamp, pointcloud.blooming_frame->sys_stamp);
                     }
    #ifdef USING_PCL_VISUALIZATION
                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  pcl_cloud = point_cloud_convert(pointcloud);
                    if (!(viewer->updatePointCloud(pcl_cloud, "cloud")))
                    {
                        viewer->addPointCloud(pcl_cloud, "cloud", 0);
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                    }
                    viewer->spinOnce(50);
    #endif
                }
            }
        }
    }
    // getchar();
}

static void export_point_cloud(zvision::PointCloud& points, std::string filename)
{
    std::ofstream out(filename, std::ios::binary);
    if (out.is_open())
    {
        out.setf(std::ios::fixed, std::ios::floatfield);
        out.precision(3);
        for (int i = 0; i < points.points.size(); ++i)
        {
            zvision::Point pt = points.points[i];
            out << pt.x << " " << pt.y << " " << pt.z << " " << std::endl;
        }
        out.close();
    }
}

#ifdef USING_PCL_VISUALIZATION
// get filepath in dir
bool GetFiles(std::vector<std::string>& paths, const std::string& root)
{
	try
	{
		paths.clear();
		boost::filesystem::path path(root);
		for (const auto& iter : boost::filesystem::directory_iterator(path))
		{
			if (boost::filesystem::is_directory(iter.path()))
				continue;

			std::string p = iter.path().string().c_str();
			if (boost::filesystem::path(p).extension().string().compare(".cal") == 0) {
				paths.push_back(p);
			}
		}
		return true;
	}
	catch (const std::exception& error)
	{
		std::string sError = error.what();
	}
	return false;
}

// convert pcap to pcd
void sample_offline2pcd(std::string lidar_ip = "192.168.10.108", int port = 2368, std::string calidir = "", std::string pcapfilename = "")
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer.reset(new pcl::visualization::PCLVisualizer("cloudviewtest"));
	// get cali files
	std::vector<std::string> calis;
	if (!GetFiles(calis, calidir)) {
        LOG_F(ERROR, "Get calibration files failed.");
		return;
	}

	int ret = 0;
	std::string pcap_filename = pcapfilename;
	// for per califile, get pointcloud
	for (auto cali : calis) {
        LOG_F(2, "For cali file:  %s.", cali.c_str());
		std::string cal_filename = cali;
		zvision::OfflinePointCloudProducer player(pcap_filename, cal_filename, lidar_ip, port);
		int size = 0;
		zvision::DeviceType type = zvision::LidarUnknown;
		if (ret = player.GetPointCloudInfo(size, type))
		{
			 LOG_F(ERROR, "OfflinePointCloudProducer GetPointCloudInfo failed, ret = %d.", ret);
		}
		else
		{
            
			if (0 == size)
			{
				 LOG_F(ERROR, "No pointcloud found for lidar %s:%d, calibration file:%s.", lidar_ip.c_str(), port, cali.c_str());
				continue;
			}
            
			for (int i = 0; i < size; ++i)
			{
				zvision::PointCloud pointcloud;
				if (ret = player.GetPointCloud(i, pointcloud))
				{
					 LOG_F(ERROR, "  GetPointCloud error, frame number is %d, ret = %d.", i, ret);
				}
				else
				{
					pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  pcl_cloud = point_cloud_convert(pointcloud);
					if (!(viewer->updatePointCloud(pcl_cloud, "cloud")))
					{
						viewer->addPointCloud(pcl_cloud, "cloud", 0);
						viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
					}
					viewer->spinOnce(50);
					// now we save current frame to pcd
					pcl::PointCloud<pcl::PointXYZI> cloud;
					cloud.resize(pointcloud.points.size());
					int id = 0;
					for (auto p : pointcloud.points) {
						pcl::PointXYZI pp;
						pp.x = p.x;
						pp.y = p.y;
						pp.z = p.z;
						pp.intensity = p.reflectivity;
						cloud.at(id) = pp;
						id++;
					}
					// save dir
					std::string save_path = boost::filesystem::path(pcap_filename).parent_path().string();
					save_path = save_path + "/" + boost::filesystem::basename(pcap_filename);
					if (!boost::filesystem::is_directory(save_path))
						boost::filesystem::create_directory(save_path);

					std::string pcd_path = save_path + "/" + boost::filesystem::basename(cal_filename) + "_" + std::to_string(i) + ".pcd";
                    LOG_F(2, "  save frame[%d]`s pcd to -->  %s.", i, pcd_path.c_str());
					pcl::io::savePCDFileASCII(pcd_path, cloud);
				}
			}
		}
	}
    LOG_F(2, "Done!");
}
#endif

void init_log(int argc, char* argv[])
{
    //loguru::init();
    //loguru::add_file("Log/log.txt", loguru::Truncate, loguru::Verbosity_INFO);
    loguru::g_stderr_verbosity = 0;
    loguru::g_preamble_date = 0;
    loguru::g_preamble_time = 0;
    loguru::g_preamble_uptime = 0;
    loguru::g_preamble_thread = 0;
    loguru::g_preamble_file = 0;
    loguru::g_preamble_verbose = 0;
    loguru::g_preamble_pipe = 0;
}

int main(int argc, char** argv)
{
    
    init_log(argc, argv);
    using Param = std::map<std::string, std::string>;
    std::map<std::string, std::string> paras;
    std::string appname = "";
    ParamResolver::GetParameters(argc, argv, paras, appname);

    Param::iterator online = paras.find("-online");
    Param::iterator offline = paras.find("-offline");
	Param::iterator offline2pcd = paras.find("-offline2pcd");
    
    zvision::set_ml30splus_b1_ep_mode_enable(false);
    if (online != paras.end())// play online sensor
    {
        Param::iterator find = paras.find("-ip");// device ip
        if (find != paras.end())
        {
            std::string ip = find->second;
            std::string calibration = "";
            bool mc_enable = false;
            std::string mcg_ip = "";
            std::string downsample_str = "none";
            std::string downsample_cfg_file = "";
            int port = -1;
			zvision::DeviceType tp = zvision::DeviceType::LidarUnknown;
            if (paras.end() != (find = paras.find("-p")))// pointcloud udp port
            {
                port = std::atoi(find->second.c_str());
            }
            if (paras.end() != (find = paras.find("-c")))// calibration file
            {
                calibration = find->second;
            }
            if (paras.end() != (find = paras.find("-j")))// join multicast group
            {
                mc_enable = true;
            }
            if (paras.end() != (find = paras.find("-g")))// multicast group ip address
            {
                mcg_ip = find->second;
            }
			if (paras.end() != (find = paras.find("-30sp"))) {
				tp = zvision::DeviceType::LidarMl30SA1Plus;
			}
            if (paras.end() != (find = paras.find("-30spb1"))) {
                tp = zvision::DeviceType::LidarMl30SB1Plus;
            }
            if (paras.end() != (find = paras.find("-30spb2"))) {
                tp = zvision::DeviceType::LidarMl30SB2Plus;
            }
            if (paras.end() != (find = paras.find("-30spb1ep1")))
            {
                tp = zvision::DeviceType::LidarMl30SB1Plus;
                zvision::set_ml30splus_b1_ep_mode_enable(true);
            }
            if(paras.end() != (find = paras.find("-30spb1IS1"))){
                tp = zvision::DeviceType::LidarMl30SpB1IS1;
                zvision::set_ml30splus_b1_ep_mode_enable(true);
            }
            if (paras.end() != (find = paras.find("-30spb2ep1"))) {
                tp = zvision::DeviceType::LidarMl30SB2Plus;
                zvision::set_ml30splus_b1_ep_mode_enable(true);
            }
            if (paras.end() != (find = paras.find("-ex"))) {
                tp = zvision::DeviceType::LidarExciton;
            }
            if (paras.end() != (find = paras.find("-d")))// downsample mode
            {
                downsample_str = find->second;
            }
            if (paras.end() != (find = paras.find("-dcfg")))// downsample config file path
            {
                downsample_cfg_file = find->second;
            }
            

            zvision::DownsampleMode mode = zvision::DownsampleMode::DownsampleNone;
            if (downsample_str.compare("1/2") == 0)
                mode = zvision::DownsampleMode::Downsample_1_2;
            else if (downsample_str.compare("1/4") == 0)
                mode = zvision::DownsampleMode::Downsample_1_4;
            else if (!downsample_cfg_file.empty())
                mode = zvision::DownsampleMode::Downsample_cfg_file;

            PlayParam param(ip, port, calibration, mc_enable, mcg_ip, tp, mode, downsample_cfg_file);

            sample_online_pointcloud(param);
            return 0;
        }
        else
        {
             LOG_F(ERROR, "Invalid parameters, no device ip address found.");
        }
    }   
    else if (offline != paras.end())// play online sensor
    {
        Param::iterator ip_find = paras.find("-ip");// device ip
        Param::iterator port_find = paras.find("-p");// pointcloud udp port
        Param::iterator pcap_find = paras.find("-f");// pointcloud udp port
        Param::iterator calibration_find = paras.find("-c");// pointcloud udp port
        if (paras.end() != paras.find("-30spb1ep1"))      // for 30s+ ep1 mode 
        {
            zvision::set_ml30splus_b1_ep_mode_enable(true);
        }
        if (paras.end() != paras.find("-30spb1IS1"))
        {
            zvision::set_ml30splus_b1_ep_mode_enable(true);
        }

        if ((ip_find != paras.end()) && (port_find != paras.end()) && (pcap_find != paras.end()) && (calibration_find != paras.end()))
        {
            std::string ip = ip_find->second;
            int port = std::atoi(port_find->second.c_str());
            std::string calibration = calibration_find->second;
            std::string filename = pcap_find->second;
            sample_offline_pointcloud(ip, port, calibration, filename);
            return 0;
        }
        else
        {
            if(ip_find == paras.end())
                 LOG_F(ERROR, "Invalid parameters, device ip address not found.");
            if (port_find == paras.end())
                 LOG_F(ERROR, "Invalid parameters, port not found.");
            if (pcap_find == paras.end())
                 LOG_F(ERROR, "Invalid parameters, filename not found.");
            if (calibration_find == paras.end())
                 LOG_F(ERROR, "Invalid parameters, calibration file name not found.");
        }
    }
#ifdef USING_PCL_VISUALIZATION
	else if (offline2pcd != paras.end())
	{
		Param::iterator ip_find = paras.find("-ip");// device ip
		Param::iterator port_find = paras.find("-p");// device udp port
		Param::iterator pcap_find = paras.find("-f");// pcap file
		Param::iterator calidir = paras.find("-cali_dir");// cali file
        if (paras.end() != paras.find("-30spb1ep1"))      // for 30s+ ep1 mode 
        {
            zvision::set_ml30splus_b1_ep_mode_enable(true);
        }

		if ((ip_find != paras.end()) && (port_find != paras.end()) && (pcap_find != paras.end()) && (calidir != paras.end()))
		{

			std::string ip = ip_find->second;
			int port = std::atoi(port_find->second.c_str());
			std::string calipath = calidir->second;
			std::string filename = pcap_find->second;
			sample_offline2pcd(ip, port, calipath, filename);
			return 0;
		}
		else
		{
			if (ip_find == paras.end())
				 LOG_F(ERROR, "Invalid parameters, device ip address not found.");
			if (port_find == paras.end())
				 LOG_F(ERROR, "Invalid parameters, port not found.");
			if (pcap_find == paras.end())
				 LOG_F(ERROR, "Invalid parameters, filename not found.");
			if (calidir == paras.end())
				 LOG_F(ERROR, "Invalid parameters, calibration file name not found.");
		}
	}
#endif

        std::cout
            << "############################# USER GUIDE "
               "################################\n\n"
            << "Online sample param:\n"
            << "        -online (required)\n"
            << "        -ip lidar_ip_address(required)\n"
            << "        -p  pointcloud_udp_port(optional)\n"
            << "        -c  calibration_file_name(optional)\n"
            << "        -j  (optional for online)\n"
            << "        -g  multicast_group_ip_address(optional, valid when -j "
               "is set)\n"
            << "        -30sp (optional, special for ML30S+ lidar)\n"
            << "        -30spb1 (optional, special for ML30S+B1 lidar)\n"
            << "        -30spb1ep1 (optional, special for ML30S+B1 EP1 lidar)\n"
            << "        -30spb1IS1 (optional, special for ML30S+B1 IS1 lidar)\n"
            << "        -ex (optional, special for EXCITON lidar)\n"
            << "        -d  downsample mode(none, 1/2 or 1/4, optional,only "
               "valid for ScanML30SA1_160)\n"
            << "        -dcfg  downsample mode(downsample config file path, "
               "optional, only valid for ScanML30SA1_160)\n"
            << "\n"
            << "Online sample 1 : -online -ip 192.168.10.108\n"
            << "Online sample 2 : -online -ip 192.168.10.108 -30sp\n"
            << "Online sample 3 : -online -ip 192.168.10.108 -30spb1\n"
            << "Online sample 4 : -online -ip 192.168.10.108 -30spb1ep1\n"
            << "Online sample 5 : -online -ip 192.168.10.108 -p 2368\n"
            << "Online sample 6 : -online -ip 192.168.10.108 -c xxxx.cal\n"
            << "Online sample 7 : -online -ip 192.168.10.108 -p 2368 -c "
               "xxxx.cal\n"
            << "Online sample 8 : -online -ip 192.168.10.108 -j\n"
            << "Online sample 9 : -online -ip 192.168.10.108 -j 239.0.0.1\n"
            << "Online sample 10 : -online -ip 192.168.10.108 -d 1/2\n"
            << "Online sample 11 : -online -ip 192.168.10.108 -dcfg xxxx.txt\n"
            << "Online sample 12 : -online -ip 192.168.19.177 -p 2368 -30spb1IS1 -j -g 225.5.5.177\n"
            << "\n"

            << "Offline sample param:\n"
            << "        -offline (required)\n"
            << "        -ip lidar_ip_address(required)\n"
            << "        -p  pointcloud_udp_port(required)\n"
            << "        -f  pcap_file_name(required)\n"
            << "        -c  calibration_file_name(required)\n"
            << "        -30spb1ep1 (optional, special for ML30S+B1 EP1 lidar)\n"
            << "        -30spb1IS1 (optional, special for ML30S+B1 IS1 lidar)\n"
            << "\n"
            << "Offline sample 1 : -offline -ip 192.168.10.108 -p 2368 -f "
               "xxxx.pcap -c xxxx.cal\n"
            << "Offline sample 2 (cal in pcap): -offline -ip 192.168.10.108 -p "
               "2368 -f xxxx.pcap -c \"\"\n"
            << "Offline sample 3 : -offline -ip 192.168.10.108 -p 2368 -f "
               "xxxx.pcap -c xxxx.cal -30spb1ep1\n"
                << "Offline sample 4 : -offline -ip 192.168.10.108 -p 2368 -f "
               "xxxx.pcap -c xxxx.cal -30spb1IS1\n"

#ifdef USING_PCL_VISUALIZATION
            << "\n"
            << "Offline pcap to pcd sample param:\n"
            << "        -offline2pcd (required)\n"
            << "        -ip lidar_ip_address(required)\n"
            << "        -p  pointcloud_udp_port(required)\n"
            << "        -f  pcap_file_name(required)\n"
            << "        -cali_dir  calibration_file_name(required)\n"
            << "        -30spb1ep1 (optional, special for ML30S+B1 EP1 lidar)\n"
            << "\n"
            << "Offline sample 1 : -offline2pcd -ip 192.168.10.108 -p 2368 -f "
               "xxxx.pcap -cali_dir /home/data/cali\n"
            << "Offline sample 2 : -offline2pcd -ip 192.168.10.108 -p 2368 -f "
               "xxxx.pcap -cali_dir /home/data/cali -30spb1ep1\n"

#endif

            << "############################# END  GUIDE "
               "################################\n\n";
    getchar();

    return zvision::InvalidParameter; 
}
