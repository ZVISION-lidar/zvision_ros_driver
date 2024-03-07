#include"filter_pointcloud.h"
#include<fstream>
#include<lidar_tools.h>
#include<common/print.h>
#include "blooming.h"

std::shared_ptr<std::vector<std::vector<int>>> BloomingModel::nearest_point_table_ptr = nullptr;

namespace zv_processor{

    FilterPointcloud::FilterPointcloud(/* args */)
    {

    }
    
    FilterPointcloud::~FilterPointcloud()
    {
        
    }

    void FilterPointcloud::set_filter_param(const FilterParam& param)
    {
        param_ = param;
        // load downsample config file
        if(param_.filter_mode == FilterMode::ConfigFile)
            load_downsample_cfg_file(param_.downsample_cfg_path);
    }

    void FilterPointcloud::setCalibrationdata(const zvision::CalibrationDataSinCosTable&cal_lut )
    {
        cal_lut_ = cal_lut;
        compute_point_line_number(cal_lut_, point_line_number_);
    }

    void FilterPointcloud::run(const zvision::PointCloud& src, zvision::PointCloud&dst)
    {
        switch (param_.filter_mode)
        {
        case Voxel:
            ft_voxel(src, dst);
            break;
        case Line:
            ft_line(src, dst);
            break;
        case OutlierRemoval:
            ft_outlier_remove(src, dst);
            break;
        case ConfigFile:
            ft_by_config_file(src, dst);
            break;
        case Blooming:
            ft_blooming_remove(src, dst);
            break;
        default:
            break;
        }
    }

    void FilterPointcloud::ft_voxel(const zvision::PointCloud& src, zvision::PointCloud&dst)
    {
        // filter pointcloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ft(new pcl::PointCloud<pcl::PointXYZI>);
	    cloud_ft->height = 1;
        cloud_ft->is_dense = false;
	    cloud_ft->width = src.points.size();
        cloud_ft->resize(src.points.size());
        for(int i = 0; i < src.points.size(); i++)
        {
            cloud_ft->at(i).x = src.points.at(i).x;
            cloud_ft->at(i).y = src.points.at(i).y;
            cloud_ft->at(i).z = src.points.at(i).z;
            cloud_ft->at(i).intensity = src.points.at(i).reflectivity;
        }
        // destination pointcloud indices
        
        // set filter leaf size
        voxel_grid_filter_.setLeafSize(param_.voxel_leaf_size, param_.voxel_leaf_size, param_.voxel_leaf_size);
        // set source pointcloud
        voxel_grid_filter_.setInputCloud(cloud_ft);
        // get filtered pointclouds
        voxel_grid_filter_.filter(*cloud_ft);
        
        dst = src;
        if(!cloud_ft->size())
        {
            dst.points.clear();
            return;
        }

        // update dst pointcloud
        int dst_cnt = cloud_ft->size();
        dst.points.resize(dst_cnt);
        for(int i = 0;i<dst_cnt; i++)
        {
            dst.points.at(i) = zvision::Point();
            dst.points.at(i).x = cloud_ft->at(i).x;
            dst.points.at(i).y = cloud_ft->at(i).y;
            dst.points.at(i).z = cloud_ft->at(i).z;
            dst.points.at(i).reflectivity = cloud_ft->at(i).intensity;
        }
    }
    
    void FilterPointcloud::compute_point_line_number(const zvision::CalibrationDataSinCosTable& cal_lut, std::vector<int>& line_numbers)
    {
        int fovs = 8;
        switch (cal_lut.device_type)
        {
            case zvision::LidarML30B1:
            case zvision::LidarMLX:
            case zvision::LidarMLYA:
            case zvision::LidarMLYB:
                fovs = 3;
                break;
            case zvision::LidarML30SA1:
            case zvision::LidarML30SA1_2:
            case zvision::LidarML30SB1:
            case zvision::LidarML30SB2:
            case zvision::LidarMl30SA1Plus:
            case zvision::LidarMl30SB1Plus:
            case zvision::LidarMl30SA1Factory:
                fovs = 8;
                break;
        
        default:
            break;
        }

        line_numbers.resize(cal_lut.data.size(),0);
        for(int i = 0; i < fovs * 2;i++)
            line_numbers[i] = 0;

        const std::vector<zvision::CalibrationDataSinCos>& data = cal_lut.data;
        for(int f = 0; f < fovs; f++)
        {
            int curr_line = 0;
            for(int g = 2; g < data.size() / fovs; g++)
            {
                int point_id = g * fovs + f;
                float pre_dif = data[point_id - fovs].azi - data[point_id - fovs * 2].azi;
                float curr_dif = data[point_id].azi - data[point_id - fovs].azi;
                if((pre_dif * curr_dif) < 0.0)
                    curr_line++;

                line_numbers[point_id] = curr_line;
            }
        }
    }

    void FilterPointcloud::ft_line(const zvision::PointCloud& src, zvision::PointCloud& dst)
    {
        dst = src;
       // ignore
        if(point_line_number_.size() < src.points.size())
        {
            return;
        }

        // filter
        dst.points.resize(src.points.size());
        int valid = 0;
        for(int i = 0; i < src.points.size(); i++)
        {
            if(0 == (point_line_number_[i] % (param_.line_sample + 1))){
                dst.points.at(valid++) = src.points.at(i);
            }
        }
        dst.points.resize(valid);
    }

    /*  outlier remove functions*/
    /* get point neighbours id, the id is invalid if the value is less than zero*/
    std::vector<int>& FilterPointcloud::get_nearest_point_index()
    {
        const int near_cnt = 8;
        static std::vector<int> g_invalid;
        static std::vector<int> g_neighborhood_index_ml30s(near_cnt * 51200, -1);
        static std::vector<int> g_neighborhood_index_mlxs(near_cnt * 108000, -1);
        static bool g_is_initialized = false;
        
        if(g_is_initialized){

            // for ml30s
            if(cal_lut_.data.size()==51200)
            {
                return g_neighborhood_index_ml30s;
            }
            // for mlxs
            else if(cal_lut_.data.size()==108000)
            {
                return g_neighborhood_index_ml30s;
            }
            else
            {
                return g_invalid;
            }
        }

        // generate pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto& point_cal : cal_lut_.data)
        {
            pcl::_PointXYZ point_data;
            float distance = 2.0;
            point_data.x = distance * point_cal.cos_ele * point_cal.sin_azi;
            point_data.y = distance * point_cal.cos_ele * point_cal.cos_azi;
            point_data.z = distance * point_cal.sin_ele;
            cloud->push_back(point_data);
        }
        pcl::search::Search<pcl::PointXYZ>::Ptr tree_;

        tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
        tree_->setInputCloud(cloud);
        // get near points id
        std::vector<int> nn_indices(near_cnt);
        std::vector<float> nn_dists(near_cnt);
        int cnt = 0;
        if(cloud->points.size()==51200)
        {
            for (auto& pp : cloud->points)
            {
                int ret = tree_->nearestKSearch(pp, near_cnt, nn_indices, nn_dists);
                if ( ret <= near_cnt)
                {
                    for (int i = 0; i < ret; i++)
                    {
                        g_neighborhood_index_ml30s.at(cnt * near_cnt + i) = nn_indices[i];
                    }
                }
                else
                {
                    // ...
                }
                cnt++;
            }
            LOG_INFO("get_nearest_point_index  30s finished.\n");
            g_is_initialized = true;
            return g_neighborhood_index_ml30s;    
        }
        else if(cloud->points.size()==108000)
        {
            for (auto& pp : cloud->points)
            {
                int ret = tree_->nearestKSearch(pp, near_cnt, nn_indices, nn_dists);
                if ( ret<= near_cnt)
                {
                    for (int i = 0; i < ret; i++)
                    {
                        g_neighborhood_index_mlxs.at(cnt * near_cnt + i) = nn_indices[i];
                    }
                }
                else
                {
                    // ...
                }
                cnt++;
            }
            LOG_INFO("get_nearest_point_index xs finished.\n");
            g_is_initialized = true;
            return g_neighborhood_index_mlxs;    
        }

        g_is_initialized = true;
        return g_invalid;
    }

    void FilterPointcloud::ft_outlier_remove(const zvision::PointCloud& src, zvision::PointCloud&dst)
    {
        // check
        dst = src;
        int src_points = src.points.size();
        if (src_points != 108000 && src_points != 51200)
            return;

        std::vector<int>& nearest_table = get_nearest_point_index();
        if(!nearest_table.size())
            return;

        int valid = 0;
        double coef = powf(param_.outlier_removal, 2);
        for (size_t i = 0; i < src_points; ++i)
        {
            // skip invalid point
            if (src.points.at(i).valid == 0 || \
                src.points.at(i).distance < 1e-5)
            {
                continue;
            }

            // dynamic threshold judgment (outlier threshold square)
            size_t valid_neighbors = 0;
            auto ots = coef * (powf(src.points.at(i).distance, 2));
            for (size_t j = (i * 8); j < (i * 8 + 8); j++)
            {
                // get neighbour id from nearest_table
                int nbr = nearest_table.at(j);
                if(nbr < 0)
                    continue;

                if(src.points.at(i).x == std::numeric_limits<float>::quiet_NaN() || \
                    src.points.at(nbr).x == std::numeric_limits<float>::quiet_NaN()) 
                   continue;

                // calculate distance from pi to pj
                auto& pi = src.points.at(i);
                auto& pj = src.points.at(nbr);
                double dis = pow((pi.x-pj.x),2) + pow((pi.y-pj.y),2) + pow((pi.z-pj.z),2);
                if (dis < ots)
                {
                    valid_neighbors++;
                    if (valid_neighbors >= 2)
                    break;
                }
            }
            //  outlier judgment
            if (valid_neighbors >= 2)
            {
                dst.points.at(valid++) = src.points.at(i);
            }
        }
        dst.points.resize(valid);
    }

    /*  filter by config file functions*/
    /* get pointcloud down sample mask*/
    void FilterPointcloud::load_downsample_cfg_file(std::string cfg_path)
    {
		if (cfg_path.empty())
			return;

        // for ml30s device, update downsample mask
		try {
            // load cfg file data
			std::ifstream in(cfg_path.c_str(), std::ios::in);
			if (!in.is_open()){
                LOG_WARN("Open downsample file failed.");
                return;
            }
            downsample_mask_.reset( new std::vector<bool>(51200,true));
	        uint8_t flg = 0x80;
		    uint8_t fov_in_group[8] = {0, 2, 4, 6, 5, 7, 1, 3};
		    int total_cnt = 640;
		    std::string line;
		    int id = 0;
		    while (std::getline(in, line))
		    {
		    	// check enter
                if(line.at(line.size()-1) == 13){
                    line = line.substr(0,line.size()-1);
                }

		    	if (line.size() != 20)
		    		continue;

		    	// get value
		    	uint8_t masks[10] = {0xFF};
		    	for (int b = 0; b < 10;b++) {
		    		char h = line.at(b * 2);
		    		char l = line.at(b * 2 + 1);
		    		masks [b] = hex2uint8(h) << 4 | hex2uint8(l);
		    	}
			    // update
				for (int j = 0; j < 10; j++) {
					for (int k = 0; k < 8;k++) {
						flg = 0x80 >> k;
						if ((flg & masks[j]) != flg) {
							int fov = id / 80;
							int group = (id * 10 * 8 + j * 8 + k) % 6400;
							int point_id = group * 8 + fov_in_group[fov];
							downsample_mask_->at(point_id) = 0;
						}
					}
				}
				id++;
			}

			in.close();
			if (id != total_cnt) {
				downsample_mask_.reset();
			}
		}
		catch (std::exception e)
		{
			downsample_mask_.reset();
			return;
		}
	}

    void FilterPointcloud::ft_by_config_file(const zvision::PointCloud& src, zvision::PointCloud&dst)
    {
        dst = src;
        // only support for ml30s
        if(!downsample_mask_ || \
            (downsample_mask_->size() != src.points.size()) || \
            (src.scan_mode != zvision::ScanML30SA1_160))
            return;

        // filter pointcloud from config file
        int valid = 0;
        for(int i = 0; i<src.points.size(); i++){
            if(downsample_mask_->at(i)){
                dst.points.at(valid) = src.points.at(i);
                valid++;
            }
        }
         dst.points.resize(valid);
    }


    void FilterPointcloud::ft_blooming_remove(const zvision::PointCloud& src, zvision::PointCloud& dst)
    {
        dst = src;
        std::string yaml_path = std::string(PROJECT_PATH) + "/config/default.yaml";
        // std::string yaml_path = "/home/z/qiaodan/zvision_exciton/src/blooming/cfg/default.yaml";
        BloomingModel blooming_model(yaml_path);

        if (dst.points.size() < 76800){return;}
        blooming_model.main_process(dst.points);
        for (size_t i = 0; i < dst.points.size(); i++) {
            if (dst.points.at(i).bom_num >= 0)
            {
                dst.points.at(i).x = 0;
                dst.points.at(i).y = 0;
                dst.points.at(i).z = 0;
                dst.points.at(i).reflectivity = 0;
                dst.points.at(i).distance = 0;
            }
        }
    }

};

