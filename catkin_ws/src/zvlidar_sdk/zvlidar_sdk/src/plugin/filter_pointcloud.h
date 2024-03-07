#pragma once
#include <point_cloud.h>

#include <util/config.hpp>
#include <pcl/PointIndices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
namespace zv_processor
{
    
    class FilterPointcloud
    {
    public:
        FilterPointcloud(/* args */);
        ~FilterPointcloud();
        /* set filter parameter */
        void set_filter_param(const FilterParam& param);
        /* set cali data */
       void setCalibrationdata(const zvision::CalibrationDataSinCosTable&cal_lut );
        /* process */
        void run(const zvision::PointCloud& src, zvision::PointCloud&dst);
        
    protected:
        /* load downsample config file */
        void load_downsample_cfg_file(std::string cfg_path) ;
        /* voxel*/
        void ft_voxel(const zvision::PointCloud& src, zvision::PointCloud&dst);
        /* line*/
        void compute_point_line_number(const zvision::CalibrationDataSinCosTable& cal_lut, std::vector<int>& line_numbers);
        void ft_line(const zvision::PointCloud& src, zvision::PointCloud&dst);
        /* outlier*/
        std::vector<int>& get_nearest_point_index();
        void ft_outlier_remove(const zvision::PointCloud& src, zvision::PointCloud&dst);
        /* config file */
        void ft_by_config_file(const zvision::PointCloud& src, zvision::PointCloud&dst);
        /* blooming filter*/
        void ft_blooming_remove(const zvision::PointCloud& src, zvision::PointCloud&dst);

    private:
        FilterParam param_;
        zvision::CalibrationDataSinCosTable cal_lut_;
         /* voxel*/
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
         /* line*/
        std::vector<int> point_line_number_;
         /* config file */
        std::shared_ptr<std::vector<bool>> downsample_mask_ = nullptr;
    };
    
} // namespace zv_processor
