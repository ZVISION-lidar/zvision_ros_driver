#pragma once
#include <yaml-cpp/yaml.h>
#include <pcl/common/pca.h>
#include <pcl/search/kdtree.h>

#include "dbscan.h"
#include "tic_toc.hpp"
#include <common/print.h>
#include <common/define.h>


typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

struct BloomingParams {
    float retro_min_samples;
    float epsilon;

    float blooming_dist;
    float retro_ins;
    float blooming_ins1;
    float blooming_ins2;
};

struct RetroObj{
    int retro_num;
    double massx = 0.0;
    double massy = 0.0;
    double massz = 0.0;
    double dist = 0.0;

    std::vector<int> retro_vec;
    std::map<int, std::set<int>> col_row_map;
    std::map<int, std::set<double>> col_dist_map;
};


class BloomingModel{
    public:
        static std::shared_ptr<std::vector<std::vector<int>>> nearest_point_table_ptr;

    private:
        int retro_nums_ = 0;

    public:
        BloomingParams params_;
        explicit BloomingModel(std::string yaml_path);
        void genNearestPointTable(std::vector<zvision::Point>& lidar_points, int near_cnt=20);
        
        void main_process(
            std::vector<zvision::Point>& lidar_points
        );
        void clustered_retro(
            std::vector<zvision::Point>& lidar_points,
            std::vector<RetroObj>& retro_objs
        );
        void find_sensitive_points(
            std::vector<zvision::Point>& lidar_points,
            std::vector<RetroObj>& retro_objs
        );

        ~BloomingModel();
};