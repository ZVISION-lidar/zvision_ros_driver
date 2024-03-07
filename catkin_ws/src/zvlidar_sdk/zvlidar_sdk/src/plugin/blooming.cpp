#include "blooming.h"


void  calc_center(
    std::vector<int>& idx_vec,
    std::vector<zvision::Point>& lidar_points,
    RetroObj& retro_obj
){
    for (int idx : idx_vec) {
        // centroid[0] += lidar_points.at(idx).x;
        // centroid[1] += lidar_points.at(idx).y;
        // centroid[2] += lidar_points.at(idx).z;
        retro_obj.dist += lidar_points.at(idx).distance;
    }
    retro_obj.dist /= idx_vec.size();
}


BloomingModel::BloomingModel(std::string yaml_path){
    YAML::Node config_node = YAML::LoadFile(yaml_path)["ml30spb1"];
    params_.retro_min_samples = config_node["retro_min_samples"].as<float>();
    params_.epsilon = config_node["epsilon"].as<float>();

    params_.blooming_dist = config_node["blooming_dist"].as<float>();
    params_.retro_ins = config_node["retro_ins"].as<float>();
    params_.blooming_ins1 = config_node["blooming_ins1"].as<float>();
    params_.blooming_ins2 = config_node["blooming_ins2"].as<float>();

}


BloomingModel::~BloomingModel() {}


void BloomingModel::genNearestPointTable(
    std::vector<zvision::Point>& lidar_points,
    int near_cnt
){
    if ( BloomingModel::nearest_point_table_ptr != nullptr){
        return;
    }else {
        BloomingModel::nearest_point_table_ptr = std::make_shared<std::vector<std::vector<int>>>();
        BloomingModel::nearest_point_table_ptr->resize(lidar_points.size());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->resize(lidar_points.size());
        for (size_t i=0; i<lidar_points.size(); i++) {
            cloud->points.at(i).x = lidar_points.at(i).azi;
            cloud->points.at(i).y = lidar_points.at(i).ele;
            cloud->points.at(i).z = 0;
        }

        pcl::search::Search<pcl::PointXYZ>::Ptr tree_;
        tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
        tree_->setInputCloud(cloud);

        for (size_t i=0; i<lidar_points.size(); i++) {
            std::vector<int> nn_indices(near_cnt);
            std::vector<float> nn_dists(near_cnt);

            pcl::PointXYZ pp;
            pp.x = lidar_points.at(i).azi;
            pp.y = lidar_points.at(i).ele;
            pp.z = 0;
            if (tree_->nearestKSearch(pp, near_cnt, nn_indices, nn_dists) != near_cnt) {continue;}
            BloomingModel::nearest_point_table_ptr->at(i) = nn_indices;
        }
    }
}


void BloomingModel::main_process(std::vector<zvision::Point>& lidar_points){

    TicToc tic, tic1;
    for (int i = 0; i < lidar_points.size(); i++) {
        if (lidar_points.at(i).reflectivity > this->params_.retro_ins) {lidar_points.at(i).retro_flag = true;}
    }

    this->genNearestPointTable(lidar_points);

    std::vector<RetroObj> retro_objs;
    clustered_retro(lidar_points, retro_objs);
    LOG_INFO("-- retro clustered cost: %.1lf ms \n", tic.toc());
    find_sensitive_points(lidar_points, retro_objs);
    LOG_INFO("- algorithm cost: %.1lf ms \n", tic1.toc());

}

void BloomingModel::clustered_retro(
    std::vector<zvision::Point>& lidar_points,
    std::vector<RetroObj>& retro_objs
){

    TicToc tic;
    std::map<int, std::vector<int>> cluster_idx_map;
    std::shared_ptr<std::vector<zvision::Point>> lidar_points_ptr = std::make_shared<std::vector<zvision::Point>>(lidar_points);

    DBSCAN db = DBSCAN(
        this->params_.retro_min_samples,
        this->params_.epsilon,
        BloomingModel::nearest_point_table_ptr,
        lidar_points_ptr
    );
    db.run();
    db.getClusterMap(cluster_idx_map);
    LOG_INFO("- dbscan cost: %.1lf ms \n", tic.toc());

    int retro_idx = 0;
    for(auto it : cluster_idx_map){

        // pca analy
        PointCloud::Ptr pca_in_cloud(new PointCloud);
        PointCloud::Ptr pca_out_cloud(new PointCloud);
        pca_in_cloud->resize(it.second.size());
        for (int i = 0; i<it.second.size(); i++) {
            pca_in_cloud->at(i).x = lidar_points.at(it.second.at(i)).x;
            pca_in_cloud->at(i).y = lidar_points.at(it.second.at(i)).y;
            pca_in_cloud->at(i).z = lidar_points.at(it.second.at(i)).z;
        }

        pcl::PCA<Point> pca;
        pca.setInputCloud(pca_in_cloud);
        try{
            pca.project(*pca_in_cloud, *pca_out_cloud);
        }catch(pcl::PCLException& e){
            continue;
        }

        std::vector<float> xvec;
        xvec.reserve(pca_out_cloud->size());
        std::transform(
            pca_out_cloud->begin(), 
            pca_out_cloud->end(),
            std::back_inserter(xvec), // 需要使用back_inserter，它会自动为vector分配空间
            [](auto &kv){ return kv.x;} // 返回输入迭代器的key，需要c++14
        );
        auto minmax_x = std::minmax_element(xvec.begin(), xvec.end());

        std::vector<float> yvec;
        yvec.reserve(pca_out_cloud->size());
        std::transform(
            pca_out_cloud->begin(),
            pca_out_cloud->end(),
            std::back_inserter(yvec), // 需要使用back_inserter，它会自动为vector分配空间
            [](auto &kv){ return kv.y;} // 返回输入迭代器的key，需要c++14
        );
        auto minmax_y = std::minmax_element(yvec.begin(), yvec.end());

        std::vector<float> zvec;
        zvec.reserve(pca_out_cloud->size());
        std::transform(
            pca_out_cloud->begin(), 
            pca_out_cloud->end(),
            std::back_inserter(zvec), // 需要使用back_inserter，它会自动为vector分配空间
            [](auto &kv){ return kv.z;} // 返回输入迭代器的key，需要c++14
        );
        auto minmax_z = std::minmax_element(zvec.begin(), zvec.end());

        double xdiff = (*minmax_x.second - *minmax_x.first);
        double ydiff = (*minmax_y.second - *minmax_y.first);
        double zdiff = (*minmax_z.second - *minmax_z.first);
        // LOG_INFO("xmin: %.2lf, xmax: %.2lf, ymin: %.2lf, ymax: %.2lf, zmin: %.2lf, zmax: %.2lf", *minmax_x.first, *minmax_x.second, *minmax_y.first, *minmax_y.second, *minmax_z.first, *minmax_z.second);
        
        RetroObj retro_obj;
        calc_center(it.second, lidar_points, retro_obj);
        // if (zdiff > 1 and retro_obj.dist < 50){continue;}
        if (xdiff < 0.5 and ydiff < 0.3 and retro_obj.dist < 15){continue;}
        if (xdiff < 0.2 or ydiff < 0.2){continue;}

        // if (retro_obj.dist < 15){continue;}

        LOG_INFO(
            "--- cluster id: %d,cluster size: %ld, mean dist: %.2lf, xdiff: %.2lf, ydiff: %.2lf, zdiff: %.2lf \n",
            it.first, it.second.size(), retro_obj.dist, xdiff, ydiff, zdiff
        );
        retro_obj.retro_num = retro_idx;
        retro_obj.retro_vec = it.second;
        for (int pidx : it.second) {
            lidar_points.at(pidx).retro_num = retro_idx;
            int col = lidar_points.at(pidx).col;
            int row = lidar_points.at(pidx).row;
            double dist = std::round(lidar_points.at(pidx).distance * 10) / 10;
            if (retro_obj.col_row_map.find(col) == retro_obj.col_row_map.end()) {
                retro_obj.col_row_map.insert({col, std::set<int>()});
                retro_obj.col_row_map.at(col).insert(row);
            }else{
                // retro_obj.col_idx_map.at(col).insert(pidx);
                retro_obj.col_row_map.at(col).insert(row);
            }

            if (retro_obj.col_dist_map.find(col) == retro_obj.col_dist_map.end()) {
                retro_obj.col_dist_map.insert({col, std::set<double>()});
                retro_obj.col_dist_map.at(col).insert(dist);
            }else{
                retro_obj.col_dist_map.at(col).insert(dist);
            }
        }
        retro_objs.emplace_back(retro_obj);
        retro_idx += 1;
    }
    LOG_INFO("-- cluster num: %ld \n", retro_objs.size());
}


void BloomingModel::find_sensitive_points(
    std::vector<zvision::Point>& lidar_points,
    std::vector<RetroObj>& retro_objs
){
    int col_offset = 6, col_pad = 6;
    for (auto retro_obj : retro_objs){
        std::vector<int> col_vec;
        double blooming_dist = this->params_.blooming_dist;
        if (retro_obj.dist >= 40)
        {
            blooming_dist *= 2 ;
        }

        std::transform(
            retro_obj.col_row_map.begin(), 
            retro_obj.col_row_map.end(),
            std::back_inserter(col_vec), // 需要使用back_inserter，它会自动为vector分配空间
            [](auto &kv){ return kv.first;} // 返回输入迭代器的key，需要c++14
        );
        
        auto minmax_col = std::minmax_element(col_vec.begin(), col_vec.end());
        int min_col = *minmax_col.first;
        int max_col = *minmax_col.second;
        // 将相邻retro左右相邻列放入处理逻辑且按照最近邻的列进行处理
        for (int i = 1; i <= col_pad; i++) {
    
            if (*minmax_col.first - i >= 0){
                retro_obj.col_row_map.insert({*minmax_col.first - i, retro_obj.col_row_map.at(*minmax_col.first)});
                retro_obj.col_dist_map.insert({*minmax_col.first - i, retro_obj.col_dist_map.at(*minmax_col.first)});
                min_col -= 1;
            }
            if (*minmax_col.second + i <= 799){
                retro_obj.col_row_map.insert({*minmax_col.second + i, retro_obj.col_row_map.at(*minmax_col.second)});
                retro_obj.col_dist_map.insert({*minmax_col.second + i, retro_obj.col_dist_map.at(*minmax_col.second)});
                max_col += 1;
            }
        }

        for (int _col_idx = min_col; _col_idx <= max_col; _col_idx++) 
        {            
            if (retro_obj.col_row_map.find(_col_idx) == retro_obj.col_row_map.end()){continue;}
            auto minmax_row = std::minmax_element(retro_obj.col_row_map.at(_col_idx).begin(), retro_obj.col_row_map.at(_col_idx).end());
            int row_min{*minmax_row.first / 12 * 12}, row_max{*minmax_row.second / 12 * 12 + 11};
            if (row_min >= row_max){continue;}

            // 处理当前slot上的本分区内的blooming
            for (int i = row_min; i <= row_max; i++) {              
                int lidar_idx = _col_idx * 96 + i;
                if (lidar_points.at(lidar_idx).cluster_id < 0){lidar_points.at(lidar_idx).sea_num = retro_obj.retro_num;}
                if (lidar_points.at(lidar_idx).reflectivity > params_.blooming_ins1){continue;}
                for (double _dist : retro_obj.col_dist_map.at(_col_idx)) {
                    if (std::abs(lidar_points.at(lidar_idx).distance - _dist) < blooming_dist ) {
                        lidar_points.at(lidar_idx).bom_num = retro_obj.retro_num;
                        break;
                    }
                }
            }

            // 处理偏移slot上的跨分区上的blooming
            for (int _area_num = *minmax_row.first / 12; _area_num <= *minmax_row.second / 12; _area_num++)
            {
                int _col_offset = col_offset;
                if (_area_num % 2 == 1){_col_offset *= -1;}
                if (_col_idx + _col_offset < 0 or _col_idx + _col_offset > 799){continue;}
                for (int _inter_area = _area_num - 1; _inter_area <= _area_num + 1; _inter_area+=2) {
                    if (_inter_area < 0 or _inter_area > 7){continue;}
                    for (int i = 0; i < 12; i++) {
                        int lidar_idx = (_col_idx + _col_offset) * 96 + (_inter_area * 12) + i;
                        if (lidar_points.at(lidar_idx).cluster_id < 0){lidar_points.at(lidar_idx).sea_num = retro_obj.retro_num;}
                        if (lidar_points.at(lidar_idx).reflectivity > params_.blooming_ins2){continue;}                  
                        for (double _dist : retro_obj.col_dist_map.at(_col_idx)) {
                            if (std::abs(lidar_points.at(lidar_idx).distance - _dist) < blooming_dist ) {
                                lidar_points.at(lidar_idx).bom_num = retro_obj.retro_num;
                                break;
                            }
                        }
                    }
                }
            }

        }

    }
}