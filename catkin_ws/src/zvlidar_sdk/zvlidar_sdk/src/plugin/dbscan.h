#pragma once

#include <memory>
#include <point_cloud.h>
#include <common/define.h>

#define SUCCESS 0
#define FAILURE -3

class DBSCAN {

public:
    std::shared_ptr<std::vector<zvision::Point>> m_points;
    std::shared_ptr<std::vector<std::vector<int>>> nearest_table_ptr;
    
private:    
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float nor_epsilon;

public:    
    DBSCAN(
        unsigned int minPts,
        float nor_eps,
        std::shared_ptr<std::vector<std::vector<int>>> nearest_table_ptr,
        std::shared_ptr<std::vector<zvision::Point>> points
    ){
        this->nearest_table_ptr = nearest_table_ptr;
        this->m_minPoints = minPts;
        this->nor_epsilon = nor_eps;
        this->m_points = points;
        this->m_pointSize = points->size();
    }
    ~DBSCAN(){}

    int run();
    void calculateCluster(zvision::Point& point, std::vector<int>& clusterIndex);
    int expandCluster(zvision::Point& point, int clusterID);
    inline double calculateDistance(const zvision::Point& pointCore, const zvision::Point& pointTarget);

    int getTotalPointSize() {return m_pointSize;}
    int getMinimumClusterSize() {return m_minPoints;}
    int getNorEpsilonSize() {return nor_epsilon;}

    void getClusterMap(std::map<int, std::vector<int>>& cluster_idx_map);
};