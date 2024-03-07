#include "dbscan.h"
#include "iostream"

int DBSCAN::run()
{
    for(size_t i=0; i<m_points->size(); i++){
        if (
            m_points->at(i).retro_flag == true
        )
        {
            m_points->at(i).cluster_id = zvision::DBScanPointType::UNCLASSIFIED;
        }else {
            m_points->at(i).cluster_id = zvision::DBScanPointType::NOCLASS;
        }
    }

    int clusterID = 1;
    std::vector<zvision::Point>::iterator iter;
    for(iter = m_points->begin(); iter != m_points->end(); ++iter)
    {
        if ( iter->cluster_id == zvision::DBScanPointType::UNCLASSIFIED )
        {
            if ( expandCluster(*iter, clusterID) != FAILURE )
            {
                clusterID += 1;
            }
        }
    }

    return 0;
}

int DBSCAN::expandCluster(zvision::Point& point, int clusterID)
{    
    std::vector<int> clusterSeeds;
    calculateCluster(point, clusterSeeds);
    if ( clusterSeeds.size() < m_minPoints )
    {
        point.cluster_id = zvision::DBScanPointType::NOISE;
        return FAILURE;
    }
    else
    {
        int index = 0, indexCorePoint = 0;
        std::vector<int>::iterator iterSeeds;
        for( iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
        {
            m_points->at(*iterSeeds).cluster_id = clusterID;
            if (m_points->at(*iterSeeds).idx == point.idx)
            {
                indexCorePoint = index;
            }
            ++index;
        }
        clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);

        for( std::vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i )
        {
            std::vector<int> clusterNeighors;
            calculateCluster(m_points->at(clusterSeeds[i]), clusterNeighors);

            if ( clusterNeighors.size() >= m_minPoints )
            {
                std::vector<int>::iterator iterNeighors;
                for ( iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors )
                {
                    if ( m_points->at(*iterNeighors).cluster_id == zvision::DBScanPointType::UNCLASSIFIED || m_points->at(*iterNeighors).cluster_id == zvision::DBScanPointType::NOISE )
                    {
                        if ( m_points->at(*iterNeighors).cluster_id == zvision::DBScanPointType::UNCLASSIFIED )
                        {
                            clusterSeeds.push_back(*iterNeighors);
                            n = clusterSeeds.size();
                        }
                        m_points->at(*iterNeighors).cluster_id = clusterID;
                    }
                }
            }
        }

        return SUCCESS;
    }
}

void DBSCAN::calculateCluster(zvision::Point& point, std::vector<int>& clusterIndex)
{
    if (point.distance <= 0){return;}
    for( int _idx : DBSCAN::nearest_table_ptr->at(point.idx))
    {
        // printf("point1: %lf, %lf, %lf, point2: %lf, %lf, %lf\n", point.az,  point.el, point.dist, this->m_points->at(_idx).az, this->m_points->at(_idx).el, this->m_points->at(_idx).dist);
        if ( this->m_points->at(_idx).retro_flag and calculateDistance(point, this->m_points->at(_idx)) <= point.distance * this->nor_epsilon)
        {
            clusterIndex.push_back(_idx);
        }
    }
    return;
}

inline double DBSCAN::calculateDistance(const zvision::Point& pointCore, const zvision::Point& pointTarget )
{
    return abs(pointCore.distance - pointTarget.distance);
}

void DBSCAN::getClusterMap(std::map<int, std::vector<int>>& cluster_idx_map){
    for (auto _point : *this->m_points) {
        if (_point.cluster_id < 1){continue;}
        if (cluster_idx_map.find(_point.cluster_id) == cluster_idx_map.end()) {
            cluster_idx_map.insert({_point.cluster_id, std::vector<int>()});
        }
        cluster_idx_map[_point.cluster_id].emplace_back(_point.idx);
    }
}
