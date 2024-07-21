//
// Created by wayne on 10/3/21.
//

#ifndef DBSCAN_DT_CLUSTER_DBSCAN_SIMPLE_QUICK_H
#define DBSCAN_DT_CLUSTER_DBSCAN_SIMPLE_QUICK_H


#include <pcl/point_types.h>
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2
using namespace pcl;
using namespace std;
//比较点的数量大小
// inline bool comparePointClusters (const pcl::PointIndices &a, const pcl::PointIndices &b) {
//     return (a.indices.size () < b.indices.size ());
// }

template <typename PointT>
class DBSCANSimpleCluster_QUI{
public:
    //输入点云
    virtual void setInputCloud(pcl::PointCloud<PointXYZ>::Ptr cloud) {
        input_cloud_ = cloud;
    }
    //设置遍历方法
    void setSearchMethod(pcl::KdTreeFLANN<pcl::PointXYZ> tree) {
        search_method_ = tree;
    }
    //输出点云vector
    void extract(std::vector<pcl::PointIndices>& cluster_indices) {
        std::vector<int> nn_indices;
        std::vector<bool> is_noise(input_cloud_->points.size(), false);
        std::vector<int> types(input_cloud_->points.size(), UN_PROCESSED);//生成点云数量大小的数组
        for (int i = 0; i < input_cloud_->points.size(); i++)
        {
            if (types[i] == PROCESSED)
            {
                continue;
            }
//            float distance = point_distance(input_cloud_, i);
////            if (distance >= 80)
////            {
////                types[i] = PROCESSED;
////                continue;
////            }
            db(i,  nn_indices, is_noise, types, cluster_indices);
        }

    }
    void db(int i,  std::vector<int>& nn_indices, std::vector<bool>& is_noise, std::vector<int>& types,std::vector<pcl::PointIndices>& cluster_indices)
    {
        // double x = 0.11;
        // double y = 0.11;
        // double z=0.2;
        //返回搜索到点的数量，      下标， 聚类距离， 存放下标， 存放距离，
        int dis=sqrt(pow(input_cloud_->points[i].x,2)+pow(input_cloud_->points[i].y,2))/10;
        int nn_size = radiusSearch(i,  nn_indices);

        //如果点的数量小于核心点的值，认为是噪声，跳过
        if (nn_size < minPts_)
        {
            is_noise[i] = true;
            return;
        }
        std::vector<int> seed_queue;
        //使用过的点设置为1,避免重复遍历
        seed_queue.push_back(i);
        types[i] = PROCESSED;
        for (int j = 0; j < nn_size; j++)
        {
            if (nn_indices[j] != i)
            {
                seed_queue.push_back(nn_indices[j]);
                types[nn_indices[j]] = PROCESSING;
            }
        } // for every point near the chosen core point.

        //如果
        int sq_idx = 1;
        while (sq_idx < seed_queue.size())
        {
            int cloud_index = seed_queue[sq_idx];
            if (is_noise[cloud_index] || types[cloud_index] == PROCESSED)
            {
                // seed_queue.push_back(cloud_index);
                types[cloud_index] = PROCESSED;
                sq_idx++;
                continue; // no need to check neighbors.
            }
            nn_size = radiusSearch(cloud_index,   nn_indices);
            if (nn_size >= minPts_)
            {
                for (int j = 0; j < nn_size; j++)
                {
                    if (types[nn_indices[j]] == UN_PROCESSED)
                    {
                        seed_queue.push_back(nn_indices[j]);
                        types[nn_indices[j]] = PROCESSING;
                    }
                }
            }
            
            types[cloud_index] = PROCESSED;
            sq_idx++;
        }
        //填入聚类后的结果
        if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size () <= max_pts_per_cluster_)
        {
            pcl::PointIndices r;
            r.indices.resize(seed_queue.size());
            for (int j = 0; j < seed_queue.size(); ++j)
            {
                r.indices[j] = seed_queue[j];
            }
            std::sort (r.indices.begin (), r.indices.end ());
            r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());
            r.header = input_cloud_->header;
            cluster_indices.push_back (r);
        }
    }
protected:
    pcl::PointCloud<PointXYZ>::Ptr input_cloud_;
    // float arr[8]={0.3,0.35,0.35,0.35,0.35,0.4,0.45,0.55};
     double radius_square=0.3;
    // int minPts_[12]={4,4,4,4,3,3,3,3,3,3,3,2};
    int minPts_=3;
    // int min_pts_per_cluster_[12]= {6,6,6,6,6,6,6,6,6,6,6,6};//最小的聚类点个数
    int min_pts_per_cluster_= 4;//最小的聚类点个数
    int max_pts_per_cluster_ {std::numeric_limits<int>::max()};//最大的聚类点个数
    pcl::KdTreeFLANN<pcl::PointXYZ> search_method_;//设置遍历方法
    //聚类核心代码
    //下标， 聚类距离， 存放下标， 存放距离
    virtual int radiusSearch(int index, std::vector<int> &k_indices)
    {
        vector<int> pointIdxRadiusSearch;          //保存每个近邻点的索引
	    vector<float> pointRadiusSquaredDistance;
        k_indices.clear();
        k_indices.push_back(index);
        double d1=sqrt(pow(input_cloud_->points[index].x,2)+pow(input_cloud_->points[index].y,2));
        // int q=d1/10;
        // if(q>=7)
        // {
        //     radius_square=0.55;
        // }
        // else
        // {
        //     radius_square=arr[q];
        // }  
        if (search_method_.radiusSearch(input_cloud_->points[index], radius_square, pointIdxRadiusSearch,pointRadiusSquaredDistance,0) > 0)
	    {

           for(int u=0;u<pointIdxRadiusSearch.size();u++)
           {
              k_indices.push_back(pointIdxRadiusSearch[u]);
           }
        return k_indices.size();
        }
//        float point_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud, int point_index)
//        {
//            return sqrt(pow(in_cloud->points[point_index].x, 2) +
//                        pow(in_cloud->points[point_index].y, 2)) + pow(in_cloud->points[point_index].z, 2);
    }
};
std::vector<pcl::PointIndices> dbscan_simple_quick(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
//    DBSCANKdtreeCluster<pcl::PointXYZ> ec;
    DBSCANSimpleCluster_QUI<pcl::PointXYZ> ec;
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);//聚类后的结果

    return cluster_indices;
};

#endif //DBSCAN_DT_CLUSTER_DBSCAN_SIMPLE_H
