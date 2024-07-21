//
// Created by wayne on 10/3/21.
//

#ifndef DBSCAN_DT_CLUSTER_DBSCAN_SIMPLE_H
#define DBSCAN_DT_CLUSTER_DBSCAN_SIMPLE_H


#include <pcl/point_types.h>
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2
using namespace pcl;
using namespace std;
//比较点的数量大小
inline bool comparePointClusters (const pcl::PointIndices &a, const pcl::PointIndices &b) {
    return (a.indices.size () < b.indices.size ());
}
template <typename PointT>
class DBSCANSimpleCluster{
public:
    //输入点云
    virtual void setInputCloud(pcl::PointCloud<PointXYZRGB>::Ptr cloud) {
        this->input_cloud_ = cloud ;
    }
    //设置模式
    void setMode(int mode)
    {
        if(mode==0)
        {
            this->za=3;
        }
        else
        {
            this->za=0.3;
        }
    }
    //设置遍历方法
    void setSearchMethod(pcl::KdTreeFLANN<pcl::PointXYZRGB> tree) {
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
    pcl::PointCloud<PointXYZRGB>::Ptr input_cloud_;
    float za=1.2;
    double radius_square=0.3;
    float arr[8]={0.2,0.25,0.3,0.3,0.35,0.4,0.45,0.5};
    int minPts_=3;
    int min_pts_per_cluster_=18;//最小的聚类点个数
    int max_pts_per_cluster_ {std::numeric_limits<int>::max()};//最大的聚类点个数
    pcl::KdTreeFLANN<pcl::PointXYZRGB> search_method_;//设置遍历方法
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
        // // if(q>7)
        // // {
        // //     radius_square=0.5;
        // // }
        // // else
        // // {
        // //     radius_square=arr[q];
        // // }

        if (search_method_.radiusSearch(input_cloud_->points[index], radius_square, pointIdxRadiusSearch,pointRadiusSquaredDistance,0) > 0)
	    {

           for(int u=0;u<pointIdxRadiusSearch.size();u++)
           {
            //   if(abs(input_cloud_->points[pointIdxRadiusSearch[u]].z-input_cloud_->points[index].z)<za)
            //   {
            //      k_indices.push_back(pointIdxRadiusSearch[u]);
            //   }
                k_indices.push_back(pointIdxRadiusSearch[u]);   

           }
        return k_indices.size();
        }
    }
};
void dbscan_simple(unordered_map<int,vector<pcl::PointCloud<pcl::PointXYZ>>>  pole_cluster , unordered_map<int,vector<vector<float>>>  pole_cluster_mean ,    vector<pcl::PointCloud<pcl::PointXYZ>>& pole_vector,
    vector<vector<float>>& mean_vector,
    vector<vector<int>>& num_vector)
{

    pcl::KdTreeFLANN<pcl::PointXYZRGB> tree;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_kd(new pcl::PointCloud<pcl::PointXYZRGB>);//导入的点云
    unordered_map<int,vector<vector<float>>>::iterator iter;
    iter = pole_cluster_mean.begin();
    while(iter != pole_cluster_mean.end())
    {
        int num=iter->first;
        vector<vector<float>> mean_buffer=iter->second;
        for(int i = 0;i<mean_buffer.size();i++)
        {
            PointXYZRGB P;
            P.x=mean_buffer[i][0];
            P.y=mean_buffer[i][1];
            P.z=0;
            P.r=num;
            P.g=i;
            cloud_kd->points.push_back(P);
        }
        iter++;
    }
    tree.setInputCloud(cloud_kd);

    std::vector<pcl::PointIndices> cluster_indices;
//  DBSCANKdtreeCluster<pcl::PointXYZ> ec;
    DBSCANSimpleCluster<pcl::PointXYZRGB> ec;
    ec.setSearchMethod(tree);
    // ec.setMode(mode);
    ec.setInputCloud(cloud_kd);
    ec.extract(cluster_indices);//聚类后的结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr pole_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);//导入的点云
    for(int p=0;p<cluster_indices.size();p++)
    {
        float mean_x=0;
        float mean_y=0;

        vector<float>mean;
        vector<int> num_cluster;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr pole_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for(int n =0;n<cluster_indices[p].indices.size();n++)
        {
            PointXYZRGB P=cloud_kd->points[cluster_indices[p].indices[n]];
            int num= (int)P.r;
            int i=(int)P.g;
            *pole_point_cloud+=pole_cluster[num][i];
            mean_x+=P.x;
            mean_y+=P.y;
            num_cluster.push_back(num);
        }
        mean_x/=cluster_indices[p].indices.size();
        mean_y/=cluster_indices[p].indices.size();
        mean.push_back(mean_x);
        mean.push_back(mean_y);
        pole_vector.push_back(*pole_point_cloud);
        mean_vector.push_back(mean);
        num_vector.push_back(num_cluster);     
        pole_point_cloud->clear();
    }
};

#endif //DBSCAN_DT_CLUSTER_DBSCAN_SIMPLE_H
