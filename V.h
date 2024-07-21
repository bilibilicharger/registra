#ifndef SQUARE_H
#define SQUARE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <queue>

class VoxelCluster
{
private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_;
    float sq_x = 0.2;
    float sq_y = 0.2;
    int length = 400;
    int sq_x_size = int(2 * length / sq_x);
    int sq_y_size = int(2 * length / sq_y);

    // 声明三维数组，第一维是水平角度，第二维是极长，第三维是点的下标
    std::vector<std::vector<std::vector<int>>> voxel_cloud;

    std::vector<std::vector<bool>> visited;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> voxel_recult;
    int stride = 2;

public:
    VoxelCluster()
    {
        voxel_cloud = std::vector<std::vector<std::vector<int>>>(sq_x_size, std::vector<std::vector<int>>(sq_y_size, std::vector<int>()));
        visited = std::vector<std::vector<bool>>(sq_x_size, std::vector<bool>(sq_y_size, false));
    }
    void BuildVoxel()
    {
        voxel_cloud = std::vector<std::vector<std::vector<int>>>(sq_x_size, std::vector<std::vector<int>>(sq_y_size, std::vector<int>()));
        visited = std::vector<std::vector<bool>>(sq_x_size, std::vector<bool>(sq_y_size, false));
        // std::cout << "input_cloud_: " << input_cloud_->points.size() << std::endl;
        for (int i = 0; i < input_cloud_->points.size(); i++)
        {

            int x_index = int(input_cloud_->points[i].x / sq_x + sq_x_size / 2);
            if (x_index >= sq_x_size || x_index < 0)
            {
                continue;
            }
            int y_index = int(input_cloud_->points[i].y / sq_y + sq_y_size / 2);
            if (y_index >= sq_y_size || y_index < 0)
            {
                continue;
            }

            voxel_cloud[x_index][y_index].push_back(i);
            visited[x_index][y_index] = true;
        }
    }
    void SetInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        input_cloud_ = cloud;
    }

    // 遍历voxel_cloud 设置步长按照广度优先遍历，进行聚类，将聚类结果存入voxel_reculst  vector中
    void Clustering()
    {
        voxel_recult.clear();
        for (int i = 0; i < sq_x_size; i++)
        {
            for (int j = 0; j < sq_y_size; j++)
            {
                std::queue<std::pair<int, int>> q;
                if (visited[i][j] == true)
                {
                    pcl::PointCloud<pcl::PointXYZI> temp_cloud;
                    q.push(std::make_pair(i, j));
                    while (!q.empty())
                    {
                        int x = q.front().first;
                        int y = q.front().second;
                        q.pop();
                        for (int k = -stride; k <= stride; k++)
                        {
                            for (int l = -stride; l <= stride; l++)
                            {
                                if (x + k >= 0 && x + k < sq_x_size && y + l >= 0 && y + l < sq_y_size && visited[x + k][y + l] == true)
                                {

                                    for (int m = 0; m < voxel_cloud[x + k][y + l].size(); m++)
                                    {
                                        temp_cloud.push_back(input_cloud_->points[voxel_cloud[x + k][y + l][m]]);
                                    }

                                    visited[x + k][y + l] = false;
                                    q.push(std::make_pair(x + k, y + l));
                                }
                            }
                        }
                    }
                    // cout << "temp_cloud.size()" << temp_cloud.size() << endl;
                    if (temp_cloud.size() > 10)
                    {
                        voxel_recult.push_back(temp_cloud);
                    }
                }
            }
        }
    }
    // 返回聚类结果
    std::vector<pcl::PointCloud<pcl::PointXYZI>> GetVoxelRecult()
    {
        
        BuildVoxel();
        // std::cout << "1" << std::endl;
        Clustering();
        return voxel_recult;
    }
};

#endif