#ifndef RANSAC_H
#define RANSAC_H
#include <iostream>
#include <pcl/io/pcd_io.h>
#include<string>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/sample_consensus/ransac.h>
#include<pcl/sample_consensus/sac_model_plane.h>
#include<pcl/common/pca.h>
#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/geometry.h>
#include <pcl/point_cloud.h>
#include <algorithm>
#include <stdlib.h>
#include <fstream>
#include <Eigen/Dense>
#include "DBSCAN_simple.h"
#include"file.h"
using namespace std;
using namespace Eigen;
using namespace pcl;
pcl::PointCloud<pcl::PointXYZ>::Ptr x_y_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,float groundZ)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x"); //滤波字段名被设置为Z轴方向s
  pass.setFilterLimits (0,100); //设置在过滤方向上的过滤范围
  // pass.setKeepOrganized(true); // 保持有序点云结构，该功能用于有序点云才有意义。
  pass.setNegative (false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
  pass.filter (*cloud_filtered);
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y"); //滤波字段名被设置为Z轴方向
  pass.setFilterLimits (-40, 40); //设置在过滤方向上的过滤范围
  // pass.setKeepOrganized(true); // 保持有序点云结构，该功能用于有序点云才有意义。
  pass.setNegative (false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
  pass.filter (*cloud_filtered);
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z"); //滤波字段名被设置为Z轴方向
  pass.setFilterLimits (-10, 20); //设置在过滤方向上的过滤范围    groundZ
  // pass.setKeepOrganized(true); // 保持有序点云结构，该功能用于有序点云才有意义。
  pass.setNegative (false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
  pass.filter (*cloud_filtered);
  return cloud_filtered;
}
void ransac_not_direction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::ModelCoefficients::Ptr coefficients,pcl::PointIndices::Ptr inliers)
{
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus <pcl::PointXYZ> ransac(model_plane);//定义RANSAC算法模型
    ransac.setDistanceThreshold(0.06);//设定距离阈值
    ransac.setMaxIterations(100);     //设置最大迭代次数
    ransac.setProbability(0.99);      //设置从离群值中选择至少一个样本的期望概率
    ransac.computeModel();            //拟合平面

    vector<int> inliers_vector;              //用于存放内点索引的vector
    ransac.getInliers(inliers_vector);//inliers表示误差能容忍的点 记录的是点云的序号
    inliers->indices = inliers_vector;

    Eigen::VectorXf coeff;
    vector<float> Eigen2vector;    //传回去的指针必须用vector赋值
    ransac.getModelCoefficients(coeff);
    Eigen2vector.push_back(coeff(0));
    Eigen2vector.push_back(coeff(1));
    Eigen2vector.push_back(coeff(2));
    
    Eigen2vector.push_back(coeff(3));
    coefficients->values=Eigen2vector; //传回去的指针必须用vector赋值
}

bool ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr ground,pcl::ModelCoefficients::Ptr coefficients,float angle = 20, float Epsdistance=0.2)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    float EpsAngle = pcl::deg2rad(angle);
    Eigen::Vector3f Axis(0.0, 0.0, 1.0);
    //创建拟合时所需要的模型系数对象coefficients及存储内点的点索引集合对象inliers。
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);  // 可选择配置，设置模型系数需要优化
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);//设置分割的模型类型
    seg.setMethodType(pcl::SAC_RANSAC);    // 设置所用随机参数估计方法1
    seg.setDistanceThreshold(Epsdistance);        // 距离阈值，单位m.
    seg.setAxis(Axis);                     // 指定的轴
    seg.setEpsAngle(EpsAngle);             // 夹角阈值(弧度制)
	seg.setMaxIterations(10000);
    // 距离阈值表示点到估计模型的距离最大值。
    seg.setInputCloud(cloud);// 输入点云
    seg.segment(*inliers, *coefficients);// 存储结果到点集合inliers及存储平面模型系数coefficients
    pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);
    for(int i=0;i<inliers->indices.size();i++)
    {
      if(sqrt(pow(cloud->points[inliers->indices[i]].x,2)+pow(cloud->points[inliers->indices[i]].y,2))<200)
      {
         inliers1->indices.push_back(inliers->indices[i]);
      }
    }
    *inliers=*inliers1;
  
    extract.setInputCloud(cloud);//待提取点云
    extract.setIndices(inliers);//平面点云对应的索引
    extract.setNegative(false);
    extract.filter(*ground);//输出平面 bb

}
bool m_pca(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Eigen::Vector3f *eigen_values,Eigen::Vector3f *eigen_values1)
{
    if(cloud->size()<3)
    {
        return 0;
    }
	//--------------------------------PCA--------------------------------------
	pcl::PCA<pcl::PointXYZ>pca;
	pca.setInputCloud(cloud);
	//pca.getEigenValues(); // 获取特征值，特征值是按从大到小排列的
	*eigen_values = pca.getEigenValues();
	// pca.getEigenVectors()获取特征值对应的特征向量，矩阵的列向量为特征向量
	//*eigen_vector = pca.getEigenVectors();
//	//----------------------获取对应特征值与特征向量---------------------------
	float l1 = pca.getEigenValues()[0];
	float l2 = pca.getEigenValues()[1];
	float l3 = pca.getEigenValues()[2];

	*eigen_values1 = pca.getEigenVectors().col(0);
	Eigen::RowVector3f e2 = pca.getEigenVectors().col(1);
	Eigen::RowVector3f e3 = pca.getEigenVectors().col(2);

//	 cout << "最大特征值为: " << l1 << "\t对应特征向量为：" << *eigen_values1<< endl;
//	 cout << "第二特征值为：" << l2 << "\t对应特征向量为：" << e2 << endl;
//	 cout << "最小特征值为：" << l3 << "\t对应特征向量为：" << e3 << endl;

	return 1;
}
unordered_map<int, Eigen::Vector3f> normalPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normal_line (new pcl::PointCloud<pcl::Normal>);
    //------------------计算法线----------------------
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;//OMP加速
    //建立kdtree来进行近邻点集搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    n.setNumberOfThreads(10);//设置openMP的线程数
    //n.setViewPoint(0,0,0);//设置视点，默认为（0，0，0）
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(15);//点云法向计算时，需要所搜的近邻点大小
    //n.setRadiusSearch(0.03);//半径搜素              
    n.compute(*normal_line);//开始进行法向计
    unordered_map<int, Eigen::Vector3f> normals;
    for(int i=0;i<normal_line->size();i++)
    {
        Vector3f ne;
        ne<<normal_line->points[i].normal_x,normal_line->points[i].normal_y,normal_line->points[i].normal_z;
        normals[i]=ne;
    }
    return normals;
}

void Rotate_2_Z(pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_rota,int k)
{
    //将某个面旋转至Z平面，z平面法向量为0,0,1
    Eigen::Vector3f proj_normal(0, 0, 1);
    Eigen::Vector3f or_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);    //原始平面法向量

    //计算平面向量和Z平面的叉积，求旋转轴并归一化
    Eigen::Vector3f n_p = or_normal.cross(proj_normal);
    n_p.normalize();

    //计算平面和Z平面向量的夹角，求旋转角(按理来说这里的两个normal应该做应该做归一化)
    double rotate_angle = acos((or_normal.dot(proj_normal)) / (or_normal.norm()*proj_normal.norm()));

    //利用旋转轴和旋转角计算旋转矩阵
    Eigen::Matrix3f rotate_mat;
    Eigen::AngleAxis<float> angle_axis(rotate_angle, n_p);
    rotate_mat = angle_axis.toRotationMatrix();
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0,0)=rotate_mat(0,0);
    transform(0,1)=rotate_mat(0,1);
    transform(0,2)=rotate_mat(0,2);
    transform(1,0)=rotate_mat(1,0);
    transform(1,1)=rotate_mat(1,1);
    transform(1,2)=rotate_mat(1,2);
    transform(2,0)=rotate_mat(2,0);
    transform(2,1)=rotate_mat(2,1);
    transform(2,2)=rotate_mat(2,2);
    fstream myFile;
    myFile.open("/home/shy/1lidarcamera/data/hap/数据/地面"+to_string(k)+".txt", ios::out); 
    if(k<10000)
    {
        fstream myFile;
        myFile.open("/home/shy/1lidarcamera/data/hap/数据/地面"+to_string(k)+".txt", ios::out); 
        if (myFile.is_open())
	    {
		  myFile << transform;
	  	  myFile.close();
  	    }  
    }
    pcl::transformPointCloud (*cloud, *cloud_after_rota, transform);
}



// 自定义哈希函数，将x、y、z的值组合成一个唯一的标识
struct KeyHash
{
     std::size_t operator()(const std::tuple<int, int, int> &key) const
     {
          int x, y, z;
          std::tie(x, y, z) = key;
          return x ^ y ^ z;
     }
};

// 自定义比较函数，将x、y、z的值视为相等
struct KeyEqual
{
     bool operator()(const std::tuple<int, int, int> &lhs, const std::tuple<int, int, int> &rhs) const
     {
          int x1, y1, z1;
          int x2, y2, z2;
          std::tie(x1, y1, z1) = lhs;
          std::tie(x2, y2, z2) = rhs;
          return x1 == x2 && y1 == y2 && z1 == z2;
     }
};


void SubSampleScan(pcl::PointCloud<pcl::PointXYZ> &scan, double size_voxel)
{

     std::unordered_map<std::tuple<int, int, int>, pcl::PointCloud<pcl::PointXYZ>, KeyHash, KeyEqual> grid;
     for (int i = 0; i < (int)scan.size(); i++)
     {
          auto kx = static_cast<int>(scan.points[i].x / size_voxel);
          auto ky = static_cast<int>(scan.points[i].y / size_voxel);
          auto kz = static_cast<int>(scan.points[i].z / size_voxel);
          grid[std::make_tuple(kx, ky, kz)].push_back(scan.points[i]);
     }
     scan.resize(0);
     int step = 0;
     for (auto &n : grid)
     {
          if (n.second.size() > 0)
          {
               for (int i = 1; i < (int)n.second.size(); i++)
               {
                    n.second.points[0].x += n.second.points[i].x;
                    n.second.points[0].y += n.second.points[i].y;
                    n.second.points[0].z += n.second.points[i].z;

               }
               n.second.points[0].x /= (double)n.second.size();
                n.second.points[0].y /= (double)n.second.size();
                n.second.points[0].z /= (double)n.second.size();
               scan.push_back(n.second.points[0]);
               step++;
          }
     }
}
void GridSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr  &scan, double size_voxel_subsampling)
{
     pcl::PointCloud<pcl::PointXYZ>::Ptr frame_sub(new pcl::PointCloud<pcl::PointXYZ>);
     *frame_sub = *scan;
     SubSampleScan(*frame_sub, size_voxel_subsampling);
     *scan = *frame_sub;
}

#endif