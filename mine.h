
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/common/pca.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <thread>
#include "DBSCAN_simple.h"
using namespace std;
using namespace Eigen;



// Given two sets of 3D points, find the rotation + translation + scale
// which best maps the first set to the second.
// Source: http://en.wikipedia.org/wiki/Kabsch_algorithm

// The input 3D points are stored as columns.

float volume(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  //   pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  //   feature_extractor.setInputCloud(cloud);
  //   feature_extractor.compute();

  // // 获取 OBB 框参数
  //   pcl::PointXYZ min_point, max_point, position;
  //   Eigen::Matrix3f rotation;
  //   feature_extractor.getOBB(min_point, max_point, position, rotation);

  // // 将点云变换到 OBB 坐标系中
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::transformPointCloud(*cloud, *transformed_cloud, Eigen::Affine3f(Eigen::Translation3f(position.x, position.y, position.z) * rotation));

  //   Eigen::Matrix<float, 3, 3> rotation_matrix = rotation.matrix();
  //   float volume = fabs(rotation_matrix.determinant()) * (max_point.x - min_point.x) * (max_point.y - min_point.y) * (max_point.z - min_point.z);
  //   return volume;


  Eigen::Vector4f pca_centroid;
  Eigen::Matrix3f pca_covariance;
  pcl::compute3DCentroid(*cloud, pca_centroid);
  pcl::computeCovarianceMatrixNormalized(*cloud, pca_centroid, pca_covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> pca_solver(pca_covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f pca_eigen_vectors = pca_solver.eigenvectors();
  Eigen::Vector3f pca_eigen_values = pca_solver.eigenvalues();

  // 计算最小包围盒的体积
  pcl::PointXYZ min_point, max_point;
  pcl::getMinMax3D(*cloud, min_point, max_point);
  float box_volume = (max_point.x - min_point.x) * (max_point.y - min_point.y) * (max_point.z - min_point.z);
  return box_volume;

}
void overlap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud0, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr& overlap_cloud) {
    // 将两个点云分别放入 KD 树中
    KdTreeFLANN<PointXYZ> kdtree;
    kdtree.setInputCloud(cloud1);

    // 对于其中一个点云中的每一个点，使用 KD 树进行最近邻搜索，找到离它最近的点在另一个点云中的索引
    vector<int> indices;
    vector<float> distances;
    for (size_t i = 0; i < cloud0->size(); ++i) {
        if(kdtree.radiusSearch(cloud0->points[i], 0.08, indices,distances,0)>0)
        {
          overlap_cloud->push_back(cloud0->points[i]);
        }
    }
}
float IOU(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_intersection(new pcl::PointCloud<pcl::PointXYZ>);
    overlap(cloud0,cloud1,cloud_intersection);
    if(cloud_intersection->size()==0)
    {
      cout<<"wrong"<<endl;
      return 0;
    }
    cloud_intersection->width=cloud_intersection->size();
    cloud_intersection->height=1;
    pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/minetest/result/test/9.pcd", *cloud_intersection); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_all=*cloud0+*cloud1;
    float V_all=volume(cloud_all);
    float V_intersect=volume(cloud_intersection);
    if(V_all/V_intersect>5)
    {
      cout<<"ha"<<V_all/V_intersect<<endl;
      return 0;
    }
    else{
      cout<<V_all/V_intersect<<endl;
      return exp(-V_all/V_intersect);
    }
    
}

// void threadFunction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloud_vector) {
//       if(cloud->size()!=0)
//       {
//         std::vector<pcl::PointIndices> cluster_target;
//         cluster_target=dbscan_simple(cloud,1);  
//         for(int c=0;c<cluster_target.size();c++)
//         {
//           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_buffer(new pcl::PointCloud<pcl::PointXYZ>);
//           for(int num =0;num<cluster_target[c].indices.size();num++)
//           {
//             cloud_buffer->points.push_back(cloud->points[cluster_target[c].indices[num]]);
//           }
//           cloud_vector.push_back(cloud_buffer);
//         }
//       }
// }
Eigen::Affine3d Find3DAffineTransform(Eigen::MatrixXd in, Eigen::MatrixXd out) {

  // Default output
  Eigen::Affine3d A;
  A.linear() = Eigen::Matrix3d::Identity(3, 3);
  A.translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < in.cols()-1; col++) {
    dist_in  += (in.col(col+1) - in.col(col)).norm();
    dist_out += (out.col(col+1) - out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0)
    return A;
  double scale = dist_out/dist_in;
  out /= scale;

  // Find the centroids then shift to the origin
  Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
  for (int col = 0; col < in.cols(); col++) {
    in_ctr  += in.col(col);
    out_ctr += out.col(col);
  }
  in_ctr /= in.cols();
  out_ctr /= out.cols();
  for (int col = 0; col < in.cols(); col++) {
    in.col(col)  -= in_ctr;
    out.col(col) -= out_ctr;
  }

  // SVD
  Eigen::MatrixXd Cov = in * out.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  A.linear() = scale * R;
  A.translation() = scale*(out_ctr - R*in_ctr);
  
  return A;
}

class Mine{
public:
    Mine(vector<vector<vector<float>>>mean_value_two){
      this->mean0=mean_value_two[0];
      this->mean1=mean_value_two[1];
    }
   
    void setInputNum(vector<int>s0,vector<int>t1){
      this->s0=s0;
      this->t1=t1;
      this->P.resize(3,s0.size());
      this->Q.resize(3,t1.size());
      for(int i=0;i<s0.size();i++)
      {
        P(0,i)=mean0[0][s0[i]];
        P(1,i)=mean0[1][s0[i]];
        P(2,i)=0;
        Q(0,i)=mean1[0][t1[i]];
        Q(1,i)=mean1[1][t1[i]];
        Q(2,i)=0;
      }
      // cout<<P<<endl<<endl;
      // cout<<Q<<endl<<endl;
    }
    
    void getTransformMatrix( Eigen::Affine3d &T){
      this->Tr=Find3DAffineTransform(P,Q);
      T=Tr;
    }

    void getError(double& err)
    {
      Eigen::Matrix4d Tr_mat = Tr.matrix();
      Eigen::Matrix3d R = Tr_mat.block<3, 3>(0, 0); // 提取旋转矩阵
      Eigen::Vector3d t = Tr_mat.block<3, 1>(0, 3);
      MatrixXd P_transformed = R * P + t.replicate(1, P.cols());

      // 计算 P_transformed 和 Q 之间的误差
      MatrixXd diff = P_transformed - Q;
      VectorXd norms  = diff.colwise().norm();
      double mean_err = norms.mean();
      this->err=mean_err;
    }
   
    void saveTransformMatrix(int n,string filename){
      fstream myFile;
      myFile.open("/home/shy/1lidarcamera/data/"+filename+"/result/旋转文件/旋转"+to_string(n)+".txt", ios::out); 
      cout<<"第"<<n<<"个结果的旋转矩阵 :"<<endl<<Tr.matrix()<<endl<<endl;
      if (myFile.is_open())
	    {
		    myFile << Tr.matrix();
	  	  myFile.close();
  	  }
    }
  
    // void getScore(vector<vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> not_vector_two,float &result)
    // {
    //   vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> NotPoleOfPole0=not_vector_two[0];
    //   vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> NotPoleOfPole1=not_vector_two[1];
    //   vector<float> IOUScore;
    //   float SumOfScore=0;
    //   for(int i=0;i<s0.size();i++)
    //   {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::transformPointCloud (*NotPoleOfPole0[s0[i]], *cloud0, this->Tr);//第一个雷达进行旋转
    //     *cloud1=*NotPoleOfPole1[t1[i]];
    //     vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector0,cloud_vector1;
    //     std::thread t0(threadFunction, cloud0,std::ref(cloud_vector0));  // 创建一个线程，并执行 myFunction 函数，传入参数 42
    //     std::thread t11(threadFunction, cloud1,std::ref(cloud_vector1));  // 创建一个线程，并执行 myFunction 函数，传入参数 42
    //     t0.join();
    //     t11.join();
    //     for(int j=0;j<cloud_vector0.size();j++)
    //     {
    //       if(volume(cloud_vector0[j])>50)
    //       {
    //         continue;
    //       }
    //       for(int k=0;k<cloud_vector1.size();k++)
    //       {          
    //         float score=IOU(cloud_vector0[j],cloud_vector1[k]);
    //         cout<<this->s0[i]<<"与"<<this->t1[i]<<"中 "<<j<<"与"<<k<<"的分数: "<<score<<endl;
    //         IOUScore.push_back(score);
    //         SumOfScore+=score;
    //       }
    //     }
    //   }
    //   result=exp(-static_cast<float> (this->err))+SumOfScore+s0.size()/4;
    // }
protected:
    vector<vector<float>>mean0,mean1;
    MatrixXd P, Q;
    Eigen::Affine3d Tr;
    vector<int>s0;
    vector<int>t1;
    double err=0;
};

// void mine(vector<vector<pcl::PointXYZ>> P_vector,vector<vector<pcl::PointXYZ>> Q_vector,vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_list,int k)
// {
//     /*******将矩阵移动到以均值为原点的新矩阵（去中心化）*********/
//     MatrixXd P(3,9);
//     P<<P_vector[0][0].x,P_vector[0][1].x,P_vector[0][2].x,P_vector[1][0].x,P_vector[1][1].x,P_vector[1][2].x,P_vector[2][0].x,P_vector[2][1].x,P_vector[2][2].x,
//            P_vector[0][0].y,P_vector[0][1].y,P_vector[0][2].y,P_vector[1][0].y,P_vector[1][1].y,P_vector[1][2].y,P_vector[2][0].y,P_vector[2][1].y,P_vector[2][2].y,
//             P_vector[0][0].z,P_vector[0][1].z,P_vector[0][2].z,P_vector[1][0].z,P_vector[1][1].z,P_vector[1][2].z,P_vector[2][0].z,P_vector[2][1].z,P_vector[2][2].z;
//     MatrixXd Q(3,9);
//     Q<<Q_vector[0][0].x,Q_vector[0][1].x,Q_vector[0][2].x,Q_vector[1][0].x,Q_vector[1][1].x,Q_vector[1][2].x,Q_vector[2][0].x,Q_vector[2][1].x,Q_vector[2][2].x,
//            Q_vector[0][0].y,Q_vector[0][1].y,Q_vector[0][2].y,Q_vector[1][0].y,Q_vector[1][1].y,Q_vector[1][2].y,Q_vector[2][0].y,Q_vector[2][1].y,Q_vector[2][2].y,
//             Q_vector[0][0].z,Q_vector[0][1].z,Q_vector[0][2].z,Q_vector[1][0].z,Q_vector[1][1].z,Q_vector[1][2].z,Q_vector[2][0].z,Q_vector[2][1].z,Q_vector[2][2].z;
//     // zed(P,Q,k);
//     Eigen::Affine3d Tr=Find3DAffineTransform(P,Q);
//     fstream myFile;
//     myFile.open("/home/shy/lidarcamera/minetest/result/旋转文件/旋转"+to_string(k)+".txt", ios::out); 
//     if (myFile.is_open())
// 	  {
// 		  myFile << Tr.matrix();
// 	  	myFile.close();
//   	}
//     cout<<k<<" xuanzhuan:"<<Tr.matrix()<<endl<<endl;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::transformPointCloud (*cloud_list[0], *cloud0, Tr);//第一个雷达进行旋转
//     *cloud=*cloud0+*cloud_list[1];
//   //  Errorjudgment(P,Q,Tr,k);
//     cloud->width=cloud->size();
//     cloud->height=1;
//     pcl::io::savePCDFileASCII("/home/shy/lidarcamera/minetest/最终结果/"+to_string(k)+".pcd", *cloud);
// }

double distance_compute(vector<double> A,vector<double>B)
{
    double x=A[0]-B[0];
    double y=A[1]-B[1];
    return sqrt(x*x+y*y);
}
double getAngle(vector<double> A,vector<double>B)
{
    Vector3f a,b;
    a(0)=A[2];
    a(1)=A[3];
    a(2)=A[4];
    b(0)=B[2];
    b(1)=B[3];
    b(2)=B[4];
    return pcl::getAngle3D(a,b,true);
}