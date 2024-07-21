// #include <Eigen/Dense>
// #include <Eigen/Sparse>
// #include <iostream>
// #include <iomanip>
// #include <math.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include "ransac.h"
// // #include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// // #include <CGAL/Polygon_2.h>
// #include <pcl/surface/concave_hull.h>
// #include <stdlib.h>
// #include <fstream>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/common/common_headers.h>
// #include <pcl/common/transforms.h>
// using namespace std;
// using namespace Eigen;

// const double DERIV_STEP = 1e-5;
// const int MAX_ITER = 100;

// #define max(a,b) (((a)>(b))?(a):(b))
// Eigen::Matrix4f Rotate_2_Z(const Vector3f& m, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
// {
//     //将某个面旋转至Z平面，z平面法向量为0,0,1
//     Eigen::Vector3f proj_normal(0, 0, 1);
//     Eigen::Vector3f or_normal(m(0), m(1), m(2));    //原始平面法向量

//     //计算平面向量和Z平面的叉积，求旋转轴并归一化
//     Eigen::Vector3f n_p = or_normal.cross(proj_normal);
//     n_p.normalize();

//     //计算平面和Z平面向量的夹角，求旋转角(按理来说这里的两个normal应该做应该做归一化)
//     double rotate_angle = acos((or_normal.dot(proj_normal)) / (or_normal.norm()*proj_normal.norm()));

//     //利用旋转轴和旋转角计算旋转矩阵
//     Eigen::Matrix3f rotate_mat;
//     Eigen::AngleAxis<float> angle_axis(rotate_angle, n_p);
//     rotate_mat = angle_axis.toRotationMatrix();
//     Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
//     transform(0,0)=rotate_mat(0,0);
//     transform(0,1)=rotate_mat(0,1);
//     transform(0,2)=rotate_mat(0,2);
//     transform(1,0)=rotate_mat(1,0);
//     transform(1,1)=rotate_mat(1,1);
//     transform(1,2)=rotate_mat(1,2);
//     transform(2,0)=rotate_mat(2,0);
//     transform(2,1)=rotate_mat(2,1);
//     transform(2,2)=rotate_mat(2,2);
//     //pcl::transformPointCloud (*cloud, *cloud_after_rota, transform);
// 	return transform;
// }
// double func(const MatrixXd& input,  const VectorXd& params, double objIndex)
// {
// 	// obj = A * sin(Bx) + C * cos(D*x) - F
// 	double x0 = params(0);
// 	double y0 = params(1);
// 	double xi = input(objIndex,0);
// 	double yi = input(objIndex,1);

// 	return sqrt(pow(xi-x0,2)+pow(yi-y0,2))-0.0225;
// }

// //return vector make up of func() element.
// VectorXd objF(const MatrixXd& input,  const VectorXd& params)
// {
// 	VectorXd obj(input.rows());
// 	for (int i = 0; i < input.rows(); i++)
// 		obj(i) = func(input, params, i);

// 	return obj;
// }

// //F = (f ^t * f)/2
// double Func(const VectorXd& obj)
// {
// 	//平方和，所有误差的平方和
// 	return obj.squaredNorm() / 2;
// }

// double Deriv(const MatrixXd& input,  int objIndex, const VectorXd& params,
// 	int paraIndex)
// {
// 	VectorXd para1 = params;
// 	VectorXd para2 = params;

// 	para1(paraIndex) -= DERIV_STEP;
// 	para2(paraIndex) += DERIV_STEP;

// 	double obj1 = func(input,  para1, objIndex);
// 	double obj2 = func(input,  para2, objIndex);

// 	return (obj2 - obj1) / (2 * DERIV_STEP);
// }

// MatrixXd Jacobin(const MatrixXd& input,  const VectorXd& params)
// {
// 	int rowNum = input.rows();
// 	int colNum = params.rows();

// 	MatrixXd Jac(rowNum, colNum);

// 	for (int i = 0; i < rowNum; i++)
// 	{
// 		for (int j = 0; j < colNum; j++)
// 		{
// 			Jac(i, j) = Deriv(input,  i, params, j);
// 		}
// 	}
// 	return Jac;
// }

// void gaussNewton(const MatrixXd& input,  VectorXd& params)
// {
// 	int errNum = input.rows();      //error  num
// 	int paraNum = params.rows();    //parameter  num

// 	VectorXd obj(errNum);

// 	double last_sum = 0;
//     // cout<<"m:"<<m<<endl;
// 	int iterCnt = 0;
// 	int qq=0;
// 	while (iterCnt < MAX_ITER)
// 	{
// 		//得到误差
// 		obj = objF(input,  params);

// 		double sum = 0;
// 		//误差平方和
// 		sum = Func(obj);

// 		// cout << "Iterator index: " << iterCnt << endl;
// 		// cout << "parameter: " << endl << params << endl;
// 		// cout << "error sum: " << endl << sum << endl << endl;

// 		//如果两次之间的误差小于1e-12，就算结束。
// 		if (fabs(sum - last_sum) <= 1e-12)
// 			break;
// 		//否则代替上次误差继续迭代
// 		last_sum = sum;

// 		MatrixXd Jac = Jacobin(input,  params);
// 		VectorXd delta(paraNum);
// 		delta = (Jac.transpose() * Jac).inverse() * Jac.transpose() * obj;
// 		// cout<<"sum"<<sum<<endl;
// 		// // if(qq==0)
// 		// // {
// 		// // 	qq=1;
// 		// // 	cout<<Jac * Jac	.transpose();
// 		// // }

// 		params -= delta;
// 		iterCnt++;
// 	}
// }
// vector<pcl::PointXYZ> nihe(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
// {
	
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rota(new pcl::PointCloud<pcl::PointXYZ>);//导入的点云
//     Eigen::Vector3f eigen_values,m;
//     m_pca(cloud,&eigen_values,&m);
// 	m.normalize();
	
// 	if(m(2)<0)
// 	{
// 		m(0)=-m(0);
// 		m(1)=-m(1);
// 		m(2)=-m(2);
// 	}
// 	Eigen::Matrix4f transform =Rotate_2_Z( m,  cloud);
// 	cloud->width=cloud->size();
// 	cloud->height=1;
// 	pcl::transformPointCloud (*cloud, *cloud_rota, transform);

//     pcl::PointXYZ min;//用于存放三个轴的最小值
//     pcl::PointXYZ max;//用于存放三个轴的最大值
//     pcl::getMinMax3D(*cloud_rota,min,max);
// 	int num_params = 2;
    
// 	//generate random data using these parameter
// 	int total_data = cloud_rota->size();

// 	MatrixXd input(total_data,2);

// 	double A = 5, B = 1, C = 10, D = 2;
// 	double mean_x,mean_y,mean_z;
// 	//load observation data
// 	for (int i = 0;i<cloud_rota->size(); i++)
// 	{
// 		//cout<<cloud_rota->points[i].x<<" "<<cloud_rota->points[i].y<<" "<<cloud_rota->points[i].z<<endl;
//         mean_x+=cloud_rota->points[i].x;
//         mean_y+=cloud_rota->points[i].y;
//         input(i,0)=cloud_rota->points[i].x;
//         input(i,1)=cloud_rota->points[i].y;
// 	}
// 	mean_x/=cloud_rota->size();
// 	mean_y/=cloud_rota->size();
   
// 	//gauss the parameters
// 	VectorXd params_gaussNewton(num_params);
// 	//init gauss
// 	params_gaussNewton << mean_x,mean_y;
 
// 	gaussNewton(input, params_gaussNewton);
    


// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);//导入的点云
// 	pcl::PointXYZ poi;
// 	poi.x=params_gaussNewton(0);
// 	poi.y=params_gaussNewton(1);
// 	float add=0;
// 	add=(max.z-min.z)/30;
// 	for(float i=min.z;i<max.z;i=i+add)
// 	{
//        poi.z=i;
// 	   cloud1->points.push_back(poi);
// 	}
// 	pcl::transformPointCloud (*cloud1, *cloud1, transform.inverse());
// 	// cloud1->width=cloud1->size();
// 	// cloud1->height=1;
//     // pcl::io::savePCDFileASCII("/home/shy/lidarcamera/knn/poi.pcd", *cloud1);
// 	cout << "gauss newton parameter: " << endl << params_gaussNewton << endl << endl << endl;

// 	float mid=(max.z+min.z)/2;
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);//导入的点云
// 	poi.z=min.z;
//     cloud2->points.push_back(poi);
// 	poi.z=mid;
//     cloud2->points.push_back(poi);
// 	poi.z=max.z;
//     cloud2->points.push_back(poi);
// 	pcl::transformPointCloud (*cloud2, *cloud2, transform.inverse());
// 	vector<pcl::PointXYZ>p_vector;
// 	if(cloud2->points[0].z>cloud2->points[1].z&&cloud2->points[0].z>cloud2->points[2].z&&cloud2->points[1].z>cloud2->points[2].z)
// 	{
// 		p_vector.push_back(cloud2->points[0]);
// 		p_vector.push_back(cloud2->points[1]);
// 		p_vector.push_back(cloud2->points[2]);
// 	}
// 	else if(cloud2->points[0].z>cloud2->points[1].z&&cloud2->points[0].z>cloud2->points[2].z&&cloud2->points[2].z>cloud2->points[1].z)
// 	{
// 		p_vector.push_back(cloud2->points[0]);
// 		p_vector.push_back(cloud2->points[2]);
// 		p_vector.push_back(cloud2->points[1]);
// 	}
// 	else if(cloud2->points[1].z>cloud2->points[0].z&&cloud2->points[1].z>cloud2->points[2].z&&cloud2->points[0].z>cloud2->points[2].z)
// 	{
// 		p_vector.push_back(cloud2->points[1]);
// 		p_vector.push_back(cloud2->points[0]);
// 		p_vector.push_back(cloud2->points[2]);
// 	}
// 	else if(cloud2->points[1].z>cloud2->points[0].z&&cloud2->points[1].z>cloud2->points[2].z&&cloud2->points[2].z>cloud2->points[0].z)
// 	{
// 		p_vector.push_back(cloud2->points[1]);
// 		p_vector.push_back(cloud2->points[2]);
// 		p_vector.push_back(cloud2->points[0]);
// 	}
// 	else if(cloud2->points[2].z>cloud2->points[0].z&&cloud2->points[2].z>cloud2->points[1].z&&cloud2->points[0].z>cloud2->points[1].z)
// 	{
// 		p_vector.push_back(cloud2->points[2]);
// 		p_vector.push_back(cloud2->points[0]);
// 		p_vector.push_back(cloud2->points[1]);
// 	}
// 	else 
// 	{
// 		p_vector.push_back(cloud2->points[2]);
// 		p_vector.push_back(cloud2->points[1]);
// 		p_vector.push_back(cloud2->points[0]);
// 	}
// 	return p_vector;
// }



