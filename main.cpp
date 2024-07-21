#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>
#include "DBSCAN_simple_quick.h"
#include "DBSCAN_simple.h"
#include <iostream>
#include <vector>
#include <stdio.h>
#include<iomanip> 
#include <map>
#include <thread>
#include <chrono>
#include "file.h"
#include "mine.h"
#include <pcl/registration/icp.h> // icp算法
#include "ransac.h"
// #include "voxel_cluster.h"
#include <variant>
using namespace std;
using namespace Eigen;

static float min_x=0;
static float min_y=-40;
static float max_x=70;
static float max_y=40;
static float min_height=1.5;
float ground_angel=20;                 //地面点角度阈值
float ground_distance_threhold=0.2;    //地面点距离阈值
float deltaz=0.1;                      //区间间隔deltaz
float pole_and_notpole_distance_threhold = 0.4;   //判断是否为非柱状物的半径阈值
float pole_max_distance=70;             //柱体距离雷达的距离范围
float PCA_Angle_Threhold=8;           //PCA角度阈值
float PCA_Rate_Threhold_Alpha=0;       //PCA比例阈值
float CTP_Searching_KDtree_Threhold=0.2; //kdtree距离阈值，尽量小
float CPP_Searching_KDtree_Threhold=0.3; //kdtree距离阈值，尽量小
float leaf_size=0.05;
float thredhold_of_cluster=0;
string filename="simulator_data/last仿真9";              //文件名
// static float meanall[8]={0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5};
float distan3D(pcl::PointXYZ X,float x,float y)
{
   return sqrt(pow(X.x-x,2)+pow(X.y-y,2));
}
float distan(pcl::PointXYZ X,pcl::PointXYZ Y)
{
    // cout<<sqrt(pow(X.x-Y.x,2)+pow(X.y-Y.y,2))<<endl;
    return sqrt(pow(X.x-Y.x,2)+pow(X.y-Y.y,2));
}
float findmax(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // cout<<cloud->size()<<endl;
    float max = 0;
    for(int i=0;i<cloud->size()-1;i++)
    {
      for(int j=i+1;j<cloud->size();j++)
      {
         if(distan(cloud->points[i],cloud->points[j])>max)
         {
            max = distan(cloud->points[i],cloud->points[j]);
         }
      }
      return max;
    }
}


void expro(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,float lev, unordered_map<int,vector<pcl::PointCloud<pcl::PointXYZ>>>&  pole , 
                        unordered_map<int,vector<vector<float>>>&  pole_cluster_mean,pcl::PointCloud<pcl::PointXYZ>::Ptr non_pole)
{

    unordered_map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr> graph_map;//按z轴划分点云，每个z【num】存储的是区间num内的点云
    for(int i=0;i<cloud->size();i++)
    {
        cloud->points[i].z-=lev;
        float z=cloud->points[i].z;
        if(z>0)
        {
           int num=z/deltaz;
           if(graph_map.count(num)==0)
           {
              pcl::PointCloud<pcl::PointXYZ>::Ptr p(new pcl::PointCloud<pcl::PointXYZ>);
              p->points.push_back(cloud->points[i]);
              graph_map.insert(make_pair(num,p));
           }
           else
           {
              graph_map[num]->points.push_back(cloud->points[i]);
           }
        }     
    }
    
    unordered_map<int,pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator iter;
    iter = graph_map.begin();
    while(iter != graph_map.end()) {                                                        //遍历graph_map
        int index= iter->first;
        vector<pcl::PointCloud<pcl::PointXYZ>> pole_cluster_each_interval;                  //用于存储每个区间的pole cluster
        vector<vector<float>> pole_cluster_mean_each_interval;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_it(new pcl::PointCloud<pcl::PointXYZ>);  //整个场景点云每个区间的点云
        *cloud_it=*(iter->second);
        std::vector<pcl::PointIndices> cluster_target;                                      //保存符合条件的平面的点云
        //以下注释的部分是可视化存储部分
        cluster_target=dbscan_simple_quick(cloud_it);                                       //对每个区间进行快速聚类，这里的聚类无视高度，且距离阈值比较大，数量较小
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity(new pcl::PointCloud<pcl::PointXYZI>);
        for(int d=0;d<cluster_target.size();d++)
        {
            // double IN=255*(1024*rand()/(RAND_MAX+1.0f));
            pcl::PointCloud<pcl::PointXYZ>::Ptr obj(new pcl::PointCloud<pcl::PointXYZ>);    //每个区间的不同聚类
            // pcl::PointCloud<pcl::PointXYZI>::Ptr objI(new pcl::PointCloud<pcl::PointXYZI>);
            float mean_x=0;
            float mean_y=0;
            for(int n=0;n<cluster_target[d].indices.size();n++)
            {
                obj->points.push_back(cloud_it->points[cluster_target[d].indices[n]]);
                mean_x+=cloud_it->points[cluster_target[d].indices[n]].x;
                mean_y+=cloud_it->points[cluster_target[d].indices[n]].y;
            }

            pcl::PointXYZ min;//用于存放三个轴的最小值
            pcl::PointXYZ max;//用于存放三个轴的最大值
            pcl::getMinMax3D(*obj,min,max);
            if(findmax(obj)<pole_and_notpole_distance_threhold)                             //判断聚类是否大于这个值，如果小于，暂定为pole，大于，则为非柱状物not_pole
            {
                mean_x/=obj->size();
                mean_y/=obj->size();
                // *pole+=*obj;
                pole_cluster_each_interval.push_back(*obj);
                vector<float> mean;                                 //存储每个cluster的均值
                mean.push_back(mean_x);
                mean.push_back(mean_y);
                pole_cluster_mean_each_interval.push_back(mean);
                // pcl::io::savePCDFileASCII("/home/shy/lidarcamera/teacher_test_2（复件）/result/ceng/"+to_string(d)+".pcd", *cloud_intensity); //带强度的点云
            }
            else{
                *non_pole+=*obj;
            }
        }
        pole[index]=pole_cluster_each_interval;
        pole_cluster_mean[index]=pole_cluster_mean_each_interval;

        iter++;
    }
}
void extractlay(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::vector<pcl::PointIndices> cluster_target,float lev,vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &obj_vector,
                                                                                                                 vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& not_vector,
                                                                                                                 vector<float> &mean_x_vector,vector<float> &mean_y_vector,const int& ll)
                                                                                                                //  vector<float> &pca_vector,vector<float> &angle_vector,vector<float> &max_vector)
{
    unordered_map<int,vector<pcl::PointCloud<pcl::PointXYZ>>>  pole_cluster;
    unordered_map<int,vector<vector<float>>>  pole_cluster_mean;
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_pole(new pcl::PointCloud<pcl::PointXYZ>);
    expro(cloud,lev,pole_cluster,pole_cluster_mean,non_pole);//！！！！！！！！！！！！！！！！！！！！！！
    // pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/result/nonpole/"+to_string(ll)+".pcd", *non_pole); 

    unordered_map<int,vector<pcl::PointCloud<pcl::PointXYZ>>>::iterator iter;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pole__all(new pcl::PointCloud<pcl::PointXYZ>);
    iter = pole_cluster.begin();
    while(iter!=pole_cluster.end())
    {
        for(int c=0;c<iter->second.size();c++)
        {
            *pole__all+=iter->second[c];
        }
        iter++;
    }
    // pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/result/pole/"+to_string(ll)+".pcd", *pole__all); 

    vector<pcl::PointCloud<pcl::PointXYZ>> pole_vector;
    vector<vector<float>> mean_vector;
    vector<vector<int>> num_vector;

    dbscan_simple(pole_cluster,pole_cluster_mean,pole_vector,mean_vector,num_vector);
    for(int p=0;p<pole_vector.size();p++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pole_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *pole_cloud=pole_vector[p];
        // pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/result/pole/"+to_string(ll)+"/"+to_string(p)+".pcd", *pole_cloud); 

        float mean_x=mean_vector[p][0];
        float mean_y=mean_vector[p][1];
        vector<int>num_sort=num_vector[p];
        sort(num_sort.begin(),num_sort.end());
        float rate_of_num=(float)num_sort.size()/(num_sort.back()-num_sort[0]);
        
        cout<<num_sort.size()<<"    "<<num_sort[0]<<"   "<<num_sort.back()<<"   "<<rate_of_num<<"   ";
        for(int i =0;i<num_sort.size();i++)
        {
            cout<<num_sort[i]<<"   ";
        }
        if(rate_of_num<thredhold_of_cluster)
        {
            continue;
        }
        // pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/result/rate/"+to_string(ll)+"/"+to_string(p)+".pcd", *pole_cloud); 

        cout<<endl;
        Eigen::Vector3f eigen_values,m,c;
        m_pca(pole_cloud,&eigen_values,&m);

        m(0)=abs(m(0));
        m(1)=abs(m(1));
        m(2)=abs(m(2));

        c<<0,0,1;
        float angle=pcl::getAngle3D(m,c,true); //计算pca的角度
        pcl::PointXYZ min;//用于存放三个轴的最小值
        pcl::PointXYZ max;//用于存放三个轴的最大值
        pcl::getMinMax3D(*pole_cloud,min,max);
        float dis=sqrt(pow(mean_x,2)+pow(mean_y,2));

        if(findmax(pole_cloud)>=0.9)          //论文以外的保险措施，防止程序哪里出错造成提取出的pole半径过大，findmax函数是取pole_cloud的最大半径
        {
            continue;
        }
        else if(max.z-min.z>min_height &&angle<PCA_Angle_Threhold&&dis<pole_max_distance&&eigen_values[0]/eigen_values[1]>PCA_Rate_Threhold_Alpha)
        {

            // cout<<eigen_values(0)/eigen_values(1)<<" "<<eigen_values(0)/eigen_values(2)<<endl;
            mean_x_vector.push_back(mean_x);   //     把符合条件的pole的均值u放入meanx和meany容器中
            mean_y_vector.push_back(mean_y);   //
            obj_vector.push_back(pole_cloud);  //----------------------------------------------------------------------- //把符合条件的pole放入obj_vector容器中
        }
    }

}



float remove_ground(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_low,pcl::ModelCoefficients::Ptr coefficients)
{
	pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
    
    ransac(cloud_low,cloud_ground,coefficients,ground_angel,ground_distance_threhold);
    
    	  //---------------------KD树半径搜索-------------------
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  
	kdtree.setInputCloud(cloud);
	vector<int> pointIdxRadiusSearch;          //保存每个近邻点的索引
	vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
	vector<int> total_index;
	float radius = 0.1;//若两点之间的距离为0.000001则认为是重合点


    /*若某一点在0.000001领域内不止其本身一个点，则认为其有重复点。
    将重复点的索引记录下来，由于后续以此重复点为查询点搜索时，此时这一点也会被定义为重复点，
    但pointIdxRadiusSearch中都是升序排列的，故从pointIdxRadiusSearch中的第二个点的索引开始记录，
    这样可以保证仅仅删除重复的点，保留第一个点*/
	for (size_t i = 0; i < cloud_ground->size(); ++i)//对cloud中的每个点与邻域内的点进行比较
	{
        pcl::PointXYZ searchPoint = cloud_ground->points[i]; 

	    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	    {
		  for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++)
		  {
			total_index.push_back(pointIdxRadiusSearch[j]);
		  }
	    }
	}
	  //-----------------------删除重复索引-----------------------
	sort(total_index.begin(), total_index.end());//将索引进行排序
	total_index.erase(unique(total_index.begin(), total_index.end()), total_index.end());//将索引中的重复索引去除

	  //-------------------根据索引删除重复的点-------------------
	pcl::PointIndices::Ptr outliners(new pcl::PointIndices());
	outliners->indices.resize(total_index.size());
	for (size_t i = 0; i < total_index.size(); i++)
	{
        outliners->indices[i] = total_index[i];
	}
	  //-------------------提取删除重复点之后的点云--------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(outliners);
	extract.setNegative(true);//设置为true则表示保存索引之外的点
	extract.filter(*cloud_filtered);
    *cloud=*cloud_filtered;

    float lev=0;
    Rotate_2_Z(coefficients,cloud_ground,cloud_ground,10001);
    for(int ll=0;ll<cloud_ground->size();ll++)
    {
       lev+=cloud_ground->points[ll].z;
    }
    lev=lev/cloud_ground->size();
    for(int ll=0;ll<cloud_ground->size();ll++)
    {
       cloud_ground->points[ll].z-=lev;
    }
    pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/minetest/result/ground.pcd", *cloud_ground); //去除地面 回正以后的点云
    return lev;
}
void x_y_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_test ,float x,float y,float z)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x"); //滤波字段名被设置为Z轴方向s
  pass.setFilterLimits (0,x); //设置在过滤方向上的过滤范围
  // pass.setKeepOrganized(true); // 保持有序点云结构，该功能用于有序点云才有意义。
  pass.setNegative (false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
  pass.filter (*cloud_filtered);
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y"); //滤波字段名被设置为Z轴方向
  pass.setFilterLimits (-y, y); //设置在过滤方向上的过滤范围
  // pass.setKeepOrganized(true); // 保持有序点云结构，该功能用于有序点云才有意义。
  pass.setNegative (false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
  pass.filter (*cloud_filtered);
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z"); //滤波字段名被设置为Z轴方向
  pass.setFilterLimits (-6, z); //设置在过滤方向上的过滤范围    groundZ
  // pass.setKeepOrganized(true); // 保持有序点云结构，该功能用于有序点云才有意义。
  pass.setNegative (false); //设置保留范围内的点还是过滤掉范围内的点，标志为false时保留范围内的点
  pass.filter (*cloud_test);
}
vector<vector<int>> sort_num(int u0,int u1,int v0,int v1,int w0,int w1)
{
    vector<vector<int>> sort;
    vector<int>vec0,vec1;
    if(u1<v1&&u1<w1&&v1<w1) //u v w
    {
        vec0.push_back(u0);vec0.push_back(v0);vec0.push_back(w0);
        vec1.push_back(u1);vec1.push_back(v1);vec1.push_back(w1);
    }
    else if (u1<w1&&u1<v1&&w1<v1) //u w v
    {
        vec0.push_back(u0);vec0.push_back(w0);vec0.push_back(v0);
        vec1.push_back(u1);vec1.push_back(w1);vec1.push_back(v1);
    }
    else if (v1<u1&&v1<w1&&u1<w1)//v u w
    {
        vec0.push_back(v0);vec0.push_back(u0);vec0.push_back(w0);
        vec1.push_back(v1);vec1.push_back(u1);vec1.push_back(w1);
    }
    else if (v1<u1&&v1<w1&&w1<u1)//v w u
    {
        vec0.push_back(v0);vec0.push_back(w0);vec0.push_back(u0);
        vec1.push_back(v1);vec1.push_back(w1);vec1.push_back(u1);
    }    
    else if (w1<u1&&w1<v1&&u1<v1)//w u v
    {
        vec0.push_back(w0);vec0.push_back(u0);vec0.push_back(v0);
        vec1.push_back(w1);vec1.push_back(u1);vec1.push_back(v1);
    }
    else if (w1<v1&&w1<u1&&v1<u1)//w v u 
    {
        vec0.push_back(w0);vec0.push_back(v0);vec0.push_back(u0);
        vec1.push_back(w1);vec1.push_back(v1);vec1.push_back(u1);
    }
    else 
    {
        cerr<<"sort wrong"<<endl;
    }
    sort.push_back(vec0);
    sort.push_back(vec1);
    return sort;
}

int main()
{
    srand((unsigned)time(NULL));
    string pcd_file1 = "/home/shy/1lidarcamera/minetest/test/"+filename+"/0";
    string pcd_file2 = "/home/shy/1lidarcamera/minetest/test/"+filename+"/1";
    File ob1,ob2;
    ob1.set(pcd_file1);
    ob2.set(pcd_file2);
    vector<string> pcd_path1 = ob1.get_pcd();
    vector<string> pcd_path2 = ob2.get_pcd();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);//导入的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);//导入的点云
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector;
    for(int dd=0;dd<pcd_path1.size();++dd)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//导入的点云
        pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path1[dd], *cloud);
        *cloud_source+=*cloud;
        x_y_filter(cloud_source,cloud_source,150,150,5);
        GridSampling(cloud_source,leaf_size); 

    }
    for(int dd=0;dd<pcd_path2.size();++dd)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//导入的点云
        pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path2[dd], *cloud);
        *cloud_target+=*cloud;
        x_y_filter(cloud_target,cloud_target,150,150,5);

        GridSampling(cloud_target,leaf_size); 


    }
    cloud_vector.push_back(cloud_source);
    cloud_vector.push_back(cloud_target);



/////////// a  cloud_source   cloud_target cloud_vector//////////////
    auto start_time = std::chrono::high_resolution_clock::now();



    /*1:两个点云提取pole阶段，记录pole :obj_vector_two       not pole:not_vector_two    pole的均值，   以及pole之间的距离:inside_distance_vector(矩阵形式)*/
    vector<vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> obj_vector_two,not_vector_two;
    vector<vector<vector<float>>> inside_distance_vector,mean_value_two;
    for(int i=0;i<cloud_vector.size();i++)//START
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//导入的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_low(new pcl::PointCloud<pcl::PointXYZ>);//导入的点云
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//保存地面的平面参数
        *cloud=*cloud_vector[i];
        x_y_filter(cloud,cloud_low,200,200,0);
        float lev=remove_ground(cloud,cloud_low,coefficients);
        x_y_filter(cloud,cloud,200,200,20);


        Rotate_2_Z(coefficients,cloud,cloud,i);//去除地面的点云旋转
        Rotate_2_Z(coefficients,cloud_vector[i],cloud_vector[i],i);//原先点云旋转.
        for(int sor=0;sor<cloud_vector[i]->size();sor++)
        {
            cloud_vector[i]->points[sor].z-=lev;
        }
        pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/noground/"+to_string(i)+".pcd", *cloud); 
        pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/test/"+to_string(i)+".pcd", *cloud_vector[i]); 
        // pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/test/"+to_string(i)+".pcd", *cloud_vector[i]); 

        // cout<<"已保存原图及去除地面点的图"<<endl;
        std::vector<pcl::PointIndices> cluster_target;
        // cluster_target=dbscan_simple(cloud,0);        //可聚类可不聚类，论文中没有聚类,没聚类更快，聚类更准


        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obj_vector,not_vector;
        vector<float> mean_x_vector,mean_y_vector;
        vector<vector<float>> mean_vector;
        extractlay(cloud,cluster_target,lev,obj_vector,not_vector,mean_x_vector,mean_y_vector,i);//！！！！！！！！！！！！！关键
        mean_vector.push_back(mean_x_vector);
        mean_vector.push_back(mean_y_vector);

        mean_value_two.push_back(mean_vector);
        obj_vector_two.push_back(obj_vector);
        not_vector_two.push_back(not_vector);  //这个是空的，因为不对notpole处理
        
        //保存点云内每两个pole的距离,其中p<q    距离矩阵：
        vector<vector<float>>  inside_distance(obj_vector.size(),vector<float>(obj_vector.size()));// == vector inside_distance[obj_vector.size()][obj_vector.size()]
        for(int p=0;p<obj_vector.size()-1;p++)
        {
            for(int q=p+1;q<obj_vector.size();q++ )
            {
                float x1=mean_x_vector[p];
                float x2=mean_x_vector[q];
                float y1=mean_y_vector[p];
                float y2=mean_y_vector[q];
                float distance=sqrt(pow(x1-x2,2)+pow(y1-y2,2));
                inside_distance[p][q]=distance; //确保inside_distance是对称矩阵，没有这一步在后续需要判断p>q,不方便
                inside_distance[q][p]=distance;
            }
        }
        inside_distance_vector.push_back(inside_distance);
    }
    
        for(int j=0;j<obj_vector_two[0].size();j++)
    {
        double IN=255*(1024*rand()/(RAND_MAX+1.0f));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr OBJ(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl::transformPointCloud (*obj_vector_two[0][j], *cloud0, T_vector[0]);//第一个雷达进行旋转
        *cloud0=*obj_vector_two[0][j];
        for(int n=0;n<cloud0->size();n++)
        {
            pcl::PointXYZI tmp;
            tmp.x=cloud0->points[n].x;
            tmp.y=cloud0->points[n].y;
            tmp.z=cloud0->points[n].z;
            tmp.intensity=IN;
            OBJ->points.push_back(tmp);
        }
        OBJ->width=OBJ->size();
        OBJ->height=1;
        pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/result/objall/"+to_string(0)+"/obj"+to_string(j)+".pcd", *OBJ); 
           
    }
    for(int j=0;j<obj_vector_two[1].size();j++)
    {
        double IN=255*(1024*rand()/(RAND_MAX+1.0f));
        pcl::PointCloud<pcl::PointXYZI>::Ptr OBJ(new pcl::PointCloud<pcl::PointXYZI>);

        for(int n=0;n<obj_vector_two[1][j]->size();n++)
        {
            pcl::PointXYZI tmp;
            tmp.x=obj_vector_two[1][j]->points[n].x;
            tmp.y=obj_vector_two[1][j]->points[n].y;
            tmp.z=obj_vector_two[1][j]->points[n].z;
            tmp.intensity=IN;
            OBJ->points.push_back(tmp);
        }
        OBJ->width=OBJ->size();
        OBJ->height=1;
        pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/result/objall/"+to_string(1)+"/obj"+to_string(j)+".pcd", *OBJ); 
           
    }


    //输出两个距离矩阵
    vector<vector<float>> inside_distance0=inside_distance_vector[0];
    vector<vector<float>> inside_distance1=inside_distance_vector[1];
    cout<<inside_distance0.size()<<"个Poles"<<endl;
    cout<<"距离矩阵 ： "<<endl;
    for(int i=0;i<inside_distance0.size();i++)
    {

        for(int j=0;j<inside_distance0[i].size();j++)
        {
            cout<<inside_distance0[i][j]<<" , ";
        }
        cout<<endl;
    }
    cout<<endl;
    cout<<inside_distance1.size()<<"个Poles"<<endl;
    cout<<"距离矩阵 ： "<<endl;
    for(int i=0;i<inside_distance1.size();i++)
    {
        for(int j=0;j<inside_distance1[i].size();j++)
        {
            cout<<inside_distance1[i][j]<<" , ";
        }
        cout<<endl;
    }
    cout<<endl;


    /*2:编码阶段，将每三个pole均值的距离分别作为 x,y,z   分别对应Target中的 pq、pr 、qr,   R G B 对应 p q r */
    /*需要用到每两个pole的距离矩阵:inside_distance_vector    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr distance3D_0(new pcl::PointCloud<pcl::PointXYZRGB>); //构成kdtree
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr distance3D_1(new pcl::PointCloud<pcl::PointXYZRGB>); //查询
    float m_e=200; //30 m以内 
    for(int p=0;p<inside_distance0.size()-2;p++)
    {
        for(int q=p+1;q<inside_distance0.size()-1;q++)
        {
            if(inside_distance0[p][q]<m_e)
            {
                for(int r=q+1;r<inside_distance0.size();r++)
                {
                    if(inside_distance0[p][r]<m_e&&inside_distance0[q][r]<m_e)
                    {
                        pcl::PointXYZRGB point3d;
                        //第一种情况:如果p0 q0 r0 对应 p1 q1 r1，那么edge(p0,q0)和edge(p1,q1)，edge(p0,r0)和edge(p1,r1),edge(q0,r0)和edge(q1,r1)分别相近                       
                        point3d.x=inside_distance0[p][q]; // point[pqr]=(edge(pq),edge(pr),edge(qr))
                        point3d.y=inside_distance0[p][r];
                        point3d.z=inside_distance0[q][r];
                        point3d.r=p;      //记录顺序
                        point3d.g=q;
                        point3d.b=r;
                        distance3D_0->points.push_back(point3d);

                        //第二种情况:如果p0 r0 q0 对应 p1 q1 r1，那么edge(p0,r0)和edge(p1,q1)，edge(p0,q0)和edge(p1,r1),edge(q0,r0)和edge(q1,r1)分别相近  
                        point3d.x=inside_distance0[p][r];  // point[prq]=(edge(pr),edge(pq),edge(qr))
                        point3d.y=inside_distance0[p][q];
                        point3d.z=inside_distance0[q][r];
                        point3d.r=p;      //记录顺序
                        point3d.g=r;
                        point3d.b=q;
                        distance3D_0->points.push_back(point3d);

                        //第三种情况:如果q0 p0 r0 对应 p1 q1 r1，那么edge(p0,q0)和edge(p1,q1)，edge(q0,r0)和edge(p1,r1),edge(p0,r0)和edge(q1,r1)分别相近  
                        point3d.x=inside_distance0[p][q];  // point[qpr]=(edge(pq),edge(qr),edge(pr))
                        point3d.y=inside_distance0[q][r];
                        point3d.z=inside_distance0[p][r];
                        point3d.r=q;      //记录顺序
                        point3d.g=p;
                        point3d.b=r;
                        distance3D_0->points.push_back(point3d);

                        //第4种情况:如果q0 r0 p0 对应 p1 q1 r1，那么edge(q0,r0)和edge(p1,q1)，edge(p0,q0)和edge(p1,r1),edge(p0,r0)和edge(q1,r1)分别相近  
                        point3d.x=inside_distance0[q][r];  // point[qrp]=(edge(qr),edge(pq),edge(pr))
                        point3d.y=inside_distance0[p][q];
                        point3d.z=inside_distance0[p][r];
                        point3d.r=q;      //记录顺序
                        point3d.g=r;
                        point3d.b=p;
                        distance3D_0->points.push_back(point3d);

                        //第5种情况:如果r0 p0 q0 对应 p1 q1 r1，那么edge(p0,r0)和edge(p1,q1)，edge(q0,r0)和edge(p1,r1),edge(p0,q0)和edge(q1,r1)分别相近  
                        point3d.x=inside_distance0[p][r];  // point[rpq]=(edge(pr),edge(qr),edge(pq))
                        point3d.y=inside_distance0[q][r];
                        point3d.z=inside_distance0[p][q];
                        point3d.r=r;      //记录顺序
                        point3d.g=p;
                        point3d.b=q;
                        distance3D_0->points.push_back(point3d);

                        //第6种情况:如果r0 q0 p0 对应 p1 q1 r1，那么edge(q0,r0)和edge(p1,q1)，edge(p0,r0)和edge(p1,r1),edge(p0,q0)和edge(q1,r1)分别相近  
                        point3d.x=inside_distance0[q][r];  // point[rqp]=(edge(qr),edge(pr),edge(pq))
                        point3d.y=inside_distance0[p][r];
                        point3d.z=inside_distance0[p][q];
                        point3d.r=r;      //记录顺序
                        point3d.g=q;
                        point3d.b=p;
                        distance3D_0->points.push_back(point3d);
                    }
                }
            }
            // else{
            //     NUM+=
            // }
        }
    }
    for(int p=0;p<inside_distance1.size()-2;p++)
    {
        for(int q=p+1;q<inside_distance1.size()-1;q++)
        {
            if(inside_distance1[p][q]<m_e)
            {
                for(int r=q+1;r<inside_distance1.size();r++)
                {
                    if(inside_distance1[p][r]<m_e&&inside_distance1[q][r]<m_e)
                    {
                        pcl::PointXYZRGB point3d;
                        //1种情况                  
                        point3d.x=inside_distance1[p][q]; // point[pqr]=(edge(pq),edge(pr),edge(qr))
                        point3d.y=inside_distance1[p][r];
                        point3d.z=inside_distance1[q][r];
                        point3d.r=p;      //记录顺序
                        point3d.g=q;
                        point3d.b=r;
                        distance3D_1->points.push_back(point3d);
                    }
                }
            }
        }
    }
    cout<<"kdtree上的点数量："<<endl;
    cout<<distance3D_0->size()<<endl;
    cout<<"查询的点数量："<<endl;
    cout<<distance3D_1->size()<<endl<<endl;



    /*3:确定对应关系， 3个P1对应一组3个3个的P0   KDtree0*/
    /*需要用到距离的3d位置关系: distance3D_0、distance3D_1*/
    std::map<std::tuple<int, int, int>, vector<vector<int>>> mymap1_0;//mymap1_0[p1 q1 r1]=vector{(p0,q0,r0)  (p0,q0r0)  (p0,q0r0)}
    pcl::KdTreeFLANN<pcl::PointXYZRGB> tree;
    tree.setInputCloud(distance3D_0);
    for(int j=0;j<distance3D_1->size();j++)
    {
        vector<int> pointIdxRadiusSearch;          //保存每个近邻点的索引
	    vector<float> pointRadiusSquaredDistance;
        if(tree.radiusSearch(distance3D_1->points[j],CTP_Searching_KDtree_Threhold,pointIdxRadiusSearch,pointRadiusSquaredDistance,0)>0)
        {
            int p=distance3D_1->points[j].r;
            int q=distance3D_1->points[j].g;
            int r=distance3D_1->points[j].b;
            vector<vector<int>>v;
            for(int u=0;u<pointIdxRadiusSearch.size();u++)
            {
               //映射关系
               vector<int> num0;// p0 q0 r0
               num0.push_back(distance3D_0->points[pointIdxRadiusSearch[u]].r);
               num0.push_back(distance3D_0->points[pointIdxRadiusSearch[u]].g);
               num0.push_back(distance3D_0->points[pointIdxRadiusSearch[u]].b);
               v.push_back(num0);
            }
            mymap1_0[std::make_tuple(p, q, r)] =v;
        }
    }

    
    /*4:遍历对应关系，并拓展子图，同时删除重复对应关系  KDtree1*/
    /*需要用到每两个pole的距离矩阵:inside_distance0、inside_distance0,以及对应关系map:mymap1_0*/
    vector<vector<int>> S0,T1;  //
    for (auto iter = mymap1_0.begin(); iter != mymap1_0.end(); ++iter) {
        std::tuple<int, int, int> key = iter->first;
        vector<vector<int>> value = iter->second;
        if(value.size()==0)
        {
            // cout<<"map wrong !"<<endl;
            continue;
        }
        int p1,q1,r1;
        p1=get<0>(key);
        q1=get<1>(key);
        r1=get<2>(key);

        pcl::PointCloud<pcl::PointXYZI>::Ptr distance_to3pole_1(new pcl::PointCloud<pcl::PointXYZI>);
        for(int o1=0;o1<inside_distance1.size();o1++)
        {
            if(o1==p1||o1==q1||o1==r1)
            {
                continue;
            }
            PointXYZI worker;
            worker.x=inside_distance1[o1][p1];
            worker.y=inside_distance1[o1][q1];
            worker.z=inside_distance1[o1][r1];
            worker.intensity=o1;
            distance_to3pole_1->points.push_back(worker);
        }
        pcl::KdTreeFLANN<pcl::PointXYZI> tree_for_exten;
        tree_for_exten.setInputCloud(distance_to3pole_1);
        for(int i=0;i<value.size();i++)
        {  
            int p0,q0,r0;
            p0=value[i][0];
            q0=value[i][1];
            r0=value[i][2];
            vector<int>s0;
            s0.push_back(p0);
            s0.push_back(q0);
            s0.push_back(r0);
            vector<int>t1;
            t1.push_back(p1);
            t1.push_back(q1);
            t1.push_back(r1);
            for(int o0=0;o0<inside_distance0.size();o0++)//kd tree寻找其他匹配pole
            {
               if(o0==p0||o0==q0||o0==r0)
               {
                  continue;
               }               
               vector<int> pointIdxRadiusSearch;          //保存每个近邻点的索引
	           vector<float> pointRadiusSquaredDistance;                
               PointXYZI worker;
               worker.x=inside_distance0[o0][p0];
               worker.y=inside_distance0[o0][q0];
               worker.z=inside_distance0[o0][r0];
               if(tree_for_exten.radiusSearch(worker,CPP_Searching_KDtree_Threhold,pointIdxRadiusSearch,pointRadiusSquaredDistance,0)>0)
               {
                //    if(pointIdxRadiusSearch.size()>1)
                //    {
                //       std::cerr << "itera warning !" << std::endl;
                //     //   std::exit(1);
                //    }
                //    else
                //    {
                      int o1=distance_to3pole_1->points[pointIdxRadiusSearch[0]].intensity;
                      s0.push_back(o0);
                      t1.push_back(o1);
                      
                //    }
               }  
            }
            //debug:
            int P_L0=0,P_L1=0;
            cout<<"s0:"<<endl;
            while(P_L0<s0.size())
            {
                cout<<s0[P_L0]<<" ,";
                P_L0++;
            }
            cout<<endl;

            cout<<"t1:"<<endl;
            while(P_L1<t1.size())
            {
                cout<<t1[P_L1]<<" ,";
                P_L1++;
            }
            cout<<endl;
            cout<<endl;
            S0.push_back(s0);
            T1.push_back(t1);
            //删除map1_0中后续部分的对应三角形
            for(int a=0;a<s0.size()-2;a++)
            {
               for(int b=a+1;b<s0.size()-1;b++)
               {
                 for(int c=b+1;c<s0.size();c++)
                 {
                    if(a==0&&b==1&&c==2)
                    {
                        continue;
                    }
                    
                    int u0=s0[a]; int v0=s0[b];int w0=s0[c];
                    int u1=t1[a]; int v1=t1[b];int w1=t1[c];
                    vector<vector<int>> sort=sort_num(u0,u1,v0,v1,w0,w1);//u1 v1 w1 按大小排序，符合map1_0
                    u0=sort[0][0];v0=sort[0][1];w0=sort[0][2];
                    u1=sort[1][0];v1=sort[1][1];w1=sort[1][2];                
                    std::tuple<int, int, int> key=make_tuple(u1,v1,w1);
                    if (mymap1_0.find(key) != mymap1_0.end())
                    {
                        vector<vector<int>> maybe_del;
                        maybe_del=mymap1_0[key];
                        for(int mm=0;mm<mymap1_0[key].size();mm++)
                        {
                            if(maybe_del[mm][0]==u0&&maybe_del[mm][1]==v0&&maybe_del[mm][2]==w0)
                            {
                                mymap1_0[key].erase(mymap1_0[key].begin()+mm);
                            }
                        }
                    }

                 }
               }
            }
        }
    }
   
   
    
    /*5.配准，计算得分（没有了）*/
    vector<Eigen::Affine3d> T_vector;
    Mine m(mean_value_two);
    pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/最终结果/target.pcd", *cloud_vector[1]); 

            //--------------------初始化ICP对象--------------------
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            //---------------------KD树加速搜索--------------------
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    icp.setSearchMethodSource(tree1);
    icp.setSearchMethodTarget(tree2);
    icp.setTransformationEpsilon(1e-3);   // 为终止条件设置最小转换差异
    icp.setMaxCorrespondenceDistance(0.1);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
    icp.setEuclideanFitnessEpsilon(0.2);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
    icp.setMaximumIterations(10000);           // 最大迭代次数
    for(int n=0;n<S0.size();n++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);//导入的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_P0(new pcl::PointCloud<pcl::PointXYZ>);//存P0出来的柱状物
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_T1(new pcl::PointCloud<pcl::PointXYZ>);//存T1出来的柱状物
        double err=0;
        // float result=0;
        if(S0.size()!=T1.size())
        {
            cout<<"S0!=T1?"<<endl;
        }
        vector<int>s0,t1;
        s0=S0[n];t1=T1[n];
        m.setInputNum(s0,t1);
        Eigen::Affine3d T_horizon,T_icp,T_final;
        m.getTransformMatrix(T_horizon);
        // 提取旋转部分
        Eigen::Matrix3d rotation_matrix = T_horizon.linear();
        // 计算旋转矩阵的欧拉角
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(0, 1, 2);
        double roll_degrees = euler_angles(1) * 180.0 / M_PI; // Roll
        double pitch_degrees = euler_angles(0) * 180.0 / M_PI; // Pitch
        // double yaw_degrees = euler_angles(0) * 180.0 / M_PI; // Pitch
        // 检查横滚角和俯仰角是否超过60度
        if (std::abs(roll_degrees) > 90.0 || std::abs(pitch_degrees) > 90.0) {
            continue;
        }
        

        for(int j=0;j<s0.size();j++)
        {
           *cloud_P0+=*obj_vector_two[0][s0[j]];
        }
        pcl::transformPointCloud (*cloud_P0, *cloud_P0, T_horizon);//第一个雷达进行旋转

        for(int j=0;j<t1.size();j++)
        {
            *cloud_T1+=*obj_vector_two[1][t1[j]];
        }
        pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/result/obj/第"+to_string(n)+"次的0.pcd", *cloud_P0); 
        pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/result/obj/第"+to_string(n)+"次的1.pcd", *cloud_T1); 



        tree1->setInputCloud(cloud_P0);

        tree2->setInputCloud(cloud_T1);

        //----------------------icp核心代码--------------------
        icp.setInputSource(cloud_P0);            // 源点云
        icp.setInputTarget(cloud_T1);            // 目标点云

        pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	    icp.align(*icp_cloud);
        //icp.setUseReciprocalCorrespondences(true);//使用相互对应关系
        // 计算需要的刚体变换以便将输入的源点云匹配到目标点云
        // pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // cout << "\nICP has converged, sore is " << icp.getFitnessScore() << endl;
        // cout << "变换矩阵：\n" << icp.getFinalTransformation() << endl;
        T_icp.matrix()=icp.getFinalTransformation().cast<double>();
        T_final=T_icp*T_horizon;
        fstream myFile;
        myFile.open("/home/shy/1lidarcamera/data/"+filename+"/result/旋转文件/旋转"+to_string(n)+".txt", ios::out); 
        int P_L0=0,P_L1=0;
        cout<<"s0:  ";
        while(P_L0<s0.size())
        {
            cout<<s0[P_L0]<<" ,";
            P_L0++;
        }
        cout<<endl;
        cout<<"t1:  ";
        while(P_L1<t1.size())
        {
            cout<<t1[P_L1]<<" ,";
            P_L1++;
        }
        cout<<endl;
        // cout<<"角度 ： "<<roll_degrees<<" , "<<pitch_degrees<<" , "<<yaw_degrees<<endl;
        cout<<"第"<<n<<"个结果的旋转矩阵 :"<<endl<<T_final.matrix()<<endl<<endl;
        if (myFile.is_open())
            {
                myFile << T_final.matrix();
            myFile.close();
        }

        // m.saveTransformMatrix(n,filename);
        // m.getError(err);
        // m.getScore(not_vector_two, result);
        T_vector.push_back(T_final);
        pcl::transformPointCloud (*cloud_vector[0], *cloud0, T_final);//第一个雷达进行旋转
        pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/最终结果/"+to_string(n)+".pcd", *cloud0); 
            // cout<<"result = "<<result<<endl;
    }


    

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_sec = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();

    // 输出程序运行时间
    std::cout << "程序运行时间为：" << duration_sec << " 秒" << std::endl;
/////////////////////////////////////////////////////////////////////////////////////////////////////











#include<ctime>


    for(int j=0;j<obj_vector_two[0].size();j++)
    {
        double IN=255*(1024*rand()/(RAND_MAX+1.0f));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr OBJ(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud (*obj_vector_two[0][j], *cloud0, T_vector[0]);//第一个雷达进行旋转
        // *cloud0=*obj_vector_two[0][j];
        for(int n=0;n<cloud0->size();n++)
        {
            pcl::PointXYZI tmp;
            tmp.x=cloud0->points[n].x;
            tmp.y=cloud0->points[n].y;
            tmp.z=cloud0->points[n].z;
            tmp.intensity=IN;
            OBJ->points.push_back(tmp);
        }
        OBJ->width=OBJ->size();
        OBJ->height=1;
        pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/result/objall/"+to_string(0)+"/obj"+to_string(j)+".pcd", *OBJ); 
           
    }
    for(int j=0;j<obj_vector_two[1].size();j++)
    {
        double IN=255*(1024*rand()/(RAND_MAX+1.0f));
        pcl::PointCloud<pcl::PointXYZI>::Ptr OBJ(new pcl::PointCloud<pcl::PointXYZI>);

        for(int n=0;n<obj_vector_two[1][j]->size();n++)
        {
            pcl::PointXYZI tmp;
            tmp.x=obj_vector_two[1][j]->points[n].x;
            tmp.y=obj_vector_two[1][j]->points[n].y;
            tmp.z=obj_vector_two[1][j]->points[n].z;
            tmp.intensity=IN;
            OBJ->points.push_back(tmp);
        }
        OBJ->width=OBJ->size();
        OBJ->height=1;
        pcl::io::savePCDFileASCII("/home/shy/1lidarcamera/data/"+filename+"/result/objall/"+to_string(1)+"/obj"+to_string(j)+".pcd", *OBJ); 
           
    }


    return 0;
}
       