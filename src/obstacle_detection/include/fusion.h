#pragma onece
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "my_struct.h"

using namespace std;



#include <pcl/io/pcd_io.h>


//相机分辨率

#define u_cam 3072
#define v_cam 2048



//将pcd数据读入容器
void read_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, vector<point_xyz> &xyz_lidar);

//根据内外参求解三维到二维的转换 函数声明
void Solve_3d(cv:: Mat Instrinsic, cv::Mat Extrinsic, vector <point_xyz> &xyz_lidar,vector <point_uv> &uv_camera);

//关联
void correlation_uv_xyz(vector <point_xyz> &xyz_lidar,vector <point_uv> &uv_camera,multimap<point_uv,point_xyz> &CamtoLidar);


//输入像素坐标找对应三维点云 函数声明
void getxyz(point_uv &uv_test,multimap<point_uv,point_xyz> &CamtoLidar,point_xyz &target_point);

//投影
void projection(vector <point_uv> &uv_camera, vector <point_xyz> &xyz_lidar, cv::Mat image, cv::Mat img_lidar);