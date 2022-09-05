#ifndef _OBSTACLE_DETECTION_CORE_H_
#define _OBSTACLE_DETECTION_CORE_H_

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <std_msgs/Header.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <sensor_msgs/PointCloud2.h>

#include "fusion.h"
#include "my_struct.h"
#include "pcl/visualization/cloud_viewer.h"



#include "ros/package.h"
#include <iostream>
#include <numeric>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <termios.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <Eigen/Dense>
#include <ros/package.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/flann.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>

#include "mytopic_ob/mytopic_ob.h"



#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>


#define CLIP_HEIGHT 3 //截取掉高于雷达自身3米的点
#define CLIP_BELOW_HEIGHT 0.05 //截取掉低于雷达自身0.05米的点
#define MIN_DISTANCE 0.2
#define RADIAL_DIVIDER_ANGLE 0.18
#define SENSOR_HEIGHT 0.4

#define concentric_divider_distance_ 0.01 //0.1 meters default
#define min_height_threshold_ 0.05
#define local_max_slope_ 8   //max slope of the ground between points, degree
#define general_max_slope_ 5 //max slope of the ground in entire point cloud, degree
#define reclass_distance_threshold_ 0.2



#define LEAF_SIZE 0.1 //定义降采样的leaf size，聚类是一个费时运算，为了减少计算量，我们通常先进行降采样
#define MIN_CLUSTER_SIZE 20
#define MAX_CLUSTER_SIZE 5000



extern int socketfd;

extern int ok;
extern struct order order;
void intial_socket(int &socket_fd);
void sendMessage_socket(int socket_fd, struct obstacle aa);
void recvMessage_socket(struct order order, int &ok);


class Obstacledetection
{

private:
  struct Detected_Obj
  {
    jsk_recognition_msgs::BoundingBox bounding_box_;
    
    

    pcl::PointXYZ min_point_;
    pcl::PointXYZ max_point_;
    pcl::PointXYZ centroid_;
  };
//订阅
  ros::Subscriber sub_point_cloud_;
  ros::Subscriber sub_lidar;
  ros::Subscriber sub_camera;
  ros::Subscriber sub_mytopic_ob;
//定义变量
  cv::Mat raw_img;
  vector <obstacle> ob;
  vector <pointcloud> allbox;
  vector <point_xyz> alltarget;
  vector<point_xyz> alltarget_filter;
  multimap<point_uv,point_xyz> CamtoLidar;
  

  ros::Publisher pub_bounding_boxs_;
  ros::Publisher pub_mytopic_ob;

  std::vector<double> seg_distance_, cluster_distance_;

  std_msgs::Header point_cloud_header_;

  void voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size);

  void cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list,vector <pointcloud> &allbox);

  void cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                       double in_max_cluster_distance, std::vector<Detected_Obj> & obj_list,vector <pointcloud> &allbox);

  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);
  void callback_lidar(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);
  void callback_camera(const sensor_msgs::Image::ConstPtr &img);

  void read_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, vector<point_xyz> &xyz_lidar);

  //根据内外参求解三维到二维的转换 函数声明
  void Solve_3d(cv:: Mat Instrinsic, cv::Mat Extrinsic, vector <point_xyz> &xyz_lidar,vector <point_uv> &uv_camera);

  //关联
  void correlation_uv_xyz(vector <point_xyz> &xyz_lidar,vector <point_uv> &uv_camera,multimap<point_uv,point_xyz> &CamtoLidar);


  //输入像素坐标找对应三维点云 函数声明
  void getxyz(point_uv &uv_test,multimap<point_uv,point_xyz> &CamtoLidar,point_xyz &target_point);

  //投影
  void projection(vector <point_uv> &uv_camera, vector <point_xyz> &xyz_lidar, cv::Mat image, cv::Mat img_lidar);
  

  void publish_cloud(const ros::Publisher &in_publisher,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                     const std_msgs::Header &in_header);
                     
  void judge_box (vector<pointcloud> &Boxes ,point_xyz &target_point, pointcloud &target_box);
  
  
  


  void publish_ob(ros::Publisher &in_publisher, vector <obstacle> ob);

  void callback_ob(const mytopic_ob::mytopic_ob &msg_ob);

public:
  Obstacledetection(ros::NodeHandle &nh);
  ~Obstacledetection();
};




#endif