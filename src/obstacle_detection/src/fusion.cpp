//#include "fusion_detection.h"
#include "fusion.h"
#include "Obstacle_detection_core.h"

//读pcd到容器
void Obstacledetection::read_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, vector<point_xyz> &xyz_lidar)
{
    for (size_t i = 0; i < cloud_in->points.size (); ++i) //
    {
        point_xyz point_xyz1;
        point_xyz1.x = cloud_in->points[i].x;
        point_xyz1.y = cloud_in->points[i].y;
        point_xyz1.z = cloud_in->points[i].z;

        if(point_xyz1.x > 0)
        {
            xyz_lidar.push_back(point_xyz1);
        }
    }        

}



//根据内外参求解三维到二维的转换
void Obstacledetection::Solve_3d( cv::Mat Instrinsic, cv::Mat Extrinsic, vector <point_xyz> &xyz_lidar, vector <point_uv> &uv_camera)
{
    //每次调用前进行数据和内存清空
    vector <point_uv>().swap(uv_camera);

	for (int i = 0; i < xyz_lidar.size(); i++)
	{
		cv::Mat Wd = (cv::Mat_<double>(4, 1) << xyz_lidar[i].x, xyz_lidar[i].y, xyz_lidar[i].z, 1);

		cv::Mat Wc = (Extrinsic * Wd);

		double Zc;
		Zc = Wc.at<double>(2, 0);

		cv::Mat M2 = (Instrinsic * (Extrinsic * Wd)) / Zc;

		point_uv  uv1;

		uv1 .x = M2.at<double>(0, 0), uv1.y = M2.at<double>(1, 0);
	
		uv_camera.push_back(uv1);

	}
    


}

//将点云坐标和像素坐标关联
void Obstacledetection::correlation_uv_xyz(vector <point_xyz> &xyz_lidar,vector <point_uv> &uv_camera, multimap<point_uv,point_xyz> &CamtoLidar)
{
    //每次调用前进行数据和内存清空
    multimap<point_uv,point_xyz>().swap(CamtoLidar);

    //剔除相机视野外的点，并将像素和雷达坐标进行关联
    for (unsigned int i = 0; i < uv_camera.size(); ++i)
    {
        if(uv_camera[i].x > 0 && uv_camera[i].x < u_cam && uv_camera[i].y > 0&& uv_camera[i].y < v_cam)
        {
            CamtoLidar.insert(make_pair(uv_camera[i], xyz_lidar[i]));
        }

    }

}



//输入像素坐标找对应三维点云，实现
void Obstacledetection::getxyz(point_uv &uv_test,  multimap<point_uv,point_xyz> &CamtoLidar, point_xyz &target_point)
{
    map<point_uv, point_xyz >::iterator it;
	it = CamtoLidar.find(uv_test);

	if (it != CamtoLidar.end()) {
        target_point.x = (*it).second.x;
        target_point.y = (*it).second.y;
        target_point.z = (*it).second.z;
        //cout << "目标物中心雷达点云坐标：" << "x: "<< target_point.x << " y: " << target_point.y << " z: " << target_point.z << endl;
	}
	else {
        target_point.x = 0;
        target_point.y = 0;
        target_point.z = 0;
		//cout << "没有找到" << endl;
	}

}



//投影
void Obstacledetection::projection(vector <point_uv> &uv_camera, vector <point_xyz> &xyz_lidar, cv::Mat image, cv::Mat img_lidar)
{
    vector <float> distance_3d;
    for(int i = 0; i < xyz_lidar.size(); i++)
    {
        distance_3d.push_back(xyz_lidar[i].x);
    }
    float maxValue = *max_element(distance_3d.begin(), distance_3d.end());
	float minValue = *min_element(distance_3d.begin(), distance_3d.end());

    int color = 0;
    for (unsigned int i = 0; i < uv_camera.size(); ++i)
    {
        if(uv_camera[i].x > 0 && uv_camera[i].x < u_cam && uv_camera[i].y > 0&& uv_camera[i].y < v_cam)
        {
            cv::Point2i points1;
            points1.x = uv_camera[i].x;
            points1.y = uv_camera[i].y;
            color = (int)(255 * (xyz_lidar[i].x) / (maxValue - minValue));
            //cout << color << endl;
			if (0 < color < 255)
			{
				circle(img_lidar, points1, 2, cv::Scalar(0, color-255, color), -1);
                circle(image, points1, 2, cv::Scalar(0, color, 0), -1);
			}
			else if (255 < color < 510)
			{
				circle(img_lidar, points1, 2, cv::Scalar(0, color-255 , 0), -1);
                circle(image, points1, 2, cv::Scalar(color - 255, color - 255, 0), -1);
			}
			else {
				circle(img_lidar, points1, 2, cv::Scalar(255, 255, color - 510), -1);
                circle(image, points1, 2, cv::Scalar(color - 255, color - 255, 0), -1);
			}

        }

    }

}






