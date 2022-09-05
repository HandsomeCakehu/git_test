#include "Obstacle_detection_core.h"
#include "yolo.h"
#include "time.h"

void Sleep(int ms)
{
    struct timeval delay;
    delay.tv_sec = 0;
    delay.tv_usec = ms * 1000; // 20 ms
    select(0, NULL, NULL, NULL, &delay);
}


string model1_path = "/home/hbs/ws_work/yolov5testliu/input/yolov5s.onnx";
int times=0;
//内参矩阵
cv::Mat Intrinsic = (cv::Mat_<double>(3, 4) << 2597.805135831851, 0, 1499.454613271735, 0,
                                        0, 2597.615996305592, 1058.121794514663, 0,
                                        0, 0, 1, 0);

cv::Mat Intrinsic1 = (cv::Mat_<double>(3, 4) << 862.0763454000013, 0, 616.4891710202677,0,
                                        0, 859.3478016445682, 551.4675575416126,0,
                                        0, 0, 1, 0);

//外参矩阵（手动）
cv::Mat Extrinsic = (cv::Mat_<double>(4, 4) << 0., -1, 0, 0,
                                        0, 0, -1, 0.01,
                                        1, 0, 0, 0.05,
                                        0, 0, 0, 1);

cv::Mat Extrinsic1 = (cv::Mat_<double>(4, 4) << 0., -1, 0, 0.02,
                                        0, 0, -1, 0.12,
                                        1, 0, 0, 0.06,
                                        0, 0, 0, 1);


cv::Mat Intrinsic3 = (cv::Mat_<double>(3, 4) << 2599.151460808269, 0, 1467.794977089861,0,
											0, 2597.342315311746, 1110.976121974972, 0,
											0, 0, 1, 0);
                                            
cv::Mat Extrinsic3 = (cv::Mat_<double>(4, 4) << 0., -1, 0, 0.05,
										    0, 0, -1, 0.08,
										    1, 0, 0, 0.05,
											0, 0, 0, 1);
//pcl::visualization::CloudViewer viewer("Simple");

Obstacledetection::Obstacledetection(ros::NodeHandle &nh)
{

    seg_distance_ = {15, 30, 45, 60};
    cluster_distance_ = {0.5, 1.0, 1.5, 2.0, 2.5};

    // intial_socket(socketfd);
    //recvMessage_socket(socketfd,order,ok);

    sub_point_cloud_ = nh.subscribe("/filtered_points_no_ground", 5, &Obstacledetection::point_cb, this);
    //sub_lidar = nh.subscribe("/livox/lidar", 10, &Obstacledetection::callback_lidar, this);
    
    sub_lidar = nh.subscribe("/rslidar_points", 10, &Obstacledetection::callback_lidar, this);

    sub_camera = nh.subscribe("/hikrobot_camera/rgb", 10, &Obstacledetection::callback_camera, this);


    pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_boxs", 5);
    pub_mytopic_ob = nh.advertise<mytopic_ob::mytopic_ob>("/msg_ob",5);

    sub_mytopic_ob = nh.subscribe("/msg_ob",10, &Obstacledetection::callback_ob, this);

    ros::spin();
}

Obstacledetection::~Obstacledetection() {}

void Obstacledetection::publish_cloud(const ros::Publisher &in_publisher,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                                  const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}

void Obstacledetection::voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(in);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*out);
}

void Obstacledetection::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                                    double in_max_cluster_distance, std::vector<Detected_Obj> &obj_list,vector <pointcloud> &allbox)
{

    

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    //create 2d pc
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc, *cloud_2d);
    //make it flat
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0;
    }

    if (cloud_2d->points.size() > 0)
        tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> local_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setInputCloud(cloud_2d);
    euclid.setClusterTolerance(in_max_cluster_distance);
    euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);

    for (size_t i = 0; i < local_indices.size(); i++)
    {
        // the structure to save one detected object
        Detected_Obj obj_info;
        pointcloud obj;

        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();

        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {
            //fill new colored cluster point by point
            pcl::PointXYZ p;
            p.x = in_pc->points[*pit].x;
            p.y = in_pc->points[*pit].y;
            p.z = in_pc->points[*pit].z;

            obj_info.centroid_.x += p.x;
            obj_info.centroid_.y += p.y;
            obj_info.centroid_.z += p.z;

            obj.x += p.x;
            obj.y += p.y;
            obj.z += p.z;

            if (p.x < min_x)
                min_x = p.x;
            if (p.y < min_y)
                min_y = p.y;
            if (p.z < min_z)
                min_z = p.z;
            if (p.x > max_x)
                max_x = p.x;
            if (p.y > max_y)
                max_y = p.y;
            if (p.z > max_z)
                max_z = p.z;
        }

        //min, max points
        obj_info.min_point_.x = min_x;
        obj_info.min_point_.y = min_y;
        obj_info.min_point_.z = min_z;

        obj_info.max_point_.x = max_x;
        obj_info.max_point_.y = max_y;
        obj_info.max_point_.z = max_z;

        //calculate centroid, average
        if (local_indices[i].indices.size() > 0)
        {
            obj_info.centroid_.x /= local_indices[i].indices.size();
            obj_info.centroid_.y /= local_indices[i].indices.size();
            obj_info.centroid_.z /= local_indices[i].indices.size();



        }
        obj.x = (obj_info.min_point_.x + obj_info.max_point_.x)/2;
        obj.y = (obj_info.min_point_.y + obj_info.max_point_.y)/2;
        obj.z = (obj_info.min_point_.z + obj_info.max_point_.z)/2;
        //calculate bounding box
        double length_ = obj_info.max_point_.x - obj_info.min_point_.x;
        double width_ = obj_info.max_point_.y - obj_info.min_point_.y;
        double height_ = obj_info.max_point_.z - obj_info.min_point_.z;

        obj.length = width_;
        obj.depth = length_;
        obj.heighth = height_;
        obj.distance = 0;

        obj_info.bounding_box_.header = point_cloud_header_;

        obj_info.bounding_box_.pose.position.x = obj_info.min_point_.x + length_ / 2;
        obj_info.bounding_box_.pose.position.y = obj_info.min_point_.y + width_ / 2;
        obj_info.bounding_box_.pose.position.z = obj_info.min_point_.z + height_ / 2;

        obj_info.bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
        obj_info.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
        obj_info.bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

        obj_list.push_back(obj_info);
        allbox.push_back(obj);
    }



}

void Obstacledetection::cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list,vector <pointcloud> &allbox)
{
    //cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
    //in this way, the points farther in the pc will also be clustered

    //0 => 0-15m d=0.5
    //1 => 15-30 d=1
    //2 => 30-45 d=1.6
    //3 => 45-60 d=2.1
    //4 => >60   d=2.6
    vector<pointcloud>().swap(allbox);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pc_array(5);

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        segment_pc_array[i] = tmp;
    }

    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_pc->points[i].x;
        current_point.y = in_pc->points[i].y;
        current_point.z = in_pc->points[i].z;

        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

        // 如果点的距离大于120m, 忽略该点
        if (origin_distance >= 120)
        {
            continue;
        }

        if (origin_distance < seg_distance_[0])
        {
            segment_pc_array[0]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[1])
        {
            segment_pc_array[1]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[2])
        {
            segment_pc_array[2]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[3])
        {
            segment_pc_array[3]->points.push_back(current_point);
        }
        else
        {
            segment_pc_array[4]->points.push_back(current_point);
        }
    }

    std::vector<pcl::PointIndices> final_indices;
    std::vector<pcl::PointIndices> tmp_indices;

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        cluster_segment(segment_pc_array[i], cluster_distance_[i], obj_list,allbox);
    }


}

void Obstacledetection::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    point_cloud_header_ = in_cloud_ptr->header;

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    // down sampling the point cloud before cluster
    voxel_grid_filer(current_pc_ptr, filtered_pc_ptr, LEAF_SIZE);

    std::vector<Detected_Obj> global_obj_list;
    cluster_by_distance(filtered_pc_ptr, global_obj_list,allbox);

    

    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    for (size_t i = 0; i < global_obj_list.size(); i++)
    {
        bbox_array.boxes.push_back(global_obj_list[i].bounding_box_);
    }
    bbox_array.header = point_cloud_header_;

    pub_bounding_boxs_.publish(bbox_array);

}




void Obstacledetection::callback_lidar( const sensor_msgs::PointCloud2::ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cliped_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    vector<point_xyz> xyz_lidar; 

    //viewer.showCloud(current_pc_ptr);
    read_cloud(current_pc_ptr,xyz_lidar);
    vector<point_uv> uv_camera;



    //根据内外参求解三维到二维的转换 函数声明
    Solve_3d(Intrinsic3, Extrinsic3, xyz_lidar,uv_camera);


    //multimap<point_uv,point_xyz> CamtoLidar;
    //关联
    correlation_uv_xyz(xyz_lidar,uv_camera,CamtoLidar);
    

    cv::Mat img(v_cam,u_cam,CV_8UC3, cv::Scalar(255, 255, 255));


    // cv::namedWindow("image",cv::WINDOW_NORMAL);
    // cv::imshow("image",img);
    // cv::waitKey(3);

    
    if(!raw_img.empty())
    {
        projection(uv_camera,xyz_lidar,raw_img,img);
        cv::namedWindow("image",cv::WINDOW_NORMAL);
        cv::imshow("image",raw_img);
        cv::waitKey(3);
    }



    

}


bool com(const point_xyz& a, const point_xyz& b) {
	return a.x < b.x;
}


void Obstacledetection::callback_camera(const sensor_msgs::Image::ConstPtr &img)
{
    cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
            }
            catch (cv_bridge::Exception &e) {
                return;
            }
    raw_img = cv_ptr->image;
    // cv::namedWindow("image",0);
    // cv::imshow("image",raw_img);
    // cv::waitKey(3);

    //cout << img->header.stamp;
    yolov5(raw_img,model1_path,ob);



    
    point_uv uv_test;
    point_xyz target_point;

    // if(!ob.empty())
    // {
    
    //     vector<point_xyz>().swap(alltarget);

    //     for (int i = 0; i < ob.size(); i++)
    //     {
    //         // cout << CamtoLidar.size() << endl;
    //         uv_test.x  =  (ob[i].u_left + ob[i].u_right )/ 2;
    //         uv_test.y  =  (ob[i].v_left + ob[i].v_right )/ 2;
    //         //cout << uv_test.x << "  " << uv_test.y << endl;
    //         //cout << ob[i].name << endl;

    //         getxyz(uv_test,CamtoLidar,target_point);

    //         if(target_point.x == 0)
    //         {
    //             for(int i = -2; i < 2; i++)
    //             {
    //                 for(int j = -5; j < 10; j++)
    //                 {
    //                     uv_test.x += i;
    //                     uv_test.y += j;
    //                     getxyz(uv_test,CamtoLidar,target_point);
    //                     if(target_point.x != 0)
    //                     {
    //                         break;
    //                     }
                        
    //                 }
    //                 if( target_point.x != 0)
    //                 {
    //                     break;
    //                 }
                    
    //             }

    //         }
    //         alltarget.push_back(target_point);
    //         ob[i].mass_x = target_point.x;
    //         ob[i].mass_y = target_point.y;
    //         ob[i].mass_z = target_point.z;

    //     }

    // }

if(!ob.empty())
    {
    
        vector<point_xyz>().swap(alltarget);
        vector<point_xyz>().swap(alltarget_filter);

        for (int i = 0; i < ob.size(); i++)
        {


            for(int k = ob[i].u_left;k < ob[i].u_right;k++)
            {
                for(int j = ob[i].v_left; j < ob[i].v_right;j++)
                {
                    uv_test.x = k;
                    uv_test.y = j;
                    getxyz(uv_test,CamtoLidar,target_point);
                    if(target_point.x != 0)
                    {
                        alltarget.push_back(target_point);

                    }

                }

            }
   
            
            sort(alltarget.begin(), alltarget.end(), com);
            

            for(int i = 0; i < alltarget.size(); i++)
            {
                int a = alltarget.size();
               
                if((float)i/a > 0.4 && (float)i/a < 0.6)
                {
                    alltarget_filter.push_back(alltarget[i]);
                }

            }


            float x_sum = 0;
            float y_sum = 0;
            float z_sum = 0;

            for(int i = 0; i < alltarget_filter.size(); i++)
            {
                x_sum = x_sum + alltarget_filter[i].x;
                y_sum = y_sum + alltarget_filter[i].y;
                z_sum = z_sum + alltarget_filter[i].z;
            }
            float b = alltarget_filter.size();

            ob[i].mass_x = x_sum/b;
            ob[i].mass_y = y_sum/b;
            ob[i].mass_z = z_sum/b;
            // ob[i].mass_x = target_point.x;
            // ob[i].mass_y = target_point.y;
            // ob[i].mass_z = target_point.z;

        }


    }

    pointcloud target_box;
    // cout << "allbox  size ="<<allbox.size() << endl;
    // for (int i = 0; i < allbox.size(); i++)alltarget
    // {
    //     cout << allbox[i].x << " " << allbox[i].y << " " << allbox[i].z << endl;;

    // }

    for(int i = 0; i < ob.size(); i++)
    {
    
        //cout << allbox[i].x  << allbox[i].y << allbox[i].z << endl;
        if (ob[i].mass_x != 0)
        {
            target_point.x = ob[i].mass_x;
            target_point.y = ob[i].mass_y;
            target_point.z = ob[i].mass_z;
            judge_box(allbox,target_point,target_box);
            
            if(target_box.x != 0)
            {
                ob[i].length = target_box.length;
                ob[i].depth = target_box.depth;
                ob[i].height = target_box.heighth;
                ob[i].mass_x = target_box.x;
                ob[i].mass_y = target_box.y;
                ob[i].mass_z = target_box.z;

            }


        }

        cout << "障碍物: " << ob[i].name <<endl;
        cout << "尺寸：" << endl;
        cout << "质心坐标：" << ob[i].mass_x << " " << ob[i].mass_y << " " << ob[i].mass_z << endl;
        cout << "长高深： " << ob[i].length << "  " << ob[i].height << "  " << ob[i].depth << endl;

        // cout << "obsize" << ob.size() << endl;


    }

    if(!ob.empty())
    {
        publish_ob(pub_mytopic_ob,ob);
    }
    


}



void Obstacledetection::publish_ob(ros::Publisher &in_publisher,vector <obstacle> ob)
{
    mytopic_ob::mytopic_ob msg_ob;
    for (int i = 0; i < ob.size(); i++)
    {
        msg_ob.u_left = ob[i].u_left;
        msg_ob.v_left = ob[i].v_left;
        msg_ob.u_right = ob[i].u_right;
        msg_ob.v_right = ob[i].v_right;
        msg_ob.mass_x = ob[i].mass_x;
        msg_ob.mass_y = ob[i].mass_y;
        msg_ob.mass_z = ob[i].mass_z;
        msg_ob.depth = ob[i].depth;
        msg_ob.height = ob[i].height;
        msg_ob.length = ob[i].length;

        in_publisher.publish(msg_ob);
    }


}

void Obstacledetection::callback_ob(const mytopic_ob::mytopic_ob &msg_ob)
{
    obstacle send_ob;
    send_ob.u_left  =  msg_ob.u_left;
    send_ob.v_left  =  msg_ob.v_left;
    send_ob.u_right  =  msg_ob.u_right;
    send_ob.v_right  =  msg_ob.v_right;
    send_ob.mass_x =  msg_ob.mass_x;
    send_ob.mass_y =  msg_ob.mass_y;
    send_ob.mass_z =  msg_ob.mass_z;
    send_ob.depth  =  msg_ob.depth;
    send_ob.height  =  msg_ob.height;
    send_ob.length  =  msg_ob.length;

    if(ok ==1)
    {
        Sleep(100);
        sendMessage_socket(socketfd,send_ob);

    }


    // cout << "尺寸：" << endl;
    // cout << "质心坐标：" << send_ob.mass_x << " " << send_ob.mass_y << " " << send_ob.mass_z << endl;
    // cout << "长高深： " << send_ob.length << "  " << send_ob.height << "  " << send_ob.depth << endl;

}



bool comp(const pointcloud& a, const pointcloud& b) {
	return a.distance < b.distance;
}

//判断在哪个框内
void Obstacledetection::judge_box (vector<pointcloud> &Boxes ,point_xyz &target_point, pointcloud &target_box)
{
    for (int i = 0; i < Boxes.size(); i++)
    {
        Boxes[i].distance = (Boxes[i].x - target_point.x) * (Boxes[i].x - target_point.x) + (Boxes[i].y - target_point.y) * (Boxes[i].y - target_point.y) +
		(Boxes[i].z - target_point.z) * (Boxes[i].z - target_point.z);
    }
    sort(Boxes.begin(), Boxes.end(), comp);

    int idbest = 0;
	for (int i = 0; i < Boxes.size(); i++)
	{
		if (target_point.x > Boxes[i].x - Boxes[i].depth / 2 && target_point.x < Boxes[i].x + Boxes[i].depth / 2 &&
			target_point.y > Boxes[i].y - Boxes[i].length / 2 && target_point.y < Boxes[i].y + Boxes[i].length / 2 &&
			target_point.z > Boxes[i].z - Boxes[i].heighth / 2 && target_point.z < Boxes[i].z + Boxes[i].heighth / 2)
		{

			// cout << "包围盒的质心坐标为; " << " X = " << Boxes[i].x << " Y = " << Boxes[i].y << " Z = " << Boxes[i].z << endl;
			// cout << "包围盒的长: " << " a = " << Boxes[i].length << endl;
			// cout << "包围盒的高: " << " b = " << Boxes[i].heighth << endl;
			// cout << "包围盒的深: " << " c = " << Boxes[i].depth << endl;
            target_box.x = Boxes[i].x;
            target_box.y = Boxes[i].y;
            target_box.z = Boxes[i].z;
            target_box.depth = Boxes[i].depth;
            target_box.heighth = Boxes[i].heighth;
            target_box.length = Boxes[i].length;
            //cout << Boxes[i].distance << endl;
			//cout << "i=" << i << endl;
			//idbest = i;
			break;
		}

        else 
        {
            target_box.x = 0;
            target_box.y = 0;
            target_box.z = 0;
            target_box.depth = 0;
            target_box.heighth = 0;
            target_box.length = 0;

        }
	}

  
	// int numeverage=0;
	// //cout << "idbest=" << idbest << endl;
	// for (int i = idbest; i < Boxes.size()-idbest; i++)
	// {
	// 	//cout << "cross1=" << pointBoxes[i].y << endl;
	// 	if (target_point.x > Boxes[i].x - Boxes[i].depth / 2 && target_point.x < Boxes[i].x + Boxes[i].depth / 2 &&
	// 		target_point.y > Boxes[i].y - Boxes[i].length / 2 && target_point.y < Boxes[i].y + Boxes[i].length / 2 &&
	// 		target_point.z > Boxes[i].z - Boxes[i].heighth / 2 && target_point.z < Boxes[i].z + Boxes[i].heighth / 2)
	// 	{
	// 		//cout << "cross=" << Boxes[i].y <<endl;
	// 		/*所有框的位置尺寸相加求平均*/
	// 		target_box.x += Boxes[i].x;
	// 		target_box.y += Boxes[i].y;
	// 		target_box.z += Boxes[i].z;
	// 		target_box.length += Boxes[i].length;
	// 		target_box.heighth += Boxes[i].heighth;
	// 		target_box.depth += Boxes[i].depth;
	// 		numeverage++;
	// 	}
	// 	else
	// 	{
    //             //cout << numeverage << endl;
	// 			target_box.x = target_box.x / numeverage;
	// 			target_box.y = target_box.y / numeverage;
	// 			target_box.z = target_box.z / numeverage;
	// 			target_box.length = target_box.length / numeverage;
	// 			target_box.heighth = target_box.heighth / numeverage;
	// 			target_box.depth = target_box.depth / numeverage;
	// 		break;
	// 	}
	// }

	// 	cout << target_box.x<<" "<< target_box.y << " "<< target_box.z << " "<< target_box.length << " "<< target_box.heighth << " "<<
	// 		target_box.depth << " "<< target_box.distance<< endl;


}



