/**********************************
Author: David Chen
Date: 2019/03/12 
Last update: 2019/03/12
Point Cloud to Depth Image
Subscribe: 
  /camera/depth_registered/points      (sensor_msgs/PointCloud2)
  /X1/rgbd_camera/depth/points
  /X1/points
Publish:
  /obstacle_marker      (visualization_msgs/MarkerArray)
  /obstacle_marker_line (visualization_msgs/MarkerArray)
  /cluster_result       (sensor_msgs/PointCloud2)
***********************************/ 
#include <iostream>
#include <vector>
#include <array>
#include <time.h>
#include <string>
#include <math.h>
//Ros Lib
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "realsense2_camera/Extrinsics.h"
//PCL lib
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
//TF lib
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
//S2D Msgs
#include <s2d_msgs/S2D_ImageList.h>
#include <s2d_msgs/S2D_Image.h>

//OpenCV lib
#include <opencv/cv.h>

#define PI 3.14159
using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
bool is_LIDAR = true;

class LIDAR2Depth{
private:
	string node_name;

	// For point cloud declare variables
	string frame_id;
	tf::StampedTransform transformStamped;
	Eigen::Affine3d transform_eigen;

	// Image
	sensor_msgs::ImagePtr img_msg;
	cv_bridge::CvImage cvIMG;
	cv::Mat depth_image;

	// Counter
	int counts;

	// for ROS
	ros::NodeHandle nh;
	ros::Subscriber sub_cloud;
	ros::Publisher  pub_cloud;
	ros::Publisher  pub_image;
	ros::Publisher  pub_data;

	// Camera information
	// D435
	/*loat fx = 614.4776611328125;
	float fy = 614.2581176757812;
	float cx = 320.2654724121094;
	float cy = 249.4024658203125;*/

	// Information
	float img_x;
	float img_y;
	float img_z;
	int IMG_WIDTH = 512;
	int IMG_HEIGHT = 512;

	// Gazebo
	float fx;
	float fy;
	float cx;
	float cy;

	// LIDAR to CAMERA TF
	//tf::TransformListener listener(ros::Duration(10));
	bool get_tf = false;
	tf::TransformListener lr;
	tf::StampedTransform tf_lidar2cam;

public:
	LIDAR2Depth(ros::NodeHandle&);
    void get_msg();
	void cbCloud(const sensor_msgs::PointCloud2ConstPtr&);
	void get_img_coordinate(float, float, float);
	void pointcloud_to_image(const PointCloudXYZRGB::Ptr, PointCloudXYZRGB::Ptr);
	void pcl_preprocess(PointCloudXYZRGB::Ptr);
};

LIDAR2Depth::LIDAR2Depth(ros::NodeHandle &n){
	nh = n;
	counts = 0;
	node_name = ros::this_node::getName();
	depth_image = cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_16UC1, cv::Scalar(0, 0, 0));

	get_msg();

	// Publisher
	pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/l2d_pcl", 1);
	pub_image = nh.advertise<sensor_msgs::Image> ("/l2d_img", 1);
	pub_data  = nh.advertise<s2d_msgs::S2D_ImageList> ("/l2d_data", 1);

	// Subscriber
	if(is_LIDAR){
		sub_cloud = nh.subscribe("/velodyne_points_32", 1, &LIDAR2Depth::cbCloud, this);
	}
	else{
		sub_cloud = nh.subscribe("/X1/rgbd_camera/depth/points", 1, &LIDAR2Depth::cbCloud, this);
	}
}

void LIDAR2Depth::get_img_coordinate(float x, float y,float z){
	img_x = (x * fx)/z + cx;
	img_y = (y * fy)/z + cy;
	return;
}

void LIDAR2Depth::cbCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	if(!get_tf){
		try{
			lr.lookupTransform("/rgbd_link", "/velodyne_32",
									ros::Time(0), tf_lidar2cam);
			get_tf = true;
		}
		catch(tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		return;
	}

	frame_id = cloud_msg->header.frame_id;
	counts++;
	//return if no cloud data
	if ((cloud_msg->width * cloud_msg->height) == 0 || counts % 3 == 0)
	{
		counts = 0;
 		return ;
	}
	const clock_t t_start = clock();

	// transfer ros msg to point cloud
	PointCloudXYZ::Ptr cloud_XYZ(new PointCloudXYZ);
	PointCloudXYZRGB::Ptr cloud_XYZRGB(new PointCloudXYZRGB);
	PointCloudXYZRGB::Ptr cloud_lidar(new PointCloudXYZRGB);
	PointCloudXYZRGB::Ptr cloud_in(new PointCloudXYZRGB);
	PointCloudXYZRGB::Ptr cloud_out(new PointCloudXYZRGB);
	if(is_LIDAR){
		pcl::fromROSMsg (*cloud_msg, *cloud_XYZ);
		copyPointCloud(*cloud_XYZ, *cloud_lidar);
		//std::cout<<tf_lidar2cam.getOrigin().x()<<"," <<tf_lidar2cam.getOrigin().y()<<","<<tf_lidar2cam.getOrigin().z()<<std::endl;
		pcl_ros::transformPointCloud(*cloud_lidar, *cloud_in, tf_lidar2cam);
	}
	else{
		pcl::fromROSMsg (*cloud_msg, *cloud_XYZRGB);
		copyPointCloud(*cloud_XYZRGB, *cloud_in);
	}
	copyPointCloud(*cloud_in, *cloud_out);

	// Remove out of range points and robot points
	//pcl_preprocess(cloud_out);
	
	//cv::Mat depth_image(480, 640, CV_32FC1, cv::Scalar(0, 0, 0));
	pointcloud_to_image(cloud_in, cloud_out);
	clock_t t_end = clock();
	//cout << "PointCloud preprocess time taken = " << (t_end-t_start)/(double)(CLOCKS_PER_SEC) << endl;

	// Publish image
	cvIMG.header = cloud_msg->header;
	cvIMG.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	cvIMG.image = depth_image;
	//img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_image).toImageMsg();
	pub_image.publish(cvIMG.toImageMsg());

	s2d_msgs::S2D_ImageList s2d_data;
	s2d_msgs::S2D_Image s2d_img;
	s2d_data.header = cloud_msg->header;
	s2d_img.img = *cvIMG.toImageMsg();
	s2d_img.fx = fx;
	s2d_img.fy = fy;
	s2d_img.cx = cx;
	s2d_img.cy = cy;
	s2d_data.list.push_back(s2d_img);
	s2d_data.size = s2d_data.list.size();
	pub_data.publish(s2d_data);

	// Publish point cloud
	sensor_msgs::PointCloud2 pcl_output;
	pcl::toROSMsg(*cloud_out, pcl_output);
	pcl_output.header = cloud_msg->header;
	pcl_output.header.frame_id = "/rgbd_link";
	pub_cloud.publish(pcl_output);
}

void LIDAR2Depth::pointcloud_to_image(const PointCloudXYZRGB::Ptr cloud_in, PointCloudXYZRGB::Ptr cloud_out){
  //depth_image.setTo(cv::Scalar(0, 0, 0));
  	depth_image = cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_16UC1, cv::Scalar(0, 0, 0));
	for(int i = 0; i < cloud_in->points.size(); i++){
		float x;
		float y;

		if(is_LIDAR){
			x = float(cloud_in->points[i].y);
			y = float(cloud_in->points[i].z);
			img_z = float(cloud_in->points[i].x);
			if(img_z < 0){
				continue;
			}
			get_img_coordinate(x, y, img_z);
			if (int(img_x) < IMG_WIDTH && int(img_y) < IMG_HEIGHT && int(img_x) > 0 && int(img_y) > 0){
				cloud_out->points[i].r = 255;
				cloud_out->points[i].g = 255;
				cloud_out->points[i].b = 0;
				
				depth_image.at<signed short int>(IMG_HEIGHT-int(img_y), IMG_WIDTH-int(img_x)) = img_z*655;
			}
		}

		else{
			x = float(cloud_in->points[i].x);
			y = float(cloud_in->points[i].y);
			img_z = float(cloud_in->points[i].z);
			/*if (std::isnan(*x) || std::isnan(*y) ||std::isnan(z)){
			std::cout << *x << ',' << *y << "," << z << std::endl;
			}*/
			if(img_z < 0){
			continue;
			}
			get_img_coordinate(x, y, img_z);
			if (int(img_x) < IMG_WIDTH && int(img_y) < IMG_HEIGHT && int(img_x) >= 0 && int(img_y) >= 0){
				depth_image.at<float>(int(img_y), int(img_x)) = img_z;
			}
		}

		//free(x);
		//free(y);
		//free(z);
	}
	//return depth_image;
}

void LIDAR2Depth::pcl_preprocess(PointCloudXYZRGB::Ptr cloud_out){
	int num = 0;
	for (int i=0 ; i <  cloud_out->points.size() ; i++)
	{
		if (cloud_out->points[i].x > 0){
			cloud_out->points[i].r = 255;
			cloud_out->points[i].g = 255;
			cloud_out->points[i].b = 0;
		}
	}
}

void LIDAR2Depth::get_msg(){
  sensor_msgs::CameraInfo::ConstPtr msg;
  bool get_camera_info = false;
  while(!get_camera_info){
    if(is_LIDAR){
      msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/rgbd/rgb/camera_info",ros::Duration(10));
      get_camera_info = true;
    }
    else{
      msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info",ros::Duration(10));
      get_camera_info = true;
    }
  }
  fx = msg->P[0];
  fy = msg->P[5];
  cx = msg->P[2];
  cy = msg->P[6];
  cout << fx << "," << fy << "," << cx << "," << cy << std::endl;
return;
}

int main (int argc, char** argv)
{
	ros::init (argc, argv, "LIDAR2Depth");
	ROS_INFO("[%s] Start Runnig", ros::this_node::getName().c_str());
	ros::NodeHandle nh("~");
	LIDAR2Depth pn(nh);
	ros::spin ();
	return 0;
}