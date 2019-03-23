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
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
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

//OpenCV lib
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#define PI 3.14159
using namespace std;
using namespace message_filters;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
bool is_LIDAR = true;

class IMAGE_PROCESS{
private:
	string node_name;

	// For point cloud declare variables
	string frame_id;
	tf::StampedTransform transformStamped;
	Eigen::Affine3d transform_eigen;

	// Image
	sensor_msgs::ImagePtr img_msg;
	cv_bridge::CvImage cvIMG;
	cv::Mat out_img;
	cv::Mat pcl_img;

	// Counter
	int counter = 0;
	int img_count = 0;

	// for ROS
	ros::NodeHandle nh;
	ros::Publisher  pub_cloud;
	ros::Publisher  pub_image;

	message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub;
	message_filters::Subscriber<sensor_msgs::Image> depth_sub;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
	typedef message_filters::Synchronizer<MySyncPolicy> Sync;
	boost::shared_ptr<Sync> sync_;

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

	// Gazebo
	float fx = 554.254691191187;
	float fy = 554.254691191187;
	float cx = 320.5;
	float cy = 240.5;

	// LIDAR to CAMERA TF
	//tf::TransformListener listener(ros::Duration(10));
	bool get_tf = false;
	tf::TransformListener lr;
	tf::StampedTransform tf_lidar2cam;

public:
	IMAGE_PROCESS(ros::NodeHandle&);
	void cbCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
	void callback_sync(const sensor_msgs::PointCloud2ConstPtr&,const sensor_msgs::ImageConstPtr&);
	void get_img_coordinate(float, float, float);
	void pointcloud_to_image(const PointCloudXYZRGB::Ptr, PointCloudXYZRGB::Ptr);
	void pcl_preprocess(PointCloudXYZRGB::Ptr);
};

IMAGE_PROCESS::IMAGE_PROCESS(ros::NodeHandle &n){
	nh = n;
	pcl_sub.subscribe(nh, "/X1/points", 1);
	depth_sub.subscribe(nh, "/X1/rgbd_camera/depth/image_raw", 1);
	sync_.reset(new Sync(MySyncPolicy(10), pcl_sub, depth_sub));
	sync_->registerCallback(boost::bind(&IMAGE_PROCESS::callback_sync, this, _1, _2));
	node_name = ros::this_node::getName();
	out_img = cv::Mat(480, 640, CV_32FC1, cv::Scalar(0, 0, 0));
	pcl_img = cv::Mat(480, 640, CV_16UC1, cv::Scalar(0, 0, 0));

	// Publisher
	pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/ppp", 1);
	pub_image = nh.advertise<sensor_msgs::Image> ("/dp_img", 1);
}

void IMAGE_PROCESS::callback_sync(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::ImageConstPtr& depth_image){
	if(!get_tf){
		try{
			lr.lookupTransform("/X1/rgbd_camera_link", "/X1/front_laser",
									ros::Time(0), tf_lidar2cam);
			get_tf = true;
		}
		catch(tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		return;
	}
	counter++;
	if(counter%7 != 0){
		return;
	}
	PointCloudXYZ::Ptr cloud_XYZ(new PointCloudXYZ);	
	PointCloudXYZRGB::Ptr cloud_lidar(new PointCloudXYZRGB);
	PointCloudXYZRGB::Ptr cloud_in(new PointCloudXYZRGB);
	PointCloudXYZRGB::Ptr cloud_out(new PointCloudXYZRGB);

	pcl::fromROSMsg (*cloud_msg, *cloud_XYZ);
	copyPointCloud(*cloud_XYZ, *cloud_lidar);
	pcl_ros::transformPointCloud(*cloud_lidar, *cloud_in, tf_lidar2cam);
	copyPointCloud(*cloud_in, *cloud_out);
	pcl_img = cv::Mat(480, 640, CV_16UC1, cv::Scalar(0, 0, 0));

	for(int i = 0; i < cloud_in->points.size(); i++){
		float x;
		float y;
		x = float(cloud_in->points[i].y);
		y = float(cloud_in->points[i].z);
		img_z = float(cloud_in->points[i].x);
		if(img_z < 0){
			continue;
		}
		get_img_coordinate(x, y, img_z);
		if (int(img_x) < 640 && int(img_y) < 480 && int(img_x) >= 0 && int(img_y) >= 0){
			cloud_out->points[i].r = 255;
			cloud_out->points[i].g = 255;
			cloud_out->points[i].b = 0;
			// I change 32FC1 to 16UC1, so the image will become an integer matrix
			// As a result, I scale up the pixel value just like D435
			pcl_img.at<unsigned short int>(480-int(img_y), 640-int(img_x)) = img_z*1000;
		}
	}

	//cv_bridge::CvImagePtr img_ptr_img = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
	cv_bridge::CvImagePtr img_ptr_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
	out_img = cv::Mat(480, 640, CV_32FC1, cv::Scalar(0, 0, 0));
	for( int nrow = 0; nrow < img_ptr_depth->image.rows; nrow++){
		for(int ncol = 0; ncol < img_ptr_depth->image.cols; ncol++){ 
			if (std::isnan(img_ptr_depth->image.at<float>(nrow, ncol)) || img_ptr_depth->image.at<float>(nrow, ncol)<=0.0){
					out_img.at<float>(nrow,ncol) = (1.0);
					pcl_img.at<unsigned short int>(nrow,ncol) = 0;
			}
		}
	}
	cvIMG.header = depth_image->header;
	cvIMG.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	string pcl_fname, depth_fname;
	// we later need to do denormalize when we want to decode img_ptr_depth->image
	img_ptr_depth->image.convertTo(img_ptr_depth->image, CV_16UC1, 1000);
	//cv::imwrite("/media/arg_ws3/5E703E3A703E18EB/data/sparse2dense/depth/img_" + std::to_string(img_count) + ".png", img_ptr_depth->image);
	//cv::imwrite("/media/arg_ws3/5E703E3A703E18EB/data/sparse2dense/pcl/img_" + std::to_string(img_count) + ".png", pcl_img);
	//cv::imwrite("./depth1.png", img_ptr_depth->image);
	//cv::imwrite("./pcl1.png", pcl_img);

	cvIMG.image = pcl_img;
	pub_image.publish(cvIMG.toImageMsg());
	img_count++;
}

void IMAGE_PROCESS::get_img_coordinate(float x, float y,float z){
	img_x = (x * fx)/z + cx;
	img_y = (y * fy)/z + cy;
	return;
}

void IMAGE_PROCESS::pointcloud_to_image(const PointCloudXYZRGB::Ptr cloud_in, PointCloudXYZRGB::Ptr cloud_out){
  //depth_image.setTo(cv::Scalar(0, 0, 0));
  	out_img = cv::Mat(480, 640, CV_32FC1, cv::Scalar(0, 0, 0));
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
			if (int(img_x) < 640 && int(img_y) < 480 && int(img_x) >= 0 && int(img_y) >= 0){
				cloud_out->points[i].r = 255;
				cloud_out->points[i].g = 255;
				cloud_out->points[i].b = 0;
				out_img.at<float>(480-int(img_y), int(img_x)) = img_z;
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
			if (int(img_x) < 640 && int(img_y) < 480 && int(img_x) >= 0 && int(img_y) >= 0){
				out_img.at<float>(int(img_y), int(img_x)) = img_z;
			}
		}

		//free(x);
		//free(y);
		//free(z);
	}
	//return depth_image;
}

void IMAGE_PROCESS::pcl_preprocess(PointCloudXYZRGB::Ptr cloud_out){
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


int main (int argc, char** argv)
{
	ros::init (argc, argv, "image_process");
	ros::NodeHandle nh("~");
	IMAGE_PROCESS pn(nh);
	ros::spin ();
	return 0;
}