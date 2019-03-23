#include "depth_to_point.h"

// void depth_to_point::generate_pointcloud(PointXYZRGB::Ptr point){
// 	/*Decode text pointcloud and text direction
// 	*/
// 	std_msgs::Float32MultiArray pointarray = text_segment_msg.pc_array;
// 	geometry_msgs::Vector3 text_pose_x = text_segment_msg.text_direc;
// 	point->points.resize(pointarray.layout.dim[0].size/7) ;
// 	point->height = 1;
// 	point->width = pointarray.layout.dim[0].size/7;
// 	point->header.frame_id = "camera1_color_optical_frame";
// 	int count = 0;
// 	for (int i=0;i<pointarray.layout.dim[0].size/7;i++){
// 		point->points[i].x=pointarray.data[count++];
// 		point->points[i].y=pointarray.data[count++];
// 		point->points[i].z=pointarray.data[count++];
// 		point->points[i].r=pointarray.data[count++];
// 		point->points[i].g=pointarray.data[count++];
// 		point->points[i].b=pointarray.data[count++];
// 		count++;
// 	}

// 	std::vector<int> indices;
// 	sensor_msgs::PointCloud2 object_cloud_msg;
// 	pcl::toROSMsg(*point, object_cloud_msg);
// 	obstacle_cloud_publisher.publish(object_cloud_msg);
// 	pcl::removeNaNFromPointCloud(*point, *point, indices);
// 	return ;
// }
bool is_gazebo = true;

void depth_to_point::getXYZ(float* a, float* b,float zc){

	float inv_fx = 1.0/fx;
	float inv_fy = 1.0/fy;
	*a = (*a - cx) * zc * inv_fx;
	*b = (*b - cy) * zc * inv_fy;
	return;
}
/*void depth_to_point::callback(const sensor_msgs::ImageConstPtr& msg_depth){
	sensor_msgs::ImageConstPtr image_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_rect_color",ros::Duration(10));
	pc.reset(new PointCloud<PointXYZRGB>());
	cv_bridge::CvImagePtr img_ptr_depth;
	img_ptr_depth = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
	
	cv_bridge::CvImagePtr img_ptr_img = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
	for( int nrow = 0; nrow < img_ptr_depth->image.rows; nrow++){  
       for(int ncol = 0; ncol < img_ptr_depth->image.cols; ncol++){  
       	if (img_ptr_depth->image.at<unsigned short int>(nrow,ncol) > 1){
       		
       		pcl::PointXYZRGB point;
       		float* x = new float(nrow);
       		float* y = new float(ncol);
       	 	float z = float(img_ptr_depth->image.at<unsigned short int>(nrow,ncol))/1000.;

       		getXYZ(y,x,z);
       		point.x = *x;
       		point.y = *y;
       		point.z = z;
       		Vec3b intensity =  img_ptr_img->image.at<Vec3b>(nrow, ncol); 
       		point.r = int(intensity[0]);
       		point.g = int(intensity[1]);
       		point.b = int(intensity[2]);
       		pc->points.push_back(point);
       		free(x);
       		free(y);
       	} 
       }  
    } 
    //cout << pc->points.size() << endl;
    
    sensor_msgs::PointCloud2 object_cloud_msg;
    toROSMsg(*pc, object_cloud_msg);
    object_cloud_msg.header.frame_id = "camera_link";
    pc2.publish(object_cloud_msg);
   
	return;
}*/

void depth_to_point::callback_sync(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::ImageConstPtr& depth_image){
	pc.reset(new PointCloud<PointXYZRGB>());
  cv_bridge::CvImagePtr img_ptr_depth;
  if(is_gazebo){
    img_ptr_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
  }
	else{
    img_ptr_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
  }
	cv_bridge::CvImagePtr img_ptr_img = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
	for( int nrow = 0; nrow < img_ptr_depth->image.rows; nrow++){  
       for(int ncol = 0; ncol < img_ptr_depth->image.cols; ncol++){  
       	if (img_ptr_depth->image.at<unsigned short int>(nrow,ncol) > 1){
       		
       		pcl::PointXYZRGB point;
       		float* x = new float(nrow);
       		float* y = new float(ncol);
          float z;
          if(is_gazebo){
            z = float(img_ptr_depth->image.at<float>(nrow,ncol));
          }
       	 	else{
            z = float(img_ptr_depth->image.at<unsigned short int>(nrow,ncol))/1000.;
          }

       		getXYZ(y,x,z);
       		point.x = z;
       		point.y = -*y;
       		point.z = -*x;
       		Vec3b intensity =  img_ptr_img->image.at<Vec3b>(nrow, ncol); 
       		point.r = int(intensity[0]);
       		point.g = int(intensity[1]);
       		point.b = int(intensity[2]);
       		pc->points.push_back(point);
       		free(x);
       		free(y);
       		// delete x;
       		// delete y;
       	} 
       }  
    } 

    //cout << pc->points.size() << endl;
    
    sensor_msgs::PointCloud2 object_cloud_msg;
    toROSMsg(*pc, object_cloud_msg);
    if(is_gazebo){
      object_cloud_msg.header.frame_id = "X1/rgbd_camera_link";
    }
    else{
      object_cloud_msg.header.frame_id = "camera_color_optical_frame";
    }
    pc2.publish(object_cloud_msg);

	// pc->width    = pc->points.size();
	// pc->height   = 1;
	// pc->is_dense = false;
   	//pcl::io::savePCDFileASCII ("/home/andyser/code/test_pcd.pcd", *pc);
	return;
}
depth_to_point::depth_to_point(){
	NodeHandle nh;
	pc2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc", 10);
	//depth_image = nh.subscribe<sensor_msgs::Image>("/depth_image", 1, &depth_to_point::callback,this);
  if(is_gazebo){
    img_sub.subscribe(nh, "/X1/rgbd_camera/rgb/image_raw", 1);
    depth_sub.subscribe(nh, "/X1/rgbd_camera/depth/image_raw", 1);
  }
  else{
    img_sub.subscribe(nh, "/camera/color/image_rect_color", 1);
    depth_sub.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", 1);
  }
	sync_.reset(new Sync(MySyncPolicy(10), img_sub, depth_sub));
	sync_->registerCallback(boost::bind(&depth_to_point::callback_sync, this, _1, _2));

}
void depth_to_point::get_msg(){
sensor_msgs::CameraInfo::ConstPtr msg(new sensor_msgs::CameraInfo);
  if(is_gazebo){
    msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/X1/rgbd_camera/rgb/camera_info",ros::Duration(10));
  }
  else{
    msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info",ros::Duration(10));
  }
  fx = msg->P[0];
  fy = msg->P[5];
  cx = msg->P[2];
  cy = msg->P[6];
  fx = 554.255;
  fy = 554.255;
  cx = 320.5;
  cy = 240.5;
  cout << fx << "," << fy << "," << cx << "," << cy << std::endl;
  int count = 0;
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 4; j++)
      Projection[i][j] = msg->P[count++];

  if(!is_gazebo){
    realsense2_camera::ExtrinsicsConstPtr msg1 = ros::topic::waitForMessage<realsense2_camera::Extrinsics>("/camera/extrinsics/depth_to_color",ros::Duration(10));
    count = 0;
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        extrinsics[i][j] = msg1->rotation[count++];
    for(int i = 0; i < 3 ; i++)
      extrinsics[i][3] = msg1->translation[i];
  }
  return;
}
int main(int argc, char** argv){
    init(argc, argv, "depth_to_point");
    depth_to_point depth_to_point;
    depth_to_point.get_msg();
    spin();
    return 0;
}