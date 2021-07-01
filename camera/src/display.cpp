#include <iostream>
#include <string>
#include <sstream>
  
#include <ros/ros.h>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>

#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace message_filters;

geometry_msgs::Pose pose;
int number_points;
float larghezza, altezza;

 pcl::PointCloud<pcl::PointXYZ> box;

void callback(const sensor_msgs::CameraInfoConstPtr& , const sensor_msgs::ImageConstPtr&);
void callback_pose(const geometry_msgs::PoseArray);
void callback_n_points(const std_msgs::UInt32);
void callback_altezza(const std_msgs::Float32);
void callback_larghezza(const std_msgs::Float32);
void callback_box(const sensor_msgs::PointCloud2ConstPtr&);


int main (int argc, char** argv) {

  ros::init (argc, argv, "display");
  ros::NodeHandle nh;

  ros::Subscriber sub_box = nh.subscribe<sensor_msgs::PointCloud2>("/objects/box", 1, callback_box);

  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "/camera/depth/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::Image> camera_sub(nh, "/camera/color/image_raw", 1);

  typedef sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::Image> syncPolicy;

  Synchronizer<syncPolicy> sync(syncPolicy(30), info_sub, camera_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseArray>("/objects/poseArray", 1, callback_pose);
  ros::Subscriber sub_n_points = nh.subscribe<std_msgs::UInt32>("objects/n_points", 1, callback_n_points);
  ros::Subscriber sub_altezza = nh.subscribe<std_msgs::Float32>("object/altezza", 1, callback_altezza);
  ros::Subscriber sub_larhezza = nh.subscribe<std_msgs::Float32>("object/larghezza", 1, callback_larghezza);

  
  
  ros::spin();

  return 0;
}


void callback(const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg, const sensor_msgs::ImageConstPtr& msg_image) {
  
 
	// sensor_msgs::Image -> cv::Mat
	cv::Mat image;
	image = cv_bridge::toCvShare(msg_image, "bgr8")->image;
	int cols = image.cols;  // this is 1280, x-axis of pixels
	int rows = image.rows; //  this is 720,  y-axis of pixels
  
    
	// conversion to pixels
  image_geometry::PinholeCameraModel cam_model;
	cam_model.fromCameraInfo(cameraInfoMsg);
/*
	int x_p,y_p;

	cv::Point3d pt_cv(-pose.position.y, -pose.position.z, pose.position.x);
	cv::Point2d uv;
	uv = cam_model.project3dToPixel(pt_cv);
	x_p = (int)uv.x;
	y_p = (int)uv.y;
  
  cv::Point3d pt_cv1(-pose.position.y - larghezza/2, -pose.position.z + altezza/2, pose.position.x);
  cv::Point3d pt_cv2(-pose.position.y + larghezza/2, -pose.position.z - altezza/2, pose.position.x);
  cv::Point2d uv1, uv2;
  uv1 = cam_model.project3dToPixel(pt_cv1);
  uv2 = cam_model.project3dToPixel(pt_cv2);

  int x1 = (int)uv1.x;
  int x2 = (int)uv2.x;
  int y1 = (int)uv1.y;
  int y2 = (int)uv2.y;

  if( (x_p >= 0) && (x_p <= cols) && (y_p >= 0) && (y_p <= rows)){
  
      //cv::rectangle(image, cv::Point(x_p-offset, y_p-offset), cv::Point(x_p+offset, y_p+offset), CV_RGB(0, 255, 0), 1);
      //cv::putText(image,"target",cv::Point(x_p-offset, y_p-offset),cv::FONT_HERSHEY_DUPLEX,0.8,CV_RGB(0, 255, 0),1);
    cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), CV_RGB(0, 255, 0), 1);
    cv::putText(image,"target",cv::Point(x1, y1),cv::FONT_HERSHEY_DUPLEX,0.8,CV_RGB(0, 255, 0),1);
  }
*/
  if(box.size() != 0){

      std::vector<int> px;
      std::vector<int> py;

      px.clear();
      py.clear();


      for (int i=0; i < box.size(); i++){

         cv::Point3d pt_box(-box.points[i].y, -box.points[i].z, box.points[i].x);
         cv::Point2d uv3 = cam_model.project3dToPixel(pt_box);
         px.push_back((int)uv3.x);
         py.push_back((int)uv3.y);
    
      }

      //base sotto
      cv::line (image, cv::Point(px[0],py[0]), cv::Point(px[1],py[1]), CV_RGB(0, 255, 0), 1, cv::LINE_8, 0);
      cv::line (image, cv::Point(px[1],py[1]), cv::Point(px[2],py[2]), CV_RGB(0, 255, 0), 1, cv::LINE_8, 0);
      cv::line (image, cv::Point(px[2],py[2]), cv::Point(px[3],py[3]), CV_RGB(0, 255, 0), 1, cv::LINE_8, 0);
      cv::line (image, cv::Point(px[3],py[3]), cv::Point(px[0],py[0]), CV_RGB(0, 255, 0), 1, cv::LINE_8, 0);

      //base sopra
      cv::line (image, cv::Point(px[4],py[4]), cv::Point(px[5],py[5]), CV_RGB(0, 255, 0), 1, cv::LINE_8, 0);
      cv::line (image, cv::Point(px[5],py[5]), cv::Point(px[6],py[6]), CV_RGB(0, 255, 0), 1, cv::LINE_8, 0);
      cv::line (image, cv::Point(px[6],py[6]), cv::Point(px[7],py[7]), CV_RGB(0, 255, 0), 1, cv::LINE_8, 0);
      cv::line (image, cv::Point(px[7],py[7]), cv::Point(px[4],py[4]), CV_RGB(0, 255, 0), 1, cv::LINE_8, 0);

      //lati
      cv::line (image, cv::Point(px[0],py[0]), cv::Point(px[4],py[4]), CV_RGB(0, 255, 0), 1, cv::LINE_8, 0);
      cv::line (image, cv::Point(px[1],py[1]), cv::Point(px[5],py[5]), CV_RGB(0, 255, 0), 1, cv::LINE_8, 0);
      cv::line (image, cv::Point(px[2],py[2]), cv::Point(px[6],py[6]), CV_RGB(0, 255, 0), 1, cv::LINE_8, 0);
      cv::line (image, cv::Point(px[3],py[3]), cv::Point(px[7],py[7]), CV_RGB(0, 255, 0), 1, cv::LINE_8, 0);
  }

  cv::imshow( "image", image );
  cv::waitKey(30);
  
}

void callback_pose(const geometry_msgs::PoseArray Pose){

	pose = Pose.poses[0];

} 

void callback_n_points(const std_msgs::UInt32 number_points_msg){

	number_points = (int)number_points_msg.data;

}

void callback_altezza(const std_msgs::Float32 altezza_msg){

  altezza = altezza_msg.data;

}

void callback_larghezza(const std_msgs::Float32 larghezza_msg){

 larghezza = larghezza_msg.data;

}

void callback_box(const sensor_msgs::PointCloud2ConstPtr& box_msg){


  // sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2
  pcl::PCLPointCloud2* box_ = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr boxPtr(box_);
  pcl_conversions::toPCL(*box_msg, *box_);
  
  //pcl::pointcloud2 -> pcl::pointxyz
  pcl::fromPCLPointCloud2(*box_, box);

}