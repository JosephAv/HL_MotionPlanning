// first run: roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

#include <iostream>

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>

#include <image_geometry/pinhole_camera_model.h>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/PointStamped.h>

using namespace message_filters;

ros::Publisher pub;
geometry_msgs::PointStamped centroid;
float MIN_Z, MAX_Z, MIN_Y, MAX_Y;

		


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg, const sensor_msgs::ImageConstPtr& msg_image) {

	// sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// pcl::pointcloud2 -> pcl::pointxyzrgb
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromPCLPointCloud2(*cloud, *cloud_xyzrgb);

	//filter on z (wall)
	pcl::PassThrough<pcl::PointXYZRGB> pass_z;
	pass_z.setInputCloud (cloud_xyzrgb);
	pass_z.setFilterFieldName ("z");
	pass_z.setFilterLimits (MIN_Z, MAX_Z);
	//pass.setFilterLimitsNegative (true);
	pass_z.filter (*cloud_xyzrgb);

	// filter on y (table, y positive downward)
	pcl::PassThrough<pcl::PointXYZRGB> pass_y;
	pass_y.setInputCloud (cloud_xyzrgb);
	pass_y.setFilterFieldName ("y");
	pass_y.setFilterLimits (MIN_Y, MAX_Y);
	//pass_y.setFilterLimitsNegative (true);
	pass_y.filter (*cloud_xyzrgb);

	pcl::PointCloud<pcl::PointXYZRGB> coord_filtered = *cloud_xyzrgb;

	float x,y,z,xc,yc,zc,sum;
	short int r,b,g,intensity;
	xc = 0;
	yc = 0;
	zc = 0;
	sum = 0;
	// computation of the centroid with color discrimination
	
	for(int i=0; i<coord_filtered.points.size();i++){
		
		r = coord_filtered.points[i].r;
		b = coord_filtered.points[i].b;
		g = coord_filtered.points[i].g;
		intensity = (short int)((float)g+(float)r+(float)g)/3;
		
		if (intensity > 150){
		
	     	x = coord_filtered.points[i].x;
   	     	y = coord_filtered.points[i].y;
   	     	z = coord_filtered.points[i].z;
   	     	
   	     	sum++;

			xc = xc + (x - xc) / sum;
			yc = yc + (y - yc) / sum;
			zc = zc + (z - zc) / sum;
		
		}

	}
	
	int x_p = -1;
	int y_p = -1;
	
	// object detected if it has more than 100 points
	if(sum > 100){
		
		// publish the centroid on the topic /object/centroid
		centroid.point.x = zc;
		centroid.point.y = -xc;
		centroid.point.z = -yc;
		centroid.header.frame_id = "camera_link";
		centroid.header.stamp = ros::Time();
		pub.publish(centroid);
		
		// conversion to pixels
		image_geometry::PinholeCameraModel cam_model;
		cam_model.fromCameraInfo(cameraInfoMsg);
		cv::Point3d pt_cv(xc, yc, zc);
  		cv::Point2d uv;
  		uv = cam_model.project3dToPixel(pt_cv);
  		x_p = (int)uv.x;
		y_p = (int)uv.y;
		
		ROS_INFO("object detected in (%f, %f, %f)",xc,yc,zc);
	}
	
	// visualization 
	
	// sensor_msgs::Image -> cv::Mat
	cv::Mat image;
	image = cv_bridge::toCvShare(msg_image, "bgr8")->image;
	int cols = image.cols;  // this is 1280, x-axis of pixels
	int rows = image.rows; //  this is 720,  y-axis of pixels
	
	// draw a rectangle on the object
	if( (x_p >= 0) && (x_p <= cols) && (y_p >= 0) && (y_p <= rows))
	rectangle(image, cv::Point(x_p-30, y_p-30), cv::Point(x_p+30, y_p+30), cv::Scalar(0, 0, 255), 1);
	
	// create a windows called "image" in which the image is displayed
	imshow( "image", image );

	//pub.publish (output);
	cv::waitKey(30);

}
 

int main (int argc, char** argv) {

	ros::init (argc, argv, "single_object_detection");
	ros::NodeHandle nh;

	nh.getParam("/single_object_detection/max_distance_z", MAX_Z);
	nh.getParam("/single_object_detection/min_distance_z", MIN_Z);
	nh.getParam("/single_object_detection/max_distance_y", MAX_Y);
	nh.getParam("/single_object_detection/min_distance_y", MIN_Y);

	message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/camera/depth/color/points", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "/camera/depth/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::Image> camera_sub(nh, "/camera/color/image_raw", 1);

	typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo, sensor_msgs::Image> syncPolicy;

	Synchronizer<syncPolicy> sync(syncPolicy(30), points_sub, info_sub, camera_sub);
	sync.registerCallback(boost::bind(&cloud_cb, _1, _2, _3));

	pub = nh.advertise<geometry_msgs::PointStamped>("/object/centroid", 1);
	ros::spin();

	return 0;
}
























