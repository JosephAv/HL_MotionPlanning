// first run: roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

#include <iostream>
	
#include <ros/ros.h>

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

#define MIN_Z 0.0
#define MAX_Z 0.9	// maximum distance from the target [m]
#define MIN_Y -1
#define MAX_Y 0.2	// distance table-camera [m]

using namespace message_filters;

 
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg, const sensor_msgs::ImageConstPtr& msg_image) {
	
	// sensor_msgs::Image -> cv::Mat
	cv::Mat image;
	image = cv_bridge::toCvShare(msg_image, "bgr8")->image;
	int cols = image.cols;  // this is 1280, x-axis of pixels
	int rows = image.rows; //  this is 720,  y-axis of pixels
	
	// sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2
	pcl::PCLPointCloud2* cloud_ = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_);
	pcl_conversions::toPCL(*cloud_msg, *cloud_);
	
	// pcl::pointcloud2 -> pcl::pointxyz
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*cloud_, *cloud);

	// Create the filtering object: downsample the dataset using a leaf size of 2 cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud (cloud);
	vg.setLeafSize (0.02f, 0.02f, 0.02f);
	vg.filter (*cloud_filtered);
	
	//filter on z (wall)
	pcl::PassThrough<pcl::PointXYZ> pass_z;
	pass_z.setInputCloud (cloud_filtered);
	pass_z.setFilterFieldName ("z");
	pass_z.setFilterLimits (MIN_Z, MAX_Z);
	//pass.setFilterLimitsNegative (true);
	pass_z.filter (*cloud_filtered);

	// filter on y (table, y positive downward)
	pcl::PassThrough<pcl::PointXYZ> pass_y;
	pass_y.setInputCloud (cloud_filtered);
	pass_y.setFilterFieldName ("y");
	pass_y.setFilterLimits (MIN_Y, MAX_Y);
	//pass_y.setFilterLimitsNegative (true);
	pass_y.filter (*cloud_filtered);

	// division of the point cloud into cluster
	int number_object = 0;
	
	if( cloud_filtered->size() > 20 ){
		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (cloud_filtered);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (0.04); // 4cm
		ec.setMinClusterSize (10);
		//ec.setMaxClusterSize (25000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud_filtered);
		ec.extract (cluster_indices);

		std::vector<uint32_t> number_points;
		std::vector<float> coord_x;	
		std::vector<float> coord_y;
		std::vector<float> coord_z;
	
	
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			
			for (const auto& idx : it->indices)
				cloud_cluster->push_back ((*cloud_filtered)[idx]);
				
			//cloud_cluster->width = cloud_cluster->size ();	
			//cloud_cluster->height = 1;				
			//cloud_cluster->is_dense = true;			
		
			number_points.push_back(cloud_cluster->size ());
		
			pcl::PointCloud<pcl::PointXYZ> coord = *cloud_cluster;
			float x,y,z,xc,yc,zc,sum;

			for(int i=0; i<coord.points.size();i++){

			     	x = coord.points[i].x;
		   	     	y = coord.points[i].y;
		   	     	z = coord.points[i].z;
		   	     	
		   	     	sum++;
				xc = xc + (x - xc) / sum;
				yc = yc + (y - yc) / sum;
				zc = zc + (z - zc) / sum;
	
			}
		
			coord_x.push_back(x);
			coord_y.push_back(y);
			coord_z.push_back(z);
		
			number_object++;
		}

		//ROS_INFO("number of object = %d", number_object);
		
		// look for the largest object (robot) and the closest object (target)
		uint32_t max = 0;
		float min_distance = MAX_Z;
		int index_max;
		int index_min;
		for(int j = 0; j < number_object; j++){
			if( number_points[j] > max ) {
				max = number_points[j];
				index_max = j;
			}
			
			if( coord_z[j] < min_distance ){
				min_distance = coord_z[j];
				index_min = j;
			}
	      			
		}
		
		// conversion to pixels
		image_geometry::PinholeCameraModel cam_model;
		cam_model.fromCameraInfo(cameraInfoMsg);
		//std::vector<int> pixel_x;	
		//std::vector<int> pixel_y;
		int x_p,y_p;
	
		// visualization 
		for (int i = 0; i < number_object; i++){
	
			cv::Point3d pt_cv(coord_x[i], coord_y[i], coord_z[i]);
	      		cv::Point2d uv;
	      		uv = cam_model.project3dToPixel(pt_cv);
	      		x_p = (int)uv.x;
	      		y_p = (int)uv.y;
	      		//pixel_x.push_back((int)uv.x);
			//pixel_y.push_back((int)uv.y);
		
			if( (x_p >= 0) && (x_p <= cols) && (y_p >= 0) && (y_p <= rows)){
				// if there is only one object, this is the target
				if( number_object == 1 ){
					cv::rectangle(image, cv::Point(x_p-30, y_p-30), cv::Point(x_p+30, y_p+30), CV_RGB(0, 255, 0), 1);
					cv::putText(image,"target",cv::Point(x_p-30, y_p-30),cv::FONT_HERSHEY_DUPLEX,0.8,CV_RGB(0, 255, 0),1);
				}
				// the largest object is the robot
				else if( i == index_max ){	
					cv::rectangle(image, cv::Point(x_p-30, y_p-30), cv::Point(x_p+30, y_p+30), CV_RGB(255, 0, 0), 1);
					cv::putText(image,"largest object (robot)",cv::Point(x_p-30, y_p-30),cv::FONT_HERSHEY_DUPLEX,0.8,CV_RGB(255, 0, 0),1);
				}
				// the closest object is the target 
				else if( i == index_min ){
					cv::rectangle(image, cv::Point(x_p-30, y_p-30), cv::Point(x_p+30, y_p+30), CV_RGB(0, 255, 0), 1);
					cv::putText(image,"smallest/closest object (target)",cv::Point(x_p-30, y_p-30),cv::FONT_HERSHEY_DUPLEX,0.8,CV_RGB(0, 255, 0),1);
				}
				// otherwise it is an object of no interest
				else{
					cv::rectangle(image, cv::Point(x_p-30, y_p-30), cv::Point(x_p+30, y_p+30), CV_RGB(255, 255, 255), 1);
					cv::putText(image,"object of no interest",cv::Point(x_p-30, y_p-30),cv::FONT_HERSHEY_DUPLEX,0.8,CV_RGB(255, 255, 255),1);
				
				}
			}
	
		}
	
	}
	//else
	//ROS_INFO("number of object = 0");
	
	std::stringstream ss;
    	ss << "number of object =" << number_object << "";
    	
	cv::putText(image,ss.str (),cv::Point(0, rows-10),cv::FONT_HERSHEY_DUPLEX,0.8,CV_RGB(255, 255, 255),1);
	cv::imshow( "image", image );

	cv::waitKey(30);
}




int main (int argc, char** argv) {

	ros::init (argc, argv, "multiple_object_detection");
	ros::NodeHandle nh;

	message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/camera/depth/color/points", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "/camera/depth/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::Image> camera_sub(nh, "/camera/color/image_raw", 1);

	typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo, sensor_msgs::Image> syncPolicy;

	Synchronizer<syncPolicy> sync(syncPolicy(10), points_sub, info_sub, camera_sub);
	sync.registerCallback(boost::bind(&cloud_cb, _1, _2, _3));

	ros::spin();

	return 0;
}

