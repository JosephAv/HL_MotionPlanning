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

using namespace message_filters;

pcl::PointCloud<pcl::PointXYZ> centroids;
ros::Publisher pub;
float MIN_Z, MAX_Z, MIN_Y, MAX_Y;
 
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
	
	// pcl::pointcloud2 -> pcl::pointxyzrgb
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromPCLPointCloud2(*cloud_, *cloud);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	//filter on z (wall)
	pcl::PassThrough<pcl::PointXYZRGB> pass_z;
	pass_z.setInputCloud (cloud);
	pass_z.setFilterFieldName ("z");
	pass_z.setFilterLimits (MIN_Z, MAX_Z);
	//pass.setFilterLimitsNegative (true);
	pass_z.filter (*cloud_filtered);

	// filter on y (table, y positive downward)
	pcl::PassThrough<pcl::PointXYZRGB> pass_y;
	pass_y.setInputCloud (cloud_filtered);
	pass_y.setFilterFieldName ("y");
	pass_y.setFilterLimits (MIN_Y, MAX_Y);
	//pass_y.setFilterLimitsNegative (true);
	pass_y.filter (*cloud_filtered);
	
	// pcl::pointxyzrgb -> pcl::pointcloud2 and downsampling
	pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloud2Ptr(cloud2);
	
	pcl::toPCLPointCloud2(*cloud_filtered, *cloud2);
	pcl::PCLPointCloud2 cloud2_filtered;
	
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud2Ptr);
	sor.setLeafSize (0.005f, 0.005f, 0.005f);
	sor.filter (cloud2_filtered);

	// pcl::pointcloud2 -> pcl::pointxyzrgb
	pcl::fromPCLPointCloud2(cloud2_filtered, *cloud_filtered);

	// division of the point cloud into cluster
	int number_object = 0;
	
	if( cloud_filtered->size() > 30 ){
		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud (cloud_filtered);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
		ec.setClusterTolerance (0.02); // 4cm
		ec.setMinClusterSize (30);
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
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
			
			for (const auto& idx : it->indices)
				cloud_cluster->push_back ((*cloud_filtered)[idx]);
				
			//cloud_cluster->width = cloud_cluster->size ();	
			//cloud_cluster->height = 1;				
			//cloud_cluster->is_dense = true;			
		
			number_points.push_back(cloud_cluster->size ());
		
			pcl::PointCloud<pcl::PointXYZRGB> coord = *cloud_cluster;

			float x,y,z,xc,yc,zc,sum;
			short int r,b,g,intensity;
			xc = 0;
			yc = 0;
			zc = 0;
			sum = 0;

			for(int i=0; i<coord.points.size();i++){
			
				r = coord.points[i].r;
				b = coord.points[i].b;
				g = coord.points[i].g;
				intensity = (short int)((float)g+(float)r+(float)g)/3;
				
				// filter points by color
				if (intensity > 150){

			     	x = coord.points[i].x;
		   	     	y = coord.points[i].y;
		   	     	z = coord.points[i].z;
		   	     	
		   	     	// computation of the centroid
		   	     	sum++;
					xc = xc + (x - xc) / sum;
					yc = yc + (y - yc) / sum;
					zc = zc + (z - zc) / sum;
				
				}
	
			}
			
			// if this object has more than 25 points (not black), stores the centroid
			if(sum > 25){
		
				coord_x.push_back(xc);
				coord_y.push_back(yc);
				coord_z.push_back(zc);
			
				number_object++;
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

					cv::rectangle(image, cv::Point(x_p-30, y_p-30), cv::Point(x_p+30, y_p+30), CV_RGB(0, 255, 0), 1);
			}	
		}
	
	// publish a point cloud containing the centroids of the objects
	centroids.width    = number_object;
	centroids.height   = 1;
	centroids.is_dense = false;
	centroids.resize (centroids.width * centroids.height);
	 
	for( int k = 0; k < number_object; k++){
	   
		centroids.points[k].x = coord_z[k];
		centroids.points[k].y = -coord_x[k];
		centroids.points[k].z = -coord_y[k];

	}
	pcl::PCLPointCloud2 centroids2;
	pcl::toPCLPointCloud2(centroids, centroids2);
	sensor_msgs::PointCloud2 output;
	pcl_conversions::moveFromPCL(centroids2, output);
	output.header.frame_id = "camera_link";
	pub.publish(output);
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

	ros::init (argc, argv, "multiple_object_detection_color");
	ros::NodeHandle nh;
	
	nh.getParam("/multiple_object_detection_color/max_distance_z", MAX_Z);
	nh.getParam("/multiple_object_detection_color/min_distance_z", MIN_Z);
	nh.getParam("/multiple_object_detection_color/max_distance_y", MAX_Y);
	nh.getParam("/multiple_object_detection_color/min_distance_y", MIN_Y);

	message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/camera/depth/color/points", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "/camera/depth/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::Image> camera_sub(nh, "/camera/color/image_raw", 1);

	typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo, sensor_msgs::Image> syncPolicy;

	Synchronizer<syncPolicy> sync(syncPolicy(10), points_sub, info_sub, camera_sub);
	sync.registerCallback(boost::bind(&cloud_cb, _1, _2, _3));
	
	pub = nh.advertise<sensor_msgs::PointCloud2>("/objects/centroids", 1);

	ros::spin();

	return 0;
}
