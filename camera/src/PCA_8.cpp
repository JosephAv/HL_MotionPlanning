// camera with transformation matrix T_0_cam


#include <iostream>
#include <string>
#include <sstream>
	
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
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/common/pca.h>
#include <Eigen/Dense>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Geometry> 
#include <math.h>

#include <pcl/sample_consensus/sac_model_plane.h>

#define PI 3.1415926535


using namespace message_filters;

namespace camera{

	struct Time_t {

	ros::Time time_old;
	ros::Time time_now;

	};
}

camera::Time_t tempo;

float MIN_Z, MAX_Z, MIN_Y, MAX_Y, THETA, PSI, PHI,ALPHA, POS_CAM_X,POS_CAM_Y,POS_CAM_Z;
pcl::PointCloud<pcl::PointXYZ> centroids;
ros::Publisher pub2;
ros::Publisher pub5;

geometry_msgs::PoseStamped pose;
geometry_msgs::PoseArray positions;
geometry_msgs::Twist velocity;
geometry_msgs::PoseArray poseArray;
float delta_T;
Eigen::Matrix4f T_0_cam;




void updatePosition (geometry_msgs::PoseStamped);
void initializationPositions();
void computeVelocity();
void computeAngularVelocity(Eigen::Quaternionf);
void computeDeltaT();
Eigen::Quaternionf productOfQuaternion(Eigen::Quaternionf,Eigen::Quaternionf);
void initFrameCamera();

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg, const sensor_msgs::ImageConstPtr& msg_image) {
	
	computeDeltaT();
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromPCLPointCloud2(*cloud_, *cloud);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	

	pcl::transformPointCloud (*cloud, *cloud_filtered, T_0_cam.inverse()); 
	
	//filter on z (wall)
	pcl::PassThrough<pcl::PointXYZRGB> pass_z;
	pass_z.setInputCloud (cloud_filtered);
	pass_z.setFilterFieldName ("z");
	pass_z.setFilterLimits (MIN_Z, MAX_Z);
	//pass_z.setFilterLimitsNegative (true);
	pass_z.filter (*cloud_filtered);

	// filter on y (table, y positive downward)
	pcl::PassThrough<pcl::PointXYZRGB> pass_y;
	pass_y.setInputCloud (cloud_filtered);
	pass_y.setFilterFieldName ("y");
	pass_y.setFilterLimits (MIN_Y, MAX_Y);
	//pass_y.setFilterLimitsNegative (true);
	pass_y.filter (*cloud_filtered);

	//pcl::transformPointCloud (*cloud_filtered, *cloud_filtered, T_0_cam); 

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  	short int r,b,g,intensity;

	for (int i = 0; i < (*cloud_filtered).size(); i++){

	    r = cloud_filtered->points[i].r;
		b = cloud_filtered->points[i].b;
		g = cloud_filtered->points[i].g;
		intensity = (short int)((float)g+(float)r+(float)g)/3;

	    if (intensity > 150){

	    		inliers->indices.push_back(i);
	    }
	  }
	  
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);
	//extract.setNegative(true);
	extract.filter(*cloud_filtered);

	pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloud2Ptr(cloud2);
	
	pcl::toPCLPointCloud2(*cloud_filtered, *cloud2);
	pcl::PCLPointCloud2 cloud2_filtered;
	
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud2Ptr);
	sor.setLeafSize (0.005f, 0.005f, 0.005f);
	sor.filter (cloud2_filtered);

	pcl::fromPCLPointCloud2(cloud2_filtered, *cloud_filtered);

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output2;
	//output2.header.frame_id = "camera_link";

	pcl_conversions::moveFromPCL(cloud2_filtered, output2);
	

	// Publish the data
	pub5.publish (output2);

	// division of the point cloud into cluster
	int number_object = 0;
	poseArray.poses.clear();
	
	if( cloud_filtered->size() > 30 ){
		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud (cloud_filtered);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
		ec.setClusterTolerance (0.04); // 4cm
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
				
			cloud_cluster->width = cloud_cluster->size ();	
			cloud_cluster->height = 1;				
			cloud_cluster->is_dense = true;			
		
			number_points.push_back(cloud_cluster->size ());
		
			pcl::PointCloud<pcl::PointXYZRGB> coord = *cloud_cluster;
			

			// Placeholder for the 3x3 covariance matrix at each surface patch
			Eigen::Matrix3f covariance_matrix;
			Eigen::Matrix3f covariance_matrix2;
			// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
			Eigen::Vector4f xyz_centroid;

			// Estimate the XYZ centroid
			compute3DCentroid (coord, xyz_centroid);

			// Compute the 3x3 covariance matrix
			computeCovarianceMatrixNormalized (coord, xyz_centroid, covariance_matrix);
			
			//computeMeanAndCovarianceMatrix (coord, covariance_matrix, xyz_centroid);

			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covariance_matrix);
			if (eigensolver.info() != Eigen::Success){
				ROS_ERROR("error eigensolver");
			}

			Eigen::Matrix3f eigenvector_matrix = eigensolver.eigenvectors();

			eigenvector_matrix.col(2) = eigenvector_matrix.col(0).cross(eigenvector_matrix.col(1)); //correct vertical between main directions
			eigenvector_matrix.col(0) = eigenvector_matrix.col(1).cross(eigenvector_matrix.col(2));
			eigenvector_matrix.col(1) = eigenvector_matrix.col(2).cross(eigenvector_matrix.col(0)); 

			Eigen::Quaternionf q(eigenvector_matrix.transpose());

			//ROS_INFO("quaternion %f %f %f %f", q.w() , q.x() ,q.y(), q.z());

			if(q.y()*q.z() > 0){

				Eigen::Matrix3f Ry;

				Ry << cosf(PI), 0,  sinf(PI),
	   				0,  1,  0,
	   				-sinf(PI),  0,  cosf(PI);

	   			Eigen::Matrix3f Rz;

	   			Rz << cosf(PI), -sinf(PI),  0,
   					sinf(PI),  cosf(PI),  0,
   					0,  0,  1;

	   			eigenvector_matrix.transpose() = Ry*Rz*eigenvector_matrix.transpose();
			}

			Eigen::Quaternionf qf(eigenvector_matrix.transpose());

			pose.pose.position.x = xyz_centroid(2);
			pose.pose.position.y = -xyz_centroid(0);
			pose.pose.position.z = -xyz_centroid(1);

			pose.pose.orientation.w = qf.w(); 
			pose.pose.orientation.x = -qf.z(); 
			pose.pose.orientation.y = qf.x(); 
			pose.pose.orientation.z = qf.y();

			pose.header.frame_id = "camera_link";
			pose.header.stamp = ros::Time();

			updatePosition(pose);
			computeVelocity();
			computeAngularVelocity(qf);

			pose.pose.position.x = positions.poses[1].position.x;
			pose.pose.position.y = positions.poses[1].position.y;
			pose.pose.position.z = positions.poses[1].position.z;


			coord_x.push_back(-positions.poses[1].position.y);
			coord_y.push_back(-positions.poses[1].position.z);
			coord_z.push_back(positions.poses[1].position.x);

			poseArray.poses.push_back(pose.pose);

			number_object++;

			
		} 

		
		pub2.publish(poseArray);

		//ROS_INFO("number of object = %d", number_object);
		
		// conversion to pixels
		image_geometry::PinholeCameraModel cam_model;
		cam_model.fromCameraInfo(cameraInfoMsg);
		//std::vector<int> pixel_x;	
		//std::vector<int> pixel_y;
		int x_p,y_p;
		Eigen::Vector4f coord2;
		// visualization 
		for (int i = 0; i < number_object; i++){

			coord2 = {coord_x[i], coord_y[i], coord_z[i], 1};
			coord2 = T_0_cam*coord2;
	
			cv::Point3d pt_cv(coord2[0], coord2[1], coord2[2]);
	      		cv::Point2d uv;
	      		uv = cam_model.project3dToPixel(pt_cv);
	      		x_p = (int)uv.x;
	      		y_p = (int)uv.y;
	      		//pixel_x.push_back((int)uv.x);
				//pixel_y.push_back((int)uv.y);
		
			if( (x_p >= 0) && (x_p <= cols) && (y_p >= 0) && (y_p <= rows)){
			
					cv::rectangle(image, cv::Point(x_p-30, y_p-30), cv::Point(x_p+30, y_p+30), CV_RGB(0, 255, 0), 1);
					//cv::putText(image,"target",cv::Point(x_p-30, y_p-30),cv::FONT_HERSHEY_DUPLEX,0.8,CV_RGB(0, 255, 0),1);
			}
		
				
		}
	
	
	 centroids.width    = number_object;
	 centroids.height   = 1;
	 centroids.is_dense = false;
	 centroids.resize (centroids.width * centroids.height);
	 
	 for( int k = 0; k < number_object; k++){
	   
	   centroids.points[k].x = coord_z[k];
	   centroids.points[k].y = -coord_x[k];
	   centroids.points[k].z = -coord_y[k];
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

	ros::init (argc, argv, "PCA_8");
	ros::NodeHandle nh;

	nh.getParam("/PCA_8/max_distance_z", MAX_Z);
	nh.getParam("/PCA_8/min_distance_z", MIN_Z);
	nh.getParam("/PCA_8/max_distance_y", MAX_Y);
	nh.getParam("/PCA_8/min_distance_y", MIN_Y);
	nh.getParam("/PCA_8/pos_camera_x", POS_CAM_X);
	nh.getParam("/PCA_8/pos_camera_y", POS_CAM_Y);
	nh.getParam("/PCA_8/pos_camera_z", POS_CAM_Z);
	nh.getParam("/PCA_8/roll", PHI);
	nh.getParam("/PCA_8/pitch", THETA);
	nh.getParam("/PCA_8/yaw", PSI);
	nh.getParam("/PCA_8/inclination_camera", ALPHA);

	
	initFrameCamera();
	initializationPositions();

	message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/camera/depth/color/points", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "/camera/depth/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::Image> camera_sub(nh, "/camera/color/image_raw", 1);

	typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo, sensor_msgs::Image> syncPolicy;

	Synchronizer<syncPolicy> sync(syncPolicy(30), points_sub, info_sub, camera_sub);
	sync.registerCallback(boost::bind(&cloud_cb, _1, _2, _3));
	
	pub2 = nh.advertise<geometry_msgs::PoseArray>("/objects/poseArray", 1);
	pub5 = nh.advertise<sensor_msgs::PointCloud2>("objects/pcl", 1);


	ros::spin();

	return 0;
}




void updatePosition (geometry_msgs::PoseStamped newPose){

	geometry_msgs::PoseStamped oldPose;
	
	oldPose.pose = positions.poses[0];
	positions.poses[0] = positions.poses[1];
	positions.poses[1] = newPose.pose;

}

void initializationPositions(){

	geometry_msgs::Pose init;

	init.position.x = -1;
	init.position.y = -1;
	init.position.z = -1;
	init.orientation.w = 1;
	init.orientation.x = 0;
	init.orientation.y = 0;
	init.orientation.z = 0;

	positions.poses.push_back(init);
	positions.poses.push_back(init);
	positions.header.frame_id = "camera_link";
	positions.header.stamp = ros::Time();

	poseArray.header.frame_id = "camera_link";
	poseArray.header.stamp = ros::Time();

	tempo.time_old = ros::Time::now();
	tempo.time_now = ros::Time::now();

} 

void computeVelocity(){

	if (positions.poses[0].position.x != -1){

		float a = (positions.poses[0].position.x - positions.poses[1].position.x);
		a = a*a;
		float b = (positions.poses[0].position.y - positions.poses[1].position.y);
		b = b*b;
		float c = (positions.poses[0].position.z - positions.poses[1].position.z);
		c = c*c;
		float dist = sqrt((a+b+c)/3);

		if(dist <= 0.0015 ){
			//ROS_INFO("è fermo");
			positions.poses[1].position.x = (positions.poses[1].position.x + positions.poses[0].position.x)/2;
			positions.poses[1].position.y = (positions.poses[1].position.y + positions.poses[0].position.y)/2;
			positions.poses[1].position.z = (positions.poses[1].position.z + positions.poses[0].position.z)/2;

			velocity.linear.x = 0;
			velocity.linear.y = 0;
			velocity.linear.z = 0;
		}
		else{
			
			velocity.linear.x = (positions.poses[1].position.x - positions.poses[0].position.x)/delta_T;
			velocity.linear.y = (positions.poses[1].position.y - positions.poses[0].position.y)/delta_T;
			velocity.linear.z = (positions.poses[1].position.z - positions.poses[0].position.z)/delta_T;
			ROS_INFO("si muove con v = (%f,%f,%f)",velocity.linear.x, velocity.linear.y, velocity.linear.z);
		}
	}

}

void computeAngularVelocity(Eigen::Quaternionf qf){

	float a = (positions.poses[0].orientation.x - positions.poses[1].orientation.x);
	
	float b = (positions.poses[0].orientation.y - positions.poses[1].orientation.y);
	
	float c = (positions.poses[0].orientation.z - positions.poses[1].orientation.z);
	
	float d = (positions.poses[0].orientation.w - positions.poses[1].orientation.w);
	

	float dist = 0.01;

	if (a > dist || a < -dist ||b > dist || b < -dist || c > dist || c < -dist|| d > dist || d < -dist) {

		float qw_dot = (positions.poses[1].orientation.w - positions.poses[0].orientation.w)/delta_T;
		float qx_dot = (positions.poses[1].orientation.x - positions.poses[0].orientation.x)/delta_T;
		float qy_dot = (positions.poses[1].orientation.y - positions.poses[0].orientation.y)/delta_T;
		float qz_dot = (positions.poses[1].orientation.z - positions.poses[0].orientation.z)/delta_T;

		Eigen::Vector4f q_dot = {qw_dot, qx_dot, qy_dot, qz_dot};
		Eigen::Vector4f W;
		Eigen::Quaternionf q2 = qf.inverse();

		float xd =  qx_dot * q2.w() + qy_dot * q2.z() - qz_dot * q2.y() + qw_dot * q2.x();
	    float yd = -qx_dot * q2.z() + qy_dot * q2.w() + qz_dot * q2.x() + qw_dot* q2.y();
	    float zd =  qx_dot * q2.y() - qy_dot * q2.x() + qz_dot * q2.w() + qw_dot * q2.z();
	    float wd = -qx_dot * q2.x() - qy_dot * q2.y() - qz_dot * q2.z() + qw_dot * q2.w();

	    Eigen::Vector4f q_dot_q_star = {wd, xd, yd, zd};

		W = 2*q_dot_q_star;

		velocity.angular.x = W(1);
		velocity.angular.y = W(2);
		velocity.angular.z = W(3);

		ROS_INFO("omega = (	%f,%f,%f)",velocity.angular.x, velocity.angular.y, velocity.angular.z);

	}

	else{

		positions.poses[1].orientation.x = (positions.poses[1].orientation.x + positions.poses[0].orientation.x)/2;
		positions.poses[1].orientation.y = (positions.poses[1].orientation.y + positions.poses[0].orientation.y)/2;
		positions.poses[1].orientation.z = (positions.poses[1].orientation.z + positions.poses[0].orientation.z)/2;
		positions.poses[1].orientation.w = (positions.poses[1].orientation.w + positions.poses[0].orientation.w)/2;

		velocity.angular.x = 0;
		velocity.angular.y = 0;
		velocity.angular.z = 0;
	}

	//ROS_INFO("omega = (	%f,%f,%f)",velocity.angular.x, velocity.angular.y, velocity.angular.z);
	//ROS_INFO("quaternion = (%f,%f,%f,%f)",qf.w(),qf.x(),qf.y(),qf.z());
	//ROS_INFO("abcd = (%f,%f,%f,%f)",a,b,c,d);




}

void computeDeltaT(){


	tempo.time_old = tempo.time_now;
	tempo.time_now = ros::Time::now();

	uint32_t delta_sec = tempo.time_now.sec - tempo.time_old.sec;
	uint32_t delta_nsec = tempo.time_now.nsec - tempo.time_old.nsec;
	uint32_t delta_T2 = (delta_sec * 1000000000 + delta_nsec);
	delta_T = (float)delta_T2/1000000000;
	//ROS_INFO("delta_T = %d, delta_T2 = %lf",delta_T, delta_T2);
}

Eigen::Quaternionf productOfQuaternion(Eigen::Quaternionf q1,Eigen::Quaternionf q2){

	Eigen::Quaternionf q;

	q.x() =  q1.x() * q2.w() + q1.y()  * q2.z() - q1.z() * q2.y() + q1.w() * q2.x();
    q.y() = -q1.x()  * q2.z() + q1.y() * q2.w() + q1.z() * q2.x() + q1.w()* q2.y();
    q.z() =  q1.x()  * q2.y() - q1.y() * q2.x() + q1.z() * q2.w() + q1.w() * q2.z();
    q.w() = -q1.x()  * q2.x() - q1.y() * q2.y() - q1.z() * q2.z() + q1.w() * q2.w();

    return q;

}

void initFrameCamera(){

	Eigen::Matrix3f R_roll,R_pitch, R_yaw, R_0_cam;
	Eigen::Vector3f pos;

	R_roll << 1, 0, 0,
				0, cosf((PHI+ALPHA)*PI/180),-sinf((PHI+ALPHA)*PI/180),
				0, sinf((PHI+ALPHA)*PI/180),cosf((PHI+ALPHA)*PI/180);

	R_pitch << cosf(THETA*PI/180), 0, sinf(THETA*PI/180),
				0, 1,0,
				-sinf(THETA*PI/180), 0,cosf(THETA*PI/180);

	R_yaw << cosf(PSI*PI/180),-sinf(PSI*PI/180), 0,
			 sinf(PSI*PI/180),cosf(PSI*PI/180) , 0,
				0, 0, 1 ;

	R_0_cam = R_yaw*R_pitch*R_roll;
	pos = {POS_CAM_X,POS_CAM_Y,POS_CAM_Z};

	T_0_cam = Eigen::Matrix4f::Identity();
	T_0_cam.block<3, 3>(0, 0) = R_0_cam;
	T_0_cam.block<3, 1>(0, 3) = pos.transpose();



}