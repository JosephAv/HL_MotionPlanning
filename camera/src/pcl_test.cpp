// PCL Example using ROS and CPP
          

// Include the ROS library
#include <ros/ros.h>

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

// Topics
static const std::string IMAGE_TOPIC = "/camera/depth/color/points";
static const std::string PUBLISH_TOPIC = "/pcl/points";

// ROS Publisher
ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	/*    // Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.02, 0.02, 0.02);
	sor.filter (cloud_filtered);*/

	//pointcloud2 -> point xyz
	pcl::PointCloud<pcl::PointXYZ>::Ptr coord(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ> coord;
	pcl::fromPCLPointCloud2(*cloud, *coord);
	/*	
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (coord);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 0.75);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*coord);
	*/ 
	//filtro su z (parete)
	pcl::PassThrough<pcl::PointXYZ> pass_z;
	pass_z.setInputCloud (coord);
	pass_z.setFilterFieldName ("z");
	pass_z.setFilterLimits (0.0, 0.5);
	//pass.setFilterLimitsNegative (true);
	pass_z.filter (*coord);

	//filtro su y (tavolo, y positiva verso il basso)
	pcl::PassThrough<pcl::PointXYZ> pass_y;
	pass_y.setInputCloud (coord);
	pass_y.setFilterFieldName ("y");
	pass_y.setFilterLimits (-1, 0.15);
	//pass_y.setFilterLimitsNegative (true);
	pass_y.filter (*coord);

	//pointxyz -> pointcloud2
	pcl::toPCLPointCloud2(*coord, cloud_filtered);

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::moveFromPCL(cloud_filtered, output);


	// Publish the data
	pub.publish (output);

}

int main (int argc, char** argv)
{
	// Initialize the ROS Node "roscpp_pcl_example"
	ros::init (argc, argv, "pcl_test");
	ros::NodeHandle nh;
	ros::Rate loop_rate(30);

	// Print "Hello" message with node name to the terminal and ROS log file
	ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

	// Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
	ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

	// Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
	pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

	// Spin
	ros::spin();

	// Success
	return 0;
}
