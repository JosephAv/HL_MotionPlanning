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

	//pointcloud2 -> point xyz
	pcl::PointCloud<pcl::PointXYZ>::Ptr coord(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*cloud, *coord);

	
	//filtro su z (parete)
	pcl::PassThrough<pcl::PointXYZ> pass_z;
	pass_z.setInputCloud (coord);
	pass_z.setFilterFieldName ("z");
	pass_z.setFilterLimits (0.0, 0.9);
	//pass.setFilterLimitsNegative (true);
	pass_z.filter (*coord);

	//filtro su y (tavolo, y positiva verso il basso)
	pcl::PassThrough<pcl::PointXYZ> pass_y;
	pass_y.setInputCloud (coord);
	pass_y.setFilterFieldName ("y");
	pass_y.setFilterLimits (-1, 0.2);
	//pass_y.setFilterLimitsNegative (true);
	pass_y.filter (*coord);

	// pcl::pointxyz -> pcl::pointcloud2 and downsampling
	pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloud2Ptr(cloud2);
	
	pcl::toPCLPointCloud2(*coord, *cloud2);
	
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud2Ptr);
	sor.setLeafSize (0.005f, 0.005f, 0.005f);
	sor.filter (cloud_filtered);

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::moveFromPCL(cloud_filtered, output);

	// Publish the data
	pub.publish (output);

}

int main (int argc, char** argv)
{

	ros::init (argc, argv, "pcl_test");
	ros::NodeHandle nh;
	ros::Rate loop_rate(30);

	ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

	pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

	ros::spin();

	
	return 0;
}
