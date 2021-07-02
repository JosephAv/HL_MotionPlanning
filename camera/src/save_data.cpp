#include <iostream>
#include <string>
#include <sstream>
  
#include <ros/ros.h>

#include<fstream>

#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

std::ofstream FilePos, FileVel ,FiledT, FileAngles_and_dot;

void callback_pose(const geometry_msgs::PoseArray Pose){

	geometry_msgs::Pose pose = Pose.poses[0];

	FilePos << pose.position.x << "," << pose.position.y << "," << pose.position.z << std::endl;

}

void callback_twist(const geometry_msgs::Twist twist_msg){

	geometry_msgs::Twist twist = twist_msg;

} 

void callback_n_points(const std_msgs::UInt32 number_points_msg){

	float number_points = (int)number_points_msg.data;

}

void callback_deltaT(const std_msgs::Float32 deltaT_msg){

	float deltaT = deltaT_msg.data;
	FiledT << deltaT << std::endl;

}

void callback_angles_and_dot(const geometry_msgs::Twist angles_and_dot_msg){

  //geometry_msgs::Twist angles_and_dot = angles_and_dot_msg;

  //FileAngles_and_dot << angles_and_dot_msg.linear.x << "," << angles_and_dot_msg.linear.y << "," << angles_and_dot_msg.linear.z << "," << angles_and_dot_msg.angular.x << "," << angles_and_dot_msg.angular.y << "," << angles_and_dot_msg.angular.z << std::endl;

} 


int main (int argc, char** argv) {

  ros::init (argc, argv, "save_data");
  ros::NodeHandle nh;

  FilePos.open("/home/federico/catkin_ws/src/camera/Dati/Dati_Pos.csv");
  FileVel.open("/home/federico/catkin_ws/src/camera/Dati/Dati_Vel.csv");
  FiledT.open("/home/federico/catkin_ws/src/camera/Dati/Dati_dT.csv");
  FileAngles_and_dot.open("/home/federico/catkin_ws/src/camera/Dati/Dati_Angles_and_dot.csv");

  ros::Subscriber sub_twist = nh.subscribe<geometry_msgs::Twist>("/object/Twist", 1, callback_twist);
  ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseArray>("/object/poseArray", 1, callback_pose);

  ros::Subscriber sub_n_points = nh.subscribe<std_msgs::UInt32>("object/n_points", 1, callback_n_points);

  ros::Subscriber sub_deltaT = nh.subscribe<std_msgs::Float32>("/vision_node/deltaT", 1, callback_deltaT);  

  ros::Subscriber sub_angles_and_dot = nh.subscribe<geometry_msgs::Twist>("/object/angles_and_dot", 1, callback_angles_and_dot);
  
  ros::spin();

  return 0;
}