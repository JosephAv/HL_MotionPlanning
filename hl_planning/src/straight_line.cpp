#include <eigen3/Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <cmath>

#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

#include <std_srvs/Trigger.h>

Eigen::VectorXd initial_EE_point(3);
bool initial_pose_init = false;
Eigen::Vector3d force;
Eigen::Vector3d torque;

// Convert xyzrpy vector to geometry_msgs Pose (PRESA DA PANDA-SOFTHAND -> TaskSequencer.cpp)
geometry_msgs::Pose convertVectorToPose(Eigen::VectorXd input_vec) {
  // Creating temporary variables
  geometry_msgs::Pose output_pose;
  Eigen::Affine3d output_affine;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;

  // Getting translation and rotation
  translation << input_vec[0], input_vec[1], input_vec[2];
  output_affine.translation() = translation;
  rotation << 1,  0,  0,
              0, -1,  0,
              0,  0, -1;
  output_affine.linear() = rotation;    
  // Converting to geometry_msgs and returning
  tf::poseEigenToMsg(output_affine, output_pose);
  return output_pose;
}

void robotPoseCallback(const geometry_msgs::PoseStamped& msg) {
  if (!initial_pose_init) {
      geometry_msgs::Pose msg_pose = msg.pose;
      Eigen::Affine3d input_affine;
      Eigen::Vector3d traslazione;
      tf::poseMsgToEigen(msg_pose,input_affine);
      traslazione = input_affine.translation();
      initial_EE_point << traslazione[0], traslazione[1], traslazione[2];
      initial_pose_init = true;
  } 
}

void ftDataCallback(const geometry_msgs::WrenchStamped& msg) {
  force  << msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z;
  torque << msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;
}

int main(int argc, char **argv) {
  //Initialize the node
  ROS_INFO("Stright Line Planner...node initialization...");
  ros::init(argc, argv, "SL_planner");

  ros::NodeHandle node("~");  // private namespace (i.e., "/node_name") node handle
  ros::NodeHandle public_node;// public namespace (i.e., "/" or "/my_ns") node handle 

  //Initialize frame trajectory publisher
  ros::Publisher pub = public_node.advertise<geometry_msgs::PoseStamped>("equilibrium_pose", 1);
  //Initialize starting pose subscriber
  ros::Subscriber robot_pose_sub = public_node.subscribe("/franka_ee_pose", 1, robotPoseCallback);
  //
  ros::Subscriber ft_data_sub = public_node.subscribe("ft_data", 1, ftDataCallback);
  //
  ros::ServiceClient ft_sensor_client = public_node.serviceClient<std_srvs::Trigger>("ft_sensor_calibration");

  //Starting and ending times definition
  double duration;                   // [s]
  double ee_displacement_x;          // [m]
  double ee_displacement_y;          // [m]
  double force_z;                    // [N]
  Eigen::VectorXd final_EE_point(3); // [m]
  Eigen::VectorXd ee_displacement(3);// [m]
  std::int32_t steps_num;            // [adimensional]
  double kPosStep{0.001};            // [m]
  double sampling_time;              // [Hz]

  Eigen::VectorXd actual_pose(3);
  geometry_msgs::Pose actual_pose_msg;
  geometry_msgs::PoseStamped actual_posestamped_msg;
  Eigen::VectorXd ee_vertical_disp(3);
  std::int16_t status = 0;
  std::int32_t index = 0;
  
  if (!node.getParam("contact_force", force_z))               ROS_ERROR("Failed to get contact_force param");
  if (!node.getParam("ee_displacement_x", ee_displacement_x)) ROS_ERROR("Failed to get ee_final_pos_x param");
  if (!node.getParam("ee_displacement_y", ee_displacement_y)) ROS_ERROR("Failed to get ee_final_pos_y param");
  if (!node.getParam("duration", duration))                   ROS_ERROR("Failed to get duration param");

  if (force_z <= 0)  ROS_ERROR("Invalid contact force, must be >= 0");
  if (duration <= 0) ROS_ERROR("Invalid duration value, must be >= 0");

  ee_displacement(0) = ee_displacement_x;
  ee_displacement(1) = ee_displacement_y;
  ee_displacement(2) = 0;
  sampling_time      = ee_displacement.norm() / (duration * kPosStep);
  steps_num          = static_cast<std::int32_t>(round(ee_displacement.norm() / kPosStep));
  
  Eigen::MatrixXd ee_trajectory_0(3,300);
  Eigen::MatrixXd ee_trajectory_1(3,500);
  Eigen::MatrixXd ee_trajectory_2(3,steps_num);

  ros::Rate rate0(10); // [Hz]
  ros::Rate rate1(10); // [Hz]
  ros::Rate rate2(sampling_time); // [Hz]
  
  while (ros::ok()) {
    // check if it is necessary calling callbacks
    ros::spinOnce();
    // switch on status to plan differently based on actual motion condition
    switch (status) {
      case 0: // set initial pose
        if (!initial_pose_init) break;
        //Trajectory computation
        ee_trajectory_0 = initial_EE_point*Eigen::RowVectorXd::Ones(300);
        ROS_INFO("INIT POSE REACHING!!!");
        // trajectory publishing
        rate0.reset();
        for (std::int32_t i = 0; i < 300; ++i) {
          actual_pose                         = ee_trajectory_0.col(i);
          actual_pose_msg                     = convertVectorToPose(actual_pose);
          actual_posestamped_msg.pose         = actual_pose_msg;
          actual_posestamped_msg.header.seq   = i;
          actual_posestamped_msg.header.stamp = ros::Time::now();
          pub.publish(actual_posestamped_msg);
          rate0.sleep();
        }
        ROS_INFO("INIT POSE REACHED!!!");
        initial_pose_init = false;
        status = 1;
        break;
      case 1: // touch the plate respecting the force limit
        if (!initial_pose_init) break;
        // f/t sensor calibration request
        if (ft_sensor_client.call()) ROS_INFO("F/T sensor calibration succesfull");
        else ROS_ERROR("F/T sensor calibration failed");
        // define a vertical displacement
        ee_vertical_disp << 0.0, 0.0, -0.5; // [m]
        // Trajectory computation
        ee_trajectory_1 = ee_vertical_disp*Eigen::RowVectorXd::LinSpaced(500, 0, 1) + initial_EE_point*Eigen::RowVectorXd::Ones(500);
        ROS_INFO("TOUCH REACHING!!!");
        // trajectory publishing
        index = 0;
        rate1.reset();
        for (;;) {
          if (index < 500) actual_pose = ee_trajectory_1.col(++index);
          else             actual_pose = ee_trajectory_1.col(index-1);
          actual_pose_msg                     = convertVectorToPose(actual_pose);
          actual_posestamped_msg.pose         = actual_pose_msg;
          actual_posestamped_msg.header.seq   = index;
          actual_posestamped_msg.header.stamp = ros::Time::now();
          pub.publish(actual_posestamped_msg);
          ros::spinOnce();
          if (std::fabs(force.z()) > force_z) break;
          rate1.sleep();
        }
        ROS_INFO("TOUCH REACHED!!!");
        initial_pose_init = false;
        status = 2;
        break;
      case 2: // move along the ee_displacement direction
        if (!initial_pose_init) break;
        //Trajectory computation
        ee_trajectory_2 = ee_displacement*Eigen::RowVectorXd::LinSpaced(steps_num, 0, 1) + initial_EE_point*Eigen::RowVectorXd::Ones(steps_num);
        // trajectory publishing
        rate2.reset();
        for (std::int32_t i = 0; i < steps_num; ++i) {
          actual_pose                         = ee_trajectory_2.col(i);
          actual_pose_msg                     = convertVectorToPose(actual_pose);
          actual_posestamped_msg.pose         = actual_pose_msg;
          actual_posestamped_msg.header.seq   = i;
          actual_posestamped_msg.header.stamp = ros::Time::now();
          pub.publish(actual_posestamped_msg);
          rate2.sleep();
        }
        initial_pose_init = false;
        status = 3;
        break;
      default: break; // do nothing
    }
  }
  ROS_INFO("TASK COMPLETED");
}