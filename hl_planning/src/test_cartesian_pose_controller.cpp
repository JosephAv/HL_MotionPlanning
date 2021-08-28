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

int main(int argc, char **argv) {
  //Initialize the node
  ROS_INFO("Test planner for cartesian pose controller...node initialization...");
  ros::init(argc, argv, "Test_planner");

  ros::NodeHandle node("~");  // private namespace (i.e., "/node_name") node handle
  ros::NodeHandle public_node;// public namespace (i.e., "/" or "/my_ns") node handle 

  //Initialize frame trajectory publisher
  ros::Publisher pub = public_node.advertise<geometry_msgs::PoseStamped>("equilibrium_pose", 1);
  //Initialize starting pose subscriber
  ros::Subscriber robot_pose_sub = public_node.subscribe("/franka_ee_pose", 1, robotPoseCallback);
  
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
  std::int16_t status = 2;
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
  
  Eigen::MatrixXd ee_trajectory_0(3,100);
  Eigen::MatrixXd ee_trajectory_1(3,500);
  Eigen::MatrixXd ee_trajectory_2(3,steps_num);

  ros::Rate rate0(10); // [Hz]
  ros::Rate rate1(10); // [Hz]
  ros::Rate rate2(sampling_time); // [Hz]
  
  // qi = 1.02;
  // qf = qi + 0.20;
  // tf = 5;
  // a1 = 0;
  // a2 = 3*(qf-qi)/tf^2;
  // a3 = -2*(qf-qi)/tf^3;
  // a0 = qi;
  // ee_trajectory_2 = a0 + a1*t + a2*t.^2 + a3*t.^3;
  
  double a3{10*ee_displacement_y/(duration*duration*duration)};
  double a4{-15*ee_displacement_y/(duration*duration*duration*duration)};
  double a5{6*ee_displacement_y/(duration*duration*duration*duration*duration)};

  Eigen::Vector3d tmp_pos;

  while (ros::ok()) {
    // check if it is necessary calling callbacks
    ros::spinOnce();
    // switch on status to plan differently based on actual motion condition
    switch (status) {
      case 2: // move along the ee_displacement direction
        if (!initial_pose_init) break;
        //Trajectory computation
        //ee_trajectory_2 = ee_displacement*Eigen::RowVectorXd::LinSpaced(steps_num, 0, 1) + initial_EE_point*Eigen::RowVectorXd::Ones(steps_num);
        // trajectory publishing
        rate2.reset();
        for (std::int32_t i = 0; i < steps_num; ++i) {
          double t{(double)(steps_num)*sampling_time};
          double tmp_pos_y = initial_EE_point(1) + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
          tmp_pos << initial_EE_point(0), tmp_pos_y, initial_EE_point(2);
          actual_pose                         = tmp_pos;
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