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

Eigen::VectorXd initial_EE_point(3);
bool initial_pose_init = false;

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
  ROS_INFO("Stright Line Planner...node initialization...");
  ros::init(argc, argv, "SL_planner");
  ros::NodeHandle node("~");

  //Initialize frame trajectory publisher
  ROS_INFO("PUBLISHER INITIALIZATION");
  ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("/franka/equilibrium_pose", 1);

  //Initialize starting pose subscriber
  ros::Subscriber robot_pose_sub = node.subscribe("/franka_ee_pose", 1, robotPoseCallback);

  ROS_INFO("Variable definitions");
  //Starting and ending times definition
  double t_start = 0;                // [s]
  double t_end;                      // [s]
  double ee_final_pos_x;             // [m]
  double ee_final_pos_y;             // [m]
  double force_z;                    // [N]
  Eigen::VectorXd final_EE_point(3); // [m]
  std::int32_t steps_num;            // [adimensional]
  double kPosStep{0.001};            // [m]
  double time_step;                  // [s]
  double sampling_time;              // [Hz]

  if (!node.getParam("contact_force", force_z))         ROS_ERROR("Failed to get contact_force param");
  if (!node.getParam("ee_final_pos_x", ee_final_pos_x)) ROS_ERROR("Failed to get ee_final_pos_x param");
  if (!node.getParam("ee_final_pos_y", ee_final_pos_y)) ROS_ERROR("Failed to get ee_final_pos_y param");

  if (force_z < 0) ROS_ERROR("Invalid contact force, must be > 0");

  final_EE_point(0) = ee_final_pos_x;
  final_EE_point(1) = ee_final_pos_y;
  final_EE_point(2) = 0;

  if (!node.getParam("duration", t_end)) {
    if (!node.getParam("speed", speed)) {
      ROS_ERROR("Failed to get duration and speed params. Please set at least one of them");
    } else {
      if (speed <= 0) ROS_ERROR("Invalid speed value, must be > 0");
      t_end = final_EE_point.norm() / speed;
    }
  } else {
    if (t_end <= 0) ROS_ERROR("Invalid duration value, must be > 0");
    speed = final_EE_point.norm() / t_end;
  }

  ROS_INFO("Params getted correctly");
  
  time_step     = kPosStep / speed;
  sampling_time = 1.0 / time_step;
  steps_num     = static_cast<std::int32_t>(round(final_EE_point.norm() / kPosStep));

  ros::Rate rate(sampling_time);
  Eigen::MatrixXd ee_trajectory(3,steps_num);

  while (ros::ok()) {
    // check if it is necessary calling callbacks
    ros::spinOnce();
    // if initial pose retrieved, plan!!!
    if (initial_pose_init) {
      // print out info
      ROS_INFO("Trajectory Computation");
      //Trajectory computation
      final_EE_point(2) = initial_EE_point(2); // same z-value, moves only on x-y plane
      ee_trajectory     = (final_EE_point - initial_EE_point)*Eigen::RowVectorXd::LinSpaced(steps_num, 0, 1) + initial_EE_point*Eigen::RowVectorXd::Ones(steps_num);

      Eigen::VectorXd actual_pose(3);
      geometry_msgs::Pose actual_pose_msg;
      geometry_msgs::PoseStamped actual_posestamped_msg;

      ROS_INFO("Trajectory Publishing");

      rate.reset();
      for (std::int32_t i = 0; i < steps_num; ++i) {
        actual_pose                 = ee_trajectory.col(i);
        actual_pose_msg             = convertVectorToPose(actual_pose);
        actual_posestamped_msg.pose = actual_pose_msg;
        pub.publish(actual_posestamped_msg);
        rate.sleep();
      }
      initial_pose_init = false;
    }
    ROS_INFO("WAITING INITIAL POSE");  
  }
  ROS_INFO("TASK COMPLETED");
}