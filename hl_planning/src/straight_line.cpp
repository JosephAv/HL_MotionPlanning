#include <eigen3/Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <string>
#include <fstream>
#include <vector>
#include <iostream>

#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#define NUM_COLS 100


std::string data_row_str;
std::vector<std::string> data_row;
Eigen::MatrixXd data_matrix;

Eigen::MatrixXd ee_trajectory(3, NUM_COLS);

Eigen::VectorXd initial_EE_point(3);
bool initial_pose_init = false;
geometry_msgs::Pose msg_pose;


// Convert xyzrpy vector to geometry_msgs Pose (PRESA DA PANDA-SOFTHAND -> TaskSequencer.cpp)
geometry_msgs::Pose convertVectorToPose(Eigen::VectorXd input_vec){
    
    // Creating temporary variables
    geometry_msgs::Pose output_pose;
    Eigen::Affine3d output_affine;

    // Getting translation and rotation
    Eigen::Vector3d translation(input_vec[0], input_vec[1], input_vec[2]);
    output_affine.translation() = translation;
    //Eigen::Matrix3d rotation = Eigen::Matrix3d(Eigen::AngleAxisd(input_vec[5], Eigen::Vector3d::UnitZ())
    //    * Eigen::AngleAxisd(input_vec[4], Eigen::Vector3d::UnitY())
    //    * Eigen::AngleAxisd(input_vec[3], Eigen::Vector3d::UnitX()));
    Eigen::Matrix3d rotation;
    rotation << 1, 0, 0, 0, -1, 0, 0, 0, -1; 
    output_affine.linear() = rotation;    
    
    // Converting to geometry_msgs and returning
    tf::poseEigenToMsg(output_affine, output_pose);  //CONTROLLARE SE METTERE #include <eigen_conversions/eigen_msg.h>
    return output_pose;
}

void robotPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    if (!initial_pose_init)
    {
        msg_pose = msg.pose;
        std::cout << "Message received" << std::endl;
        std::cout << msg << std::endl;
        Eigen::Affine3d input_affine;
        Eigen::Vector3d traslazione;
        Eigen::Vector3d rpy;
        Eigen::Matrix3d mat_rotazione;
        tf::poseMsgToEigen(msg_pose,input_affine);
        traslazione = input_affine.translation();
        mat_rotazione = input_affine.rotation();
        rpy = mat_rotazione.eulerAngles(0, 1, 2);

        
        initial_EE_point << traslazione[0], traslazione[1], traslazione[2];
        std::cout << "Starting pose read from topic:" << std::endl;
        std::cout << initial_EE_point << std::endl;

        initial_pose_init = true;
    }
    
}


int main(int argc, char **argv)
{
    //Initialize the node
    ROS_INFO("Stright Line Planner...node initialization...");
    ros::init(argc, argv, "SL_planner");
    ros::NodeHandle node;

    //Initialize frame trajectory publisher
    ROS_INFO("PUBLISHER INITIALIZATION");
    ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("/franka/equilibrium_pose", 1);

    //Initialize starting pose subscriber
    ros::Subscriber robot_pose_sub = node.subscribe("/franka_ee_pose", 1, robotPoseCallback);

    ROS_INFO("Variable definitions");
    //Starting and ending times definition
    double t_start = 0;
    double t_end;

    std::cout << "Input time to perform trajectory [s]";
    std::cin >> t_end;

    //Time axis definition
    Eigen::RowVectorXd t;
    t = Eigen::RowVectorXd::LinSpaced(NUM_COLS, 0, 1);
    
    double aux;

    //Cartesian constrains definition

    Eigen::VectorXd final_EE_point(3);

    std::cout << "Input final pose vector element by element (xyzrpy)";
    
    for (int i = 0; i < 3; i++)
    {
        std::cin >> aux;
        final_EE_point(i) = aux;
    }

    ROS_INFO("INPUT COMPLETED");


    double dt;
    dt = t_end/(NUM_COLS-1);

    while (ros::ok())
    {
        ros::spinOnce();
        if (initial_pose_init)
        {
            ROS_INFO("Trajectory Computation");
            //Trajectory computation

            ee_trajectory = (final_EE_point - initial_EE_point)*Eigen::RowVectorXd::LinSpaced(NUM_COLS, 0, 1) + initial_EE_point*Eigen::RowVectorXd::Ones(NUM_COLS);

            Eigen::VectorXd actual_pose(3);
            geometry_msgs::Pose actual_pose_msg;
            geometry_msgs::PoseStamped actual_posestamped_msg;

            std::cout << ee_trajectory.col(0)<<std::endl;

            ros::Rate rate(1/dt);

            ROS_INFO("Trajectory Publishing");
            for (int i = 0; i < NUM_COLS; i++)
            {
                actual_pose = ee_trajectory.col(i);
                std::cout << actual_pose <<std::endl;
                actual_pose_msg = convertVectorToPose(actual_pose);
                std::cout << actual_pose_msg<<std::endl;
                actual_posestamped_msg.pose = actual_pose_msg;
                pub.publish(actual_posestamped_msg);
                rate.sleep();
            }

            break;
        }
        ROS_INFO("WAITING INITIAL POSE");
        
    }
    
    ROS_INFO("TASK COMPLETED");

}