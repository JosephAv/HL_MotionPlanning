#include <sstream>
  
#include <ros/ros.h>

#include <Eigen/Dense>

#include <Eigen/Geometry> 
#include <math.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include<apriltag_ros/AprilTagDetectionArray.h>

#include<fstream>
#include <Eigen/QR>
#include<Eigen/Core> 
#include<Eigen/SVD>   

template<typename _Matrix_Type_> 
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon()) 
{  
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);  
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);  
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint(); 
} 

bool get_tag_pose_{false};

int passo = 0;
int punto = 1;
int n_passi;
geometry_msgs::Pose pose_TAG;
ros::Publisher pub_T_RC;

Eigen::MatrixXf P_RT_N;
Eigen::MatrixXf P_CT_N;


std::ofstream Final_pose;

void callback_EE_pose(const geometry_msgs::PoseStamped);
void callback_TAG_pose(const apriltag_ros::AprilTagDetectionArray);

int main (int argc, char** argv) {

  ros::init (argc, argv, "calibration_pinv");
  ros::NodeHandle nh;

  Final_pose.open("/home/federico/catkin_ws/src/camera/Calibration/Pose_cam.csv");


  ros::Subscriber sub_EE_pose = nh.subscribe<geometry_msgs::PoseStamped>("/franka_ee_pose", 1, callback_EE_pose);
  ros::Subscriber sub_TAG_pose = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, callback_TAG_pose);
 
  
  get_tag_pose_ = false;

  pub_T_RC = nh.advertise<geometry_msgs::PoseStamped>("/pose_robot_cam", 1);

  ros::spin();



  return 0;
}
void callback_TAG_pose(const apriltag_ros::AprilTagDetectionArray pose_TAG_msg){
  
  if (!get_tag_pose_) return;

  pose_TAG = pose_TAG_msg.detections[0].pose.pose.pose;

  get_tag_pose_ = false;

  std::cout << "\n\nGET TAG POSE\n\n";

}

void callback_EE_pose(const geometry_msgs::PoseStamped pose_EE_msg){

  	geometry_msgs::Pose pose_EE;
	pose_EE = pose_EE_msg.pose;

	if(passo == 0){
		std::cout << "inserire numero di punti da cui calcolare la T_RC: \n";
		std::cin >> n_passi;
		int n_cols;
    n_cols = n_passi;
		P_RT_N.resize(4, n_cols);
		P_CT_N.resize(4, n_cols);

		if(n_passi < 1)
			ROS_ERROR("inserire almeno un punto");
	}
	else if(passo % 2 != 0){//se il numero di passi è dispari
  		std::cout << "Portare EE nel punto numero " << punto << " e premere invio \n";
		char a;
  		std::cin >> a;
  		punto += 1;
  		get_tag_pose_ = true;
	}
	else{
		if (get_tag_pose_) return;
		std::cout << "\n\nGET TAG POSE - DONE\n\n";

		Eigen::Vector4f pos_RT;
	    pos_RT(0) = pose_EE.position.x;
	    pos_RT(1) = pose_EE.position.y;
	    pos_RT(2) = pose_EE.position.z;
	    pos_RT(3) = 1;
	   
	   	Eigen::Vector4f pos_CT;
	    pos_CT(0) = pose_TAG.position.x;
	    pos_CT(1) = pose_TAG.position.y;
	    pos_CT(2) = pose_TAG.position.z;
	    pos_CT(3) = 1;

	

    	int n_cols;
    	n_cols = (passo/2) -1;

    	P_RT_N(0,n_cols) = pos_RT(0);
    	P_RT_N(1,n_cols) = pos_RT(1);
    	P_RT_N(2,n_cols) = pos_RT(2);
    	P_RT_N(3,n_cols) = pos_RT(3);

    	P_CT_N(0,n_cols) = pos_CT(0);
    	P_CT_N(1,n_cols) = pos_CT(1);
    	P_CT_N(2,n_cols) = pos_CT(2);
    	P_CT_N(3,n_cols) = pos_CT(3);


    	

    	//std::cout << T_RT_N << std::endl;
    	//std::cout << "\n" << std::endl;
    	//std::cout << T_CT_N << std::endl;

	}

	if (passo == n_passi*2){

		Eigen::Matrix4f T_RC_stimata;

		Eigen::MatrixXf P_CT_N_pinv;
		P_CT_N_pinv.resize(n_passi, 4);

		P_CT_N_pinv = pseudoInverse(P_CT_N);

		T_RC_stimata = P_RT_N*P_CT_N_pinv;

		//std::cout << T_RC_stimata << std::endl;
		//std::cout << "\n" << std::endl;

		//Eigen::JacobiSVD<Eigen::Matrix3f> svd(T_RC_stimata.block<3, 3>(0, 0)); 
		//T_RC_stimata.block<3, 3>(0, 0) = svd.matrixU()*svd.matrixV().transpose();

		//Eigen::Quaternionf q(T_RC_stimata.block<3, 3>(0, 0));
		//q.normalize();
		//T_RC_stimata.block<3, 3>(0, 0) = q.toRotationMatrix();

		Eigen::Quaternionf q_RC(T_RC_stimata.block<3, 3>(0, 0));
		geometry_msgs::Pose T_RC;
		T_RC.orientation.x = q_RC.x();
		T_RC.orientation.y = q_RC.y();
		T_RC.orientation.z = q_RC.z();
		T_RC.orientation.w = q_RC.w();

		T_RC.position.x = T_RC_stimata(0,3);
		T_RC.position.y = T_RC_stimata(1,3);
		T_RC.position.z = T_RC_stimata(2,3);

		std::cout << T_RC_stimata << std::endl;
		Final_pose << T_RC.position.x << "," << T_RC.position.y << "," << T_RC.position.z  << "," << T_RC.orientation.w  << "," << T_RC.orientation.x  << "," << T_RC.orientation.y << "," << T_RC.orientation.z << std::endl;
		std::cout << "FINE" << std::endl;
		
		
		system("rosnode kill calibration_pinv"); 
		while(1);


	}
	else
		passo += 1;

     


  
}

