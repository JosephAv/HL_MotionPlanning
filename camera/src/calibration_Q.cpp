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


int passo = 0;
int punto = 1;
int n_passi;
geometry_msgs::Pose pose_TAG;
ros::Publisher pub_T_RC;

bool get_tag_pose_{false};

Eigen::MatrixXf Q_RC_N;
Eigen::MatrixXf P_RC_N;


std::ofstream Final_pose;

void callback_EE_pose(const geometry_msgs::PoseStamped);
void callback_TAG_pose(const apriltag_ros::AprilTagDetectionArray);
Eigen::Quaternionf productOfQuaternion(Eigen::Quaternionf q1,Eigen::Quaternionf);
void rotationMatrixToRPY(float&, float&, float&, Eigen::Matrix3f);
Eigen::Matrix3f RPYtoRotationMatrix(float , float , float);

int main (int argc, char** argv) {

  ros::init (argc, argv, "calibration_Q");
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
		Q_RC_N.resize(4, n_cols);
		P_RC_N.resize(3, n_cols);

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
		Eigen::Quaternionf q_RT;
		q_RT.w() = pose_EE.orientation.w;
		q_RT.x() = pose_EE.orientation.x;
		q_RT.y() = pose_EE.orientation.y;
		q_RT.z() = pose_EE.orientation.z;

		Eigen::Quaternionf q_CT;
		q_CT.w() = pose_TAG.orientation.w;
		q_CT.x() = pose_TAG.orientation.x;
		q_CT.y() = pose_TAG.orientation.y;
		q_CT.z() = pose_TAG.orientation.z;


		Eigen::Quaternionf q_RC;
    q_RC = productOfQuaternion(q_RT,q_CT.conjugate());

    Eigen::Quaternionf P_CT_quaternion;
    P_CT_quaternion.w() = 0;
    P_CT_quaternion.x() = pose_TAG.position.x;
    P_CT_quaternion.y() = pose_TAG.position.y;
    P_CT_quaternion.z() = pose_TAG.position.z;

    P_CT_quaternion = productOfQuaternion(q_RC,P_CT_quaternion);
    P_CT_quaternion = productOfQuaternion(P_CT_quaternion,q_RC.conjugate());

    int n_cols;
    n_cols = (passo/2) -1;
    
    P_RC_N(0,n_cols) = pose_EE.position.x - P_CT_quaternion.x();
    P_RC_N(1,n_cols) = pose_EE.position.y - P_CT_quaternion.y();
    P_RC_N(2,n_cols) = pose_EE.position.z - P_CT_quaternion.z();

    Q_RC_N(0,n_cols)= q_RC.w();
    Q_RC_N(1,n_cols) = q_RC.x();
    Q_RC_N(2,n_cols) = q_RC.y();
    Q_RC_N(3,n_cols) = q_RC.z();

	}

	if (passo == n_passi*2){

		
		// media posizioni

		float xm, ym, zm, sum;

		xm = 0;
		ym = 0;
		zm = 0;
		sum = 1;

		for(int i = 0; i<n_passi; i++){

			xm = xm + (P_RC_N(0,i) - xm) / sum;
			ym = ym + (P_RC_N(1,i) - ym) / sum;
			zm = zm + (P_RC_N(2,i) - zm) / sum;
			sum = sum +1;

		}

		std::cout << "pos_matrix = "<< P_RC_N << std::endl;

		// media orientazioni

		Eigen::Quaternionf qi;
		float sin_roll, sin_pitch, sin_yaw, cos_roll, cos_pitch, cos_yaw;
		float sin_roll_m = 0;
		float sin_pitch_m = 0;
		float sin_yaw_m = 0;
		float cos_roll_m = 0;
		float cos_pitch_m = 0;
		float cos_yaw_m = 0;

		sum = 1;
		for(int i = 0; i<n_passi; i++){

			qi.w()= Q_RC_N(0,i);
			qi.x()= Q_RC_N(1,i);
			qi.y()= Q_RC_N(2,i);
			qi.z()= Q_RC_N(3,i);

			Eigen::Vector3f RPY = qi.toRotationMatrix().eulerAngles(0, 1, 2);
 		 	

			//Eigen::Vector3f  RPY;
			//rotationMatrixToRPY(RPY(0), RPY(1), RPY(2), qi.toRotationMatrix());

			sin_roll = sinf(RPY(0));
			sin_pitch = sinf(RPY(1));
			sin_yaw = sinf(RPY(2));

			cos_roll = cosf(RPY(0));
			cos_pitch = cosf(RPY(1));
			cos_yaw = cosf(RPY(2));

			sin_roll_m = sin_roll_m + (sin_roll - sin_roll_m) / sum;
			sin_pitch_m = sin_pitch_m + (sin_pitch - sin_pitch_m) / sum;
			sin_yaw_m = sin_yaw_m + (sin_yaw - sin_yaw_m) / sum;

			cos_roll_m = cos_roll_m + (cos_roll - cos_roll_m) / sum;
			cos_pitch_m = cos_pitch_m + (cos_pitch - cos_pitch_m) / sum;
			cos_yaw_m = cos_yaw_m + (cos_yaw - cos_yaw_m) / sum;

			sum = sum +1;

		}

		std::cout << "Q_matrix = "<< Q_RC_N << std::endl;

		float roll, pitch, yaw;

		roll = atan2f(sin_roll_m, cos_roll_m);
		pitch = atan2f(sin_pitch_m, cos_pitch_m);
		yaw = atan2f(sin_yaw_m, cos_yaw_m);



		//Eigen::Matrix3f R = RPYtoRotationMatrix(roll, pitch, yaw);
		//Eigen::Quaternionf Q_RC = Eigen::Quaternionf(R);

		Eigen::Quaternionf Q_RC = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());


		geometry_msgs::Pose T_RC;
		T_RC.orientation.x = Q_RC.x();
		T_RC.orientation.y = Q_RC.y();
		T_RC.orientation.z = Q_RC.z();
		T_RC.orientation.w = Q_RC.w();

		T_RC.position.x = xm;
		T_RC.position.y = ym;
		T_RC.position.z = zm;

		
		Final_pose << T_RC.position.x << "," << T_RC.position.y << "," << T_RC.position.z  << "," << T_RC.orientation.w  << "," << T_RC.orientation.x  << "," << T_RC.orientation.y << "," << T_RC.orientation.z << std::endl;
		std::cout << "FINE" << std::endl;

		system("rosnode kill calibration_Q"); 
		while(1);


	}
	else
		passo += 1;

     


  
}

Eigen::Quaternionf productOfQuaternion(Eigen::Quaternionf q1,Eigen::Quaternionf q2){

  Eigen::Quaternionf q;

  q.x() =  q1.x() * q2.w() + q1.y()  * q2.z() - q1.z() * q2.y() + q1.w() * q2.x();
    q.y() = -q1.x()  * q2.z() + q1.y() * q2.w() + q1.z() * q2.x() + q1.w()* q2.y();
    q.z() =  q1.x()  * q2.y() - q1.y() * q2.x() + q1.z() * q2.w() + q1.w() * q2.z();
    q.w() = -q1.x()  * q2.x() - q1.y() * q2.y() - q1.z() * q2.z() + q1.w() * q2.w();

    return q;

}


void rotationMatrixToRPY(float& roll, float& pitch, float& yaw, Eigen::Matrix3f R){


  yaw = atan2f(R(1,0),R(0,0));
  pitch = atan2f(-R(2,0) , sqrtf( ( R(2,1)*R(2,1) ) + ( R(2,2)*R(2,2) ) ));
  roll = atan2f(R(2,1),R(2,2));

  

}

Eigen::Matrix3f RPYtoRotationMatrix(float roll, float pitch, float yaw){


  Eigen::Matrix3f R_roll,R_pitch, R_yaw, R;

  R_roll << 1, 0, 0,
        0, cosf(roll),-sinf(roll),
        0, sinf(roll),cosf(roll);

  R_pitch << cosf(pitch), 0, sinf(pitch),
        0, 1,0,
        -sinf(pitch), 0,cosf(pitch);

  R_yaw << cosf(yaw),-sinf(yaw), 0,
       sinf(yaw),cosf(yaw) , 0,
        0, 0, 1 ;

  R = R_yaw*R_pitch*R_roll;

  return R;

}