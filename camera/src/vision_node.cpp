// camera with z axis parallel to the table 

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
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>

#include <pcl/common/pca.h>
#include <Eigen/Dense>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>


#include <Eigen/Geometry> 
#include <math.h>


#include<fstream>

#define PI 3.1415926535

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");


#define NUM_ROWS 1
#define NUM_COLS 7

std::string matrix_name = "/home/federico/catkin_ws/src/camera/Calibration/Pose_cam.csv";
std::ifstream file_stream(matrix_name);
std::string data_row_str;
std::vector<std::string> data_row;
Eigen::MatrixXd data_matrix;



namespace camera{

  struct Time_t {

  ros::Time time_old;
  ros::Time time_now;

  };


  struct Angles_T{

    float roll_old;
    float roll_new;
    float pitch_old;
    float pitch_new;
    float yaw_old;
    float yaw_new;
  };

}

camera::Time_t tempo;
camera::Angles_T angles;

float MIN_Z, MAX_Z, MIN_Y, MAX_Y, MIN_X, MAX_X, THETA, PSI, PHI,ALPHA, POS_CAM_X,POS_CAM_Y,POS_CAM_Z;
pcl::PointCloud<pcl::PointXYZ> centroids;
ros::Publisher pub_pcl;
ros::Publisher pub_pose;
ros::Publisher pub_twist;
ros::Publisher pub_deltaT;
ros::Publisher pub_n_points;
ros::Publisher pub_altezza;
ros::Publisher pub_larghezza;
ros::Publisher pub_box;
ros::Publisher pub_camera_link;
ros::Publisher pub_cloud2;
ros::Publisher pub_angles_and_dot;

geometry_msgs::PoseStamped pose;
geometry_msgs::PoseArray positions;
geometry_msgs::Twist velocity;
geometry_msgs::PoseArray poseArray;
geometry_msgs::Twist angles_and_dot;
float delta_T;

Eigen::Matrix4f T_0_cam;

// kalman filter linear
int times_seen;
Eigen::Matrix<float,6,6> Pk_1k;
Eigen::Matrix<float,6,6> R;
Eigen::Matrix<float,6,6> C;
Eigen::VectorXf xk_1k(6,1);
Eigen::VectorXf xkk(6,1);
Eigen::Matrix<float,6,6> Pkk;
float q;
int reset;
// kalman filter angular
Eigen::Matrix<float,6,6> P2k_1k;
Eigen::Matrix<float,6,6> R2;
Eigen::Matrix<float,6,6> C2;
Eigen::VectorXf wk_1k(6,1);
Eigen::VectorXf wkk(6,1);
Eigen::Matrix<float,6,6> P2kk;
float q2;



void callback(const sensor_msgs::PointCloud2ConstPtr&);
void updatePosition (geometry_msgs::PoseStamped);
void updateAngles (float , float , float);
void initializationPositions();
void computeVelocity();
void computeAngularVelocity(Eigen::Quaternionf);
void computeAngularVelocity2();
void computeDeltaT();
Eigen::Quaternionf productOfQuaternion(Eigen::Quaternionf,Eigen::Quaternionf);
void Kalman_filter();
void resetKalman();
void rotationMatrixToRPY(float&, float&, float&, Eigen::Matrix3f);
Eigen::Matrix3f RPYtoRotationMatrix(float , float , float);
void initFrameCamera();
void dimension(pcl::PointCloud<pcl::PointXYZRGB> ,Eigen::Matrix3f, float&, float&, float& );
void box(pcl::PointCloud<pcl::PointXYZRGB> ,Eigen::Matrix3f, float, float, float );
void Kalman_filter_orientation(float, float, float);
void resetKalman_orientation(float , float, float);



int main (int argc, char** argv) {

  ros::init (argc, argv, "vision_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(15);

  nh.getParam("/vision_node/max_distance_z", MAX_Z);
  nh.getParam("/vision_node/min_distance_z", MIN_Z);
  nh.getParam("/vision_node/max_distance_y", MAX_Y);
  nh.getParam("/vision_node/min_distance_y", MIN_Y);
  nh.getParam("/vision_node/max_distance_x", MAX_X);
  nh.getParam("/vision_node/min_distance_x", MIN_X);
  nh.getParam("/vision_node/pos_camera_x", POS_CAM_X);
  nh.getParam("/vision_node/pos_camera_y", POS_CAM_Y);
  nh.getParam("/vision_node/pos_camera_z", POS_CAM_Z);
  nh.getParam("/vision_node/roll_camera", PHI);
  nh.getParam("/vision_node/pitch_camera", THETA);
  nh.getParam("/vision_node/yaw_camera", PSI);
  nh.getParam("/vision_node/inclination_camera", ALPHA);

  
  initFrameCamera();
  initializationPositions();


  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, callback);

  pub_pose = nh.advertise<geometry_msgs::PoseArray>("/object/poseArray", 1);
  pub_twist = nh.advertise<geometry_msgs::Twist>("/object/Twist", 1);
  pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("object/pcl", 1);
  pub_n_points = nh.advertise<std_msgs::UInt32>("objects/n_points", 1);
  pub_deltaT = nh.advertise<std_msgs::Float32>("/vision_node/deltaT", 1);

  pub_altezza = nh.advertise<std_msgs::Float32>("/object/altezza", 1);
  pub_larghezza = nh.advertise<std_msgs::Float32>("/object/larghezza", 1);

  pub_box = nh.advertise<sensor_msgs::PointCloud2>("/object/box", 1);
  pub_cloud2 = nh.advertise<sensor_msgs::PointCloud2>("/cloud2", 1);

  pub_camera_link = nh.advertise<geometry_msgs::PoseStamped>("/camera_link", 1);
  pub_angles_and_dot = nh.advertise<geometry_msgs::Twist>("/object/angles_and_dot", 1);

  ros::spin();

  return 0;
}


void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {

  geometry_msgs::PoseStamped camera_link;
  camera_link.pose.position.x = 0;
  camera_link.pose.position.y = 0;
  camera_link.pose.position.z = 0;
  camera_link.pose.orientation.w = 1;
  camera_link.pose.orientation.x = 0;
  camera_link.pose.orientation.y = 0;
  camera_link.pose.orientation.z = 0;

  camera_link.header.frame_id = "/panda_link0";
  camera_link.header.stamp = ros::Time();
  pub_camera_link.publish(camera_link);
  
  computeDeltaT();
  pub_deltaT.publish (delta_T);
  
  // sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2
  pcl::PCLPointCloud2* cloud_ = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_);
  pcl_conversions::toPCL(*cloud_msg, *cloud_);

  cloud_->header.frame_id = "/franka_link0";
  
  // pcl::pointcloud2 -> pcl::pointxyzrgb
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(*cloud_, *cloud);
  
  // point cloud in the word frame 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud (*cloud, *cloud_filtered, T_0_cam); 

  
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

  // filter on y (table, y positive downward)
  pcl::PassThrough<pcl::PointXYZRGB> pass_x;
  pass_x.setInputCloud (cloud_filtered);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (MIN_X, MAX_X);
  //pass_y.setFilterLimitsNegative (true);
  pass_x.filter (*cloud_filtered);

  // filter color
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    short int r,b,g,intensity;

  for (int i = 0; i < (*cloud_filtered).size(); i++){

    r = cloud_filtered->points[i].r;
    b = cloud_filtered->points[i].b;
    g = cloud_filtered->points[i].g;
    intensity = (short int)((float)g+(float)r+(float)g)/3;

      if (intensity > 25){
      //if (r > 200 && g > 160 && b < 150){

          inliers->indices.push_back(i);
      }
    }

  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  //extract.setNegative(true);
  extract.filter(*cloud_filtered);

  // point cloud downsampling
  pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloud2Ptr(cloud2);
  
  pcl::toPCLPointCloud2(*cloud_filtered, *cloud2);
  pcl::PCLPointCloud2 cloud2_filtered;
  
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud2Ptr);
  sor.setLeafSize (0.01f, 0.010f, 0.010f);
  sor.filter (cloud2_filtered);
  pcl::fromPCLPointCloud2(cloud2_filtered, *cloud_filtered);


  //pcl::PointCloud<pcl::PointXYZRGB> copia;
  //copia = *cloud_filtered;

 // pcl::PCLPointCloud2 cloud22;
 // pcl::toPCLPointCloud2(*cloud2_filtered, cloud22);
  

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output2;
  
  pcl_conversions::moveFromPCL(cloud22, output2);
  output2.header.frame_id = "/panda_link0";
  // Publish the data
  pub_pcl.publish (output2);



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

    //std::vector<uint32_t> number_points;
    uint32_t number_points;
    uint32_t number_points_min = 10000;
    pcl::PointCloud<pcl::PointXYZRGB> coord;

  
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      
      for (const auto& idx : it->indices)
        cloud_cluster->push_back ((*cloud_filtered)[idx]);
        
      cloud_cluster->width = cloud_cluster->size ();  
      cloud_cluster->height = 1;        
      cloud_cluster->is_dense = true;     
    
      //number_points.push_back(cloud_cluster->size ());
      number_points = cloud_cluster->size ();

      if (number_points < number_points_min){
        coord = *cloud_cluster;
        number_points_min = number_points;
      }
    
      number_object++;
    }

    Eigen::Matrix3f covariance_matrix;
    Eigen::Vector4f xyz_centroid;

    // Estimate the XYZ centroid
    compute3DCentroid (coord, xyz_centroid);
    
    // Compute the 3x3 covariance matrix
    computeCovarianceMatrixNormalized (coord, xyz_centroid, covariance_matrix);
      

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covariance_matrix);
    if (eigensolver.info() != Eigen::Success){
      ROS_ERROR("error eigensolver");
    }

    Eigen::Matrix3f eigenvector_matrix = eigensolver.eigenvectors();

    eigenvector_matrix.col(2) = eigenvector_matrix.col(0).cross(eigenvector_matrix.col(1)); //correct vertical between main directions
    eigenvector_matrix.col(0) = eigenvector_matrix.col(1).cross(eigenvector_matrix.col(2));
    eigenvector_matrix.col(1) = eigenvector_matrix.col(2).cross(eigenvector_matrix.col(0)); 

    Eigen::Quaternionf q(eigenvector_matrix.transpose());

  //  std::cout << "quaternion: " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " " << std::endl;


/*
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
*/
    Eigen::Quaternionf qf(eigenvector_matrix.transpose());

    float roll, pitch, yaw;
    rotationMatrixToRPY(roll, pitch, yaw, eigenvector_matrix.transpose());
    //Eigen::Matrix3f R2 = RPYtoRotationMatrix(roll, pitch, yaw);
    //Eigen::Quaternionf q2(R2);
    updateAngles(roll, pitch, yaw);

    angles_and_dot.linear.x = roll;
    angles_and_dot.linear.y = pitch;
    angles_and_dot.linear.z = yaw;


    pose.pose.position.x = xyz_centroid(0);
    pose.pose.position.y = xyz_centroid(1);
    pose.pose.position.z = xyz_centroid(2);

    pose.pose.orientation.w = qf.w(); 
    pose.pose.orientation.x = qf.x(); 
    pose.pose.orientation.y = qf.y(); 
    pose.pose.orientation.z = qf.z();

    updatePosition(pose);
    computeVelocity();
    //computeAngularVelocity(qf);
    computeAngularVelocity2();

    angles_and_dot.angular.x = velocity.angular.x;
    angles_and_dot.angular.y = velocity.angular.y;
    angles_and_dot.angular.z = velocity.angular.z;

    pub_angles_and_dot.publish(angles_and_dot);
    
    Kalman_filter();
    positions.poses[1].position.x = xkk[0];
    positions.poses[1].position.y = xkk[1];
    positions.poses[1].position.z = xkk[2];
    velocity.linear.x = xkk[3];
    velocity.linear.y = xkk[4];
    velocity.linear.z = xkk[5];

/*
    Kalman_filter_orientation(roll,pitch,yaw);
    Eigen::Matrix3f R_f = RPYtoRotationMatrix(wkk[0], wkk[1], wkk[2]);
    Eigen::Quaternionf q_f(R_f);
    positions.poses[1].orientation.w = q_f.w();
    positions.poses[1].orientation.x = q_f.x();
    positions.poses[1].orientation.y = q_f.y();
    positions.poses[1].orientation.z = q_f.z();
    velocity.angular.x = wkk[3];
    velocity.angular.y = wkk[4];
    velocity.angular.z = wkk[5];

*/

  
    
    pose.header.frame_id = "/panda_link0";
    pose.header.stamp = ros::Time();

    if(times_seen < 3)
      times_seen = times_seen + 1;

    //pose.pose.position.x = xyz_centroid(2);
    //pose.pose.position.y = -xyz_centroid(0);
    //pose.pose.position.z = -xyz_centroid(1);
/*
    pose.pose.orientation.w = qf.w(); 
    pose.pose.orientation.x = -qf.z(); 
    pose.pose.orientation.y = qf.x(); 
    pose.pose.orientation.z = qf.y();
    
    pose.pose.position.x = positions.poses[1].position.z;
    pose.pose.position.y = -positions.poses[1].position.x;
    pose.pose.position.z = -positions.poses[1].position.y;
*/
    pose.pose.orientation.w = qf.w(); 
    pose.pose.orientation.x = qf.x(); 
    pose.pose.orientation.y = qf.y(); 
    pose.pose.orientation.z = qf.z();
    
    pose.pose.position.x = positions.poses[1].position.x;
    pose.pose.position.y = positions.poses[1].position.y;
    pose.pose.position.z = positions.poses[1].position.z;

    poseArray.poses.push_back(pose.pose);
    
    pub_pose.publish(poseArray);
    pub_twist.publish(velocity);
    pub_n_points.publish(number_points);

    //ROS_INFO("mando = %f, %f, %f",pose.pose.position.x,pose.pose.position.y, pose.pose.position.z);


    float altezza, larghezza, spessore;
    //dimension (coord, eigenvector_matrix.transpose(), altezza, larghezza, spessore);
    //box (coord, eigenvector_matrix.transpose(), altezza, larghezza, spessore);

    //std::cout << altezza << " " << larghezza << " " << spessore << "\n" << std::endl;

    //pub_altezza.publish(altezza);
    //pub_larghezza.publish(larghezza);
    
   
}
  
  else
   times_seen = 0;

 

} 


void updatePosition (geometry_msgs::PoseStamped newPose){

  geometry_msgs::PoseStamped oldPose;
  
  oldPose.pose = positions.poses[0];
  positions.poses[0] = positions.poses[1];
  positions.poses[1] = newPose.pose;

}

void updateAngles (float roll, float pitch, float yaw ){

  angles.roll_old = angles.roll_new;
  angles.pitch_old = angles.pitch_new;
  angles.yaw_old = angles.yaw_new;
  
  angles.roll_new = roll;
  angles.pitch_new = pitch;
  angles.yaw_new = yaw;

}

void initializationPositions(){

  geometry_msgs::Pose init;

  times_seen = 0;

  init.position.x = -1;
  init.position.y = -1;
  init.position.z = -1;
  init.orientation.w = 1;
  init.orientation.x = 0;
  init.orientation.y = 0;
  init.orientation.z = 0;

  angles.roll_old = 20;
  angles.pitch_old = 20;
  angles.yaw_old = 20;
  angles.roll_new = 20;
  angles.pitch_new = 20;
  angles.yaw_new = 20;

  positions.poses.push_back(init);
  positions.poses.push_back(init);
  positions.header.frame_id = "/panda_link0";
  positions.header.stamp = ros::Time();

  poseArray.header.frame_id = "/panda_link0";
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
      
    velocity.linear.x = (positions.poses[1].position.x - positions.poses[0].position.x)/delta_T;
    velocity.linear.y = (positions.poses[1].position.y - positions.poses[0].position.y)/delta_T;
    velocity.linear.z = (positions.poses[1].position.z - positions.poses[0].position.z)/delta_T;
    //ROS_INFO("si muove con v = (%f,%f,%f)",velocity.linear.x, velocity.linear.y, velocity.linear.z);
    
    }

}

void computeAngularVelocity(Eigen::Quaternionf qf){


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

}


void computeAngularVelocity2(){

  if (angles.roll_old != 20){
      
    velocity.angular.x = (angles.roll_new - angles.roll_old)/delta_T;
    velocity.angular.y = (angles.pitch_new - angles.pitch_old)/delta_T;
    velocity.angular.z = (angles.yaw_new - angles.yaw_old)/delta_T;
    //std::cout << velocity.angular.x << velocity.angular.y << velocity.angular.z << std::endl;

    }

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

void Kalman_filter(){   // k_1k = k+1 dato k, kk_1 = k dato k-1

  if (times_seen == 2){//non c'è un oggetto -> reset filter

    resetKalman();
    reset = 1;

  }
  else if(times_seen == 3){

    reset = 0;
    Eigen::Matrix<float,6,6> A = Eigen::Matrix<float,6,6>::Identity();
    A.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity()*delta_T;



    // predizione
    xk_1k = A*xkk;

    Eigen::VectorXf D(6,1);
    D << xk_1k(3),
          xk_1k(4), 
          xk_1k(5), 
          xk_1k(3),
          xk_1k(4), 
          xk_1k(5);
  
    Pk_1k = A*Pkk*A.transpose() + D*q*D.transpose();
    

    Eigen::VectorXf y_k(6,1);
    y_k << positions.poses[1].position.x,
            positions.poses[1].position.y,
            positions.poses[1].position.z,
            velocity.linear.x,
            velocity.linear.y,
            velocity.linear.z;

    Eigen::VectorXf xkk_1(6,1);
    Eigen::Matrix<float,6,6> Pkk_1;

    // correzione
    xkk_1 = xk_1k;
    Pkk_1 = Pk_1k;
    
    Eigen::VectorXf e_k(6,1);
    e_k = y_k - C*xkk_1;

    Eigen::Matrix<float,6,6> S_k;
    S_k = R + C*Pkk_1*C.transpose();

    Eigen::Matrix<float,6,6> L_k;
    L_k = Pkk_1*C.transpose()*S_k.inverse();
    
    xkk = xkk_1 + L_k*e_k;
    Pkk = (Eigen::Matrix<float,6,6>::Identity() - L_k*C)*Pkk_1*((Eigen::Matrix<float,6,6>::Identity() - L_k*C).transpose()) + L_k*R*L_k.transpose();
    
  

  }

}

void resetKalman(){


  R << 0.000343680526444, 0, 0, 0, 0, 0,
        0, 0.000395054969864, 0, 0, 0, 0,
        0, 0, 0.000221925837272, 0, 0, 0,
        0, 0, 0, 0.076750078219258, 0, 0,
        0, 0, 0, 0, 0.142189089274859, 0,
        0, 0, 0, 0, 0, 0.160573690619297;
  R = 0.001*R;


  C = Eigen::Matrix<float,6,6>::Identity();

  Pkk = R;
  xkk << positions.poses[1].position.x,
            positions.poses[1].position.y,
            positions.poses[1].position.z,
            velocity.linear.x,
            velocity.linear.y,
            velocity.linear.z;

  q = 4.65038623432314*0.0001;
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

void initFrameCamera(){
/*
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
*/

  if (!file_stream.is_open())
    {
        std::cout << "ERRORE APERTURA Pose_cam.csv\n";
    }

    data_matrix.resize(NUM_ROWS, NUM_COLS);

    for (int i = 0; i < NUM_ROWS; ++i)
    {
        std::getline(file_stream, data_row_str);
        boost::algorithm::split(data_row, data_row_str, boost::is_any_of(","), boost::token_compress_on);
        for (int j = 0; j < NUM_COLS; ++j)
        {
            data_matrix(i,j) = std::stod(data_row[j]);
        }

    }
    //std::cout << data_matrix << std::endl;
    Eigen::Quaternionf q_cam;
    Eigen::Vector3f pos_cam;
    pos_cam(0) = data_matrix(0);
    pos_cam(1) = data_matrix(1);
    pos_cam(2) = data_matrix(2);
    //pos_cam(0) = 0;
    //pos_cam(1) = 0;
    //pos_cam(2) = 0;
    q_cam.w() = data_matrix(3);
    q_cam.x() = data_matrix(4);
    q_cam.y() = data_matrix(5);
    q_cam.z() = data_matrix(6);

    Eigen::Matrix3f R = q_cam.toRotationMatrix();

    T_0_cam = Eigen::Matrix4f::Identity();
    T_0_cam.block<3, 3>(0, 0) = R;
    T_0_cam.block<3, 1>(0, 3) = pos_cam.transpose();

}

void dimension(pcl::PointCloud<pcl::PointXYZRGB> cloud,Eigen::Matrix3f R, float& altezza, float& larghezza, float& spessore){


  Eigen::Matrix3f R_roll,R_pitch,R2;
  Eigen::Matrix4f T2;

  R_roll << 1, 0, 0,
        0, cosf((90)*PI/180),-sinf((90)*PI/180),
        0, sinf((90)*PI/180),cosf((90)*PI/180);

  R_pitch << cosf(-90*PI/180), 0, sinf(-90*PI/180),
        0, 1,0,
        -sinf(-90*PI/180), 0,cosf(-90*PI/180);

  R2 = R_pitch*R_roll;

  T2 = Eigen::Matrix4f::Identity();
  T2.block<3, 3>(0, 0) = R2;

  //pcl::transformPointCloud (cloud, cloud, T2.inverse()); 

  pcl::PointCloud<pcl::PointXYZRGB> cloud2;

  float centroid_x, centroid_y, centroid_z;
  //centroid_x = -positions.poses[1].position.y;
  //centroid_y = -positions.poses[1].position.z;
  //centroid_z = positions.poses[1].position.x;
  centroid_x = positions.poses[1].position.x;
  centroid_y = positions.poses[1].position.y;
  centroid_z = positions.poses[1].position.z;

  Eigen::Vector3f pos;
  pos = {-centroid_x,-centroid_y,-centroid_z};
  Eigen::Matrix4f T;
  T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
  T.block<3, 1>(0, 3) = pos.transpose();

  pcl::transformPointCloud (cloud, cloud, T); 


  pos = {0,0,0};
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = pos.transpose();

  pcl::transformPointCloud (cloud, cloud, T); 

  pos = {centroid_x,centroid_y,centroid_z};
  T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
  T.block<3, 1>(0, 3) = pos.transpose();

  pcl::transformPointCloud (cloud, cloud, T); 
  //pcl::transformPointCloud (cloud, cloud, T2.inverse()); 
/*
  pcl::PCLPointCloud2 cloud3;
  pcl::toPCLPointCloud2(cloud, cloud3);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud3, output);
  output.header.frame_id = "camera_link";
  pub_box.publish(output);
*/
  float sinistra_x = 10;
  float destra_x = -10;
  float basso_y = -10;
  float alto_y = 10;
  float vicino_z = 10;
  float lontano_z = -10;
 

  for (int i=0; i < cloud.points.size(); i++){
    
    if(cloud.points[i].y > basso_y)
        basso_y = cloud.points[i].y;

    if(cloud.points[i].y < alto_y)
        alto_y = cloud.points[i].y;

    if(cloud.points[i].x > destra_x)
        destra_x = cloud.points[i].x;

    if(cloud.points[i].x < sinistra_x)
        sinistra_x = cloud.points[i].x;

    if(cloud.points[i].z < vicino_z)
        vicino_z = cloud.points[i].z;

    if(cloud.points[i].z > lontano_z)
        lontano_z = cloud.points[i].z;

  }


  altezza = basso_y - alto_y;
  larghezza = sinistra_x - destra_x;
  spessore = lontano_z - vicino_z;

  if(altezza < 0)
    altezza = -altezza;

  if(larghezza < 0)
    larghezza = -larghezza;

  if(spessore < 0)
    spessore = -spessore;
}

void box(pcl::PointCloud<pcl::PointXYZRGB> cloud,Eigen::Matrix3f R, float altezza, float larghezza, float spessore){

  pcl::PCLPointCloud2 cloud2;
  pcl::toPCLPointCloud2(cloud, cloud2);  
  sensor_msgs::PointCloud2 output2;
  pcl_conversions::moveFromPCL(cloud2, output2);
  output2.header.frame_id = "/panda_link0";
  output2.header.stamp = ros::Time();
  pub_cloud2.publish(output2);

  float centroid_x, centroid_y, centroid_z;
  centroid_x = positions.poses[1].position.x;
  centroid_y = positions.poses[1].position.y;
  centroid_z = positions.poses[1].position.z;
  //centroid_x = -positions.poses[1].position.y;
  //centroid_y = -positions.poses[1].position.z;
  //centroid_z = positions.poses[1].position.x;

  pcl::PointXYZ spigolo_basso_destra;
  spigolo_basso_destra.x =  larghezza/2;
  spigolo_basso_destra.y =  altezza/2;
  spigolo_basso_destra.z = - spessore/2;

  pcl::PointXYZ spigolo_basso_sinistra;
  spigolo_basso_sinistra.x = - larghezza/2;
  spigolo_basso_sinistra.y =  altezza/2;
  spigolo_basso_sinistra.z =  - spessore/2;

  pcl::PointXYZ spigolo_alto_sinistra;
  spigolo_alto_sinistra.x = - larghezza/2;
  spigolo_alto_sinistra.y =  - altezza/2;
  spigolo_alto_sinistra.z =  - spessore/2;

  pcl::PointXYZ spigolo_alto_destra;
  spigolo_alto_destra.x =  + larghezza/2;
  spigolo_alto_destra.y =  - altezza/2;
  spigolo_alto_destra.z =  - spessore/2;

  pcl::PointXYZ spigolo_alto_destra_back;
  spigolo_alto_destra_back.x =  + larghezza/2;
  spigolo_alto_destra_back.y =  - altezza/2;
  spigolo_alto_destra_back.z =   spessore/2;

  pcl::PointXYZ spigolo_alto_sinistra_back;
  spigolo_alto_sinistra_back.x = - larghezza/2;
  spigolo_alto_sinistra_back.y =  - altezza/2;
  spigolo_alto_sinistra_back.z =  spessore/2;

  pcl::PointXYZ spigolo_basso_sinistra_back;
  spigolo_basso_sinistra_back.x =  - larghezza/2;
  spigolo_basso_sinistra_back.y =   altezza/2;
  spigolo_basso_sinistra_back.z =   spessore/2;

  pcl::PointXYZ spigolo_basso_destra_back;
  spigolo_basso_destra_back.x = larghezza/2;
  spigolo_basso_destra_back.y =  altezza/2;
  spigolo_basso_destra_back.z = spessore/2;

  pcl::PointCloud<pcl::PointXYZ> box;

  box.width = 8;
  box.height   = 1;
  box.is_dense = false;
  box.resize (box.width * box.height);
     
  box.points[0] = spigolo_basso_sinistra;
  box.points[1] = spigolo_basso_destra;
  box.points[2] = spigolo_alto_destra;
  box.points[3] = spigolo_alto_sinistra;
  box.points[4] = spigolo_basso_sinistra_back;
  box.points[5] = spigolo_basso_destra_back;
  box.points[6] = spigolo_alto_destra_back;
  box.points[7] = spigolo_alto_sinistra_back;


  Eigen::Vector3f pos;
  pos = {0,0,0};
  Eigen::Matrix4f T;
  T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = pos.transpose();

  pcl::transformPointCloud (box, box, T.inverse()); 
 //per una giusta visualizzazione
  Eigen::Matrix3f R_roll,R_pitch,R2;
  Eigen::Matrix4f T2;

  R_roll << 1, 0, 0,
        0, cosf((90)*PI/180),-sinf((90)*PI/180),
        0, sinf((90)*PI/180),cosf((90)*PI/180);

  R_pitch << cosf(-90*PI/180), 0, sinf(-90*PI/180),
        0, 1,0,
        -sinf(-90*PI/180), 0,cosf(-90*PI/180);

  R2 = R_pitch*R_roll;

  T2 = Eigen::Matrix4f::Identity();
  T2.block<3, 3>(0, 0) = R2;

  pcl::transformPointCloud (box, box, T2.inverse()); 

  

  pos = {centroid_z,-centroid_x,-centroid_y};
  T.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
  T.block<3, 1>(0, 3) = pos.transpose();
  pcl::transformPointCloud (box, box, T); 

  pcl::PCLPointCloud2 box3;
  pcl::toPCLPointCloud2(box, box3);  
  
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(box3, output);
  output.header.frame_id = "/panda_link0";
  output.header.stamp = ros::Time();
  pub_box.publish(output);


}


void Kalman_filter_orientation(float roll, float pitch, float yaw){   // k_1k = k+1 dato k, kk_1 = k dato k-1

  if (times_seen == 2){//non c'è un oggetto -> reset filter

    resetKalman_orientation(roll,pitch,yaw);

  }
  else if(times_seen == 3){

  
    Eigen::Matrix<float,6,6> A = Eigen::Matrix<float,6,6>::Identity();
    A.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity()*delta_T;



    // predizione
    wk_1k = A*wkk;

    Eigen::VectorXf D(6,1);
    D << xk_1k(3),
          xk_1k(4), 
          xk_1k(5), 
          xk_1k(3),
          xk_1k(4), 
          xk_1k(5);
  
    P2k_1k = A*P2kk*A.transpose() + D*q2*D.transpose();
    

    Eigen::VectorXf y_k(6,1);
    y_k << roll,
            pitch,
            yaw,
            velocity.angular.x,
            velocity.angular.y,
            velocity.angular.z;

    Eigen::VectorXf wkk_1(6,1);
    Eigen::Matrix<float,6,6> P2kk_1;

    // correzione
    wkk_1 = wk_1k;
    P2kk_1 = P2k_1k;
    
    Eigen::VectorXf e_k(6,1);
    e_k = y_k - C2*wkk_1;

    Eigen::Matrix<float,6,6> S_k;
    S_k = R2 + C2*P2kk_1*C2.transpose();

    Eigen::Matrix<float,6,6> L_k;
    L_k = P2kk_1*C2.transpose()*S_k.inverse();
    
    wkk = wkk_1 + L_k*e_k;
    P2kk = (Eigen::Matrix<float,6,6>::Identity() - L_k*C2)*P2kk_1*((Eigen::Matrix<float,6,6>::Identity() - L_k*C2).transpose()) + L_k*R2*L_k.transpose();
    
  

  }

}

void resetKalman_orientation(float roll, float pitch, float yaw){


  R2 << 0.000490116893648, 0, 0, 0, 0, 0,
        0, 0.000080841694352, 0, 0, 0, 0,
        0, 0, 0.002275261760142, 0, 0, 0,
        0, 0, 0, 0.254865766733068, 0, 0,
        0, 0, 0, 0, 0.041844576892309, 0,
        0, 0, 0, 0, 0, 1.207790068851138;


  C2 = Eigen::Matrix<float,6,6>::Identity();

  P2kk = R2;
  wkk << roll,
          pitch,
          yaw,
          velocity.angular.x,
          velocity.angular.y,
          velocity.angular.z;

  q2 = 8.085896210624806*0.00001;
}