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


#define NUM_ROWS 24
#define NUM_COLS 128

std::string matrix_name = "/home/mb/catkin_ws/src/hl_planning/src/matrice_pc.csv";
std::ifstream file_stream(matrix_name);
std::string data_row_str;
std::vector<std::string> data_row;
Eigen::MatrixXd data_matrix;

//SISTEMARE VARIABILI GLOBALI
Eigen::RowVectorXd t_finale(2*NUM_COLS);
Eigen::MatrixXd final_trajectory(6, 2*NUM_COLS);
double final_t_vp;

Eigen::VectorXd initial_EE_point(6);
bool initial_pose_init = false;
geometry_msgs::Pose msg_pose;

void load_fpc()
{
    if (!file_stream.is_open())
    {
        std::cout << "ERRORE APERTURA .csv\n";
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
}

Eigen::RowVectorXd single_dof(double starting_point, double ending_point, double starting_vel, double ending_vel, int i, double dt)
{
    //Starting and ending values of fPCs
    double fPC0_start = data_matrix(i*4,0);
    double fPC1_start = data_matrix(i*4+1,0);
    double fPC2_start = data_matrix(i*4+2,0);
    double fPC3_start = data_matrix(i*4+3,0);
    double fPC0_end = data_matrix(i*4,NUM_COLS-1);
    double fPC1_end = data_matrix(i*4+1,NUM_COLS-1);
    double fPC2_end = data_matrix(i*4+2,NUM_COLS-1);
    double fPC3_end = data_matrix(i*4+3,NUM_COLS-1);

    //Starting and ending values of first derivative of fPCs
    double fPC0dot_start = (data_matrix(i*4,1) - data_matrix(i*4,0))/dt;
    double fPC1dot_start = (data_matrix(i*4+1,1) - data_matrix(i*4+1,0))/dt;
    double fPC2dot_start = (data_matrix(i*4+2,1) - data_matrix(i*4+2,0))/dt;
    double fPC3dot_start = (data_matrix(i*4+3,1) - data_matrix(i*4+3,0))/dt;
    double fPC0dot_end = (data_matrix(i*4,NUM_COLS-1) - data_matrix(i*4,NUM_COLS-2))/dt;
    double fPC1dot_end = (data_matrix(i*4+1,NUM_COLS-1) - data_matrix(i*4+1,NUM_COLS-2))/dt;
    double fPC2dot_end = (data_matrix(i*4+2,NUM_COLS-1) - data_matrix(i*4+2,NUM_COLS-2))/dt;
    double fPC3dot_end = (data_matrix(i*4+3,NUM_COLS-1) - data_matrix(i*4+3,NUM_COLS-2))/dt;

    //Definition of linear system for trajectory computation
    Eigen::Vector4d b;
    b << starting_point-fPC0_start, ending_point-fPC0_end, starting_vel-fPC0dot_start, ending_vel-fPC0dot_end;

    Eigen::Matrix4d A;
    Eigen::Matrix4d A_I;
    A << 1, fPC1_start, fPC2_start, fPC3_start,
         1, fPC1_end, fPC2_end, fPC3_end,
         0, fPC1dot_start, fPC2dot_start, fPC3dot_start,
         0, fPC1dot_end, fPC2dot_end, fPC3dot_end;
    
    A_I = A.inverse();

    //fPCs' weight computation
    Eigen::Vector4d x;
    x = A_I*b;

    //Single DoF trajectory computation
    Eigen::RowVectorXd trajectory_single_dof;
    trajectory_single_dof = x(0)*Eigen::RowVectorXd::Ones(NUM_COLS) + data_matrix.row(i*4) + x(1)*data_matrix.row(i*4+1) + x(2)*data_matrix.row(i*4+2) + x(3)*data_matrix.row(i*4+3);

    return trajectory_single_dof;
}

// Convert xyzrpy vector to geometry_msgs Pose (PRESA DA PANDA-SOFTHAND -> TaskSequencer.cpp)
geometry_msgs::Pose convert_vector_to_pose(Eigen::VectorXd input_vec){
    
    // Creating temporary variables
    geometry_msgs::Pose output_pose;
    Eigen::Affine3d output_affine;

    // Getting translation and rotation
    Eigen::Vector3d translation(input_vec[0], input_vec[1], input_vec[2]);
    output_affine.translation() = translation;
    Eigen::Matrix3d rotation = Eigen::Matrix3d(Eigen::AngleAxisd(input_vec[5], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(input_vec[4], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(input_vec[3], Eigen::Vector3d::UnitX()));
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

        
        initial_EE_point << traslazione[0], traslazione[1], traslazione[2], rpy[0], rpy[1], rpy[2];
        std::cout << "Starting pose read from topic:" << std::endl;
        std::cout << initial_EE_point << std::endl;

        initial_pose_init = true;
    }
    
}

bool point_collision_check(Eigen::Vector3d point, Eigen::MatrixXd obstacles)
{
    int num_obs = obstacles.cols();
    Eigen::Vector3d diff;
    double dist;
    for(int i=0; i<num_obs; i++)
    {
        diff = point-obstacles.block(0,i,3,1); //RICONTROLLARE IL COMANDO BLOCK()
        dist = diff.norm();
        if (dist <= obstacles(3,i))
        {
            return true;
        }
        
    }
    return false;
}

bool trajectory_check_collision(Eigen::MatrixXd trajectory, Eigen::MatrixXd obstacles)
{
    int num_samples=trajectory.cols();
    for(int i=0; i < num_samples; i++)
    {
        if (point_collision_check(trajectory.col(i), obstacles))
        {
            return true;
        }   
    }
    return false;
}

double trajectory_length_computation(Eigen::MatrixXd trajectory)
{
    double l=0;
    Eigen::Vector3d diff;
    int num_sample = trajectory.cols();

    for (int i = 0; i < num_sample-1; i++)
    {
        diff = trajectory.col(i+1)-trajectory.col(i);
        l = l + diff.norm();
    }

    return l;
    
}

void obs_traj_computation(Eigen::VectorXd inizio, Eigen::VectorXd fine, Eigen::VectorXd vel_inizio, Eigen::VectorXd vel_fine, double t_iniziale, double t_fine, Eigen::MatrixXd ostacoli)
{
    Eigen::Vector3d inizio_pos;
    inizio_pos << inizio(0), inizio(1), inizio(2);
    Eigen::Vector3d fine_pos;
    fine_pos << fine(0), fine(1), fine(2);
    Eigen::Vector3d inizio_ori;
    inizio_ori << inizio(3), inizio(4), inizio(5);
    Eigen::Vector3d fine_ori;
    fine_ori << fine(3), fine(4), fine(5);
    Eigen::MatrixXd candidate_trajectory(3, 2*NUM_COLS);
    Eigen::Vector3d candidato_vp;
    Eigen::Vector3d diff_1;
    Eigen::Vector3d diff_2;
    double l_1;
    double l_2;
    double t_vp;
    Eigen::Vector3d v_vp;
    Eigen::MatrixXd traj_aux(3,NUM_COLS);
    Eigen::RowVectorXd t_aux(2*NUM_COLS);
    double l_opt = std::numeric_limits<double>::infinity();
    double l_candidate;

    for(int n=0; n<2000; n++)
    {
        Eigen::Vector3d offset;
        offset << 0.5, 0, 0.5;
        candidato_vp = 0.5*Eigen::Vector3d::Random() + offset; //SETTARE LO SPAZIO DI CAMPIONAMENTO
        diff_1 = candidato_vp- inizio_pos;
        diff_2 = fine_pos - candidato_vp;
        l_1 = diff_1.norm();
        l_2 = diff_2.norm();
        t_vp = (l_1/(l_1+l_2))*(t_fine-t_iniziale)+t_iniziale;
        v_vp = (candidato_vp-inizio_pos)/(t_vp-t_iniziale);

        for(int i=0; i<3; i++)
        {
            traj_aux.block(i,0,1,NUM_COLS) = single_dof(inizio_pos(i), candidato_vp(i), vel_inizio(i), v_vp(i), i, ((t_vp-t_iniziale)/(NUM_COLS-1)));
        }
        if (trajectory_check_collision(traj_aux, ostacoli))
        {
            break;
        }
        candidate_trajectory.block(0, 0, 3, NUM_COLS) = traj_aux;

        for(int i=0; i<3; i++)
        {
            traj_aux.block(i,0,1,NUM_COLS) = single_dof(candidato_vp(i), fine_pos(i), v_vp(i), vel_fine(i), i, ((t_fine-t_vp)/(NUM_COLS-1)));
        }
        if (trajectory_check_collision(traj_aux, ostacoli))
        {
            break;
        }
        candidate_trajectory.block(0, NUM_COLS, 3, NUM_COLS) = traj_aux;

        l_candidate = trajectory_length_computation(candidate_trajectory);

        if (l_candidate < l_opt)
        {
            final_trajectory.block(0, 0, 3, 2*NUM_COLS) = candidate_trajectory;
            l_opt = l_candidate;
            t_finale.head(NUM_COLS) = Eigen::RowVectorXd::LinSpaced(NUM_COLS, t_iniziale, t_vp);
            t_finale.tail(NUM_COLS) = Eigen::RowVectorXd::LinSpaced(NUM_COLS, t_vp, t_fine);
            final_t_vp = t_vp;
        }
        
    }
    //Calcolo orientazione
    Eigen::Vector3d v_vp_ori = (fine_ori-inizio_ori)/(t_fine-t_iniziale);
    Eigen::Vector3d vp_ori = (fine_ori+inizio_ori)/2; //DA RICONTROLLARE

    for(int i=0; i<3; i++)
    {
        final_trajectory.block(i+3,0,1,NUM_COLS) = single_dof(inizio_ori(i), vp_ori(i), vel_inizio(i+3), v_vp_ori(i), i, ((final_t_vp-t_iniziale)/(NUM_COLS-1)));
    }

    for (int i = 0; i < 3; i++)
    {
        final_trajectory.block(i+3,NUM_COLS,1,NUM_COLS) = single_dof(vp_ori(i), fine_ori(i), v_vp_ori(i), vel_fine(i+3), i, ((t_fine - final_t_vp)/(NUM_COLS-1)));
    }
    

}


int main(int argc, char **argv)
{
    //Initialize the node
    ROS_INFO("NODE INITIALIZATION");
    ros::init(argc, argv, "HL_planner");
    ros::NodeHandle node;

    //Initialize frame trajectory publisher
    ROS_INFO("PUBLISHER INITIALIZATION");
    ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("/franka/equilibrium_pose", 1); //DA SISTEMARE SIA PER IL NOME CHE PER IL TIPO DI MSG

    //Initialize starting pose subscriber
    ros::Subscriber robot_pose_sub = node.subscribe("/franka_ee_pose", 1, robotPoseCallback);
    
    //Load the fPCs
    ROS_INFO("LOAD fPCs");
    load_fpc();

    ROS_INFO("Variable definitions");
    //Starting and ending times definition
    double t_start = 0;
    double t_end;

    std::cout << "Input time to perform trajectory [s]";
    std::cin >> t_end;

    //Obstacle definition
    Eigen::Vector4d obstacles;
    obstacles << 0.3, 0, 0.2, 0.15;

    //Time axis definition
    Eigen::VectorXd t;
    t = Eigen::VectorXd::LinSpaced(NUM_COLS, t_start, t_end);

    //Velocities constrains definition
    Eigen::VectorXd initial_velocity(6);
    Eigen::VectorXd final_velocity(6);

    initial_velocity << 0, 0, 0, 0, 0, 0; //POI DA SOSTITUIRE CON QUELLA ALL'ISTANTE INIZIALE DEL ROBOT
    
    double aux;
    
    final_velocity << 0, 0, 0, 0, 0, 0;

    //Cartesian constrains definition

    Eigen::VectorXd final_EE_point(6);

    std::cout << "Input final pose vector element by element (xyzrpy)";
    
    for (int i = 0; i < 6; i++)
    {
        std::cin >> aux;
        final_EE_point(i) = aux;
    }

    ROS_INFO("INPUT COMPLETED");

    double dt;


    while (ros::ok())
    {
        ros::spinOnce();
        if (initial_pose_init)
        {
            ROS_INFO("Trajectory Computation");
            //Trajectory computation
            ros::Time begin = ros::Time::now();
            obs_traj_computation(initial_EE_point, final_EE_point, initial_velocity, final_velocity, t_start, t_end, obstacles);
            ros::Time end = ros::Time::now();

            ros::Duration difference;
            difference = end - begin;

            std::cout << "Computation Time: " << difference << std::endl;
            
            Eigen::VectorXd actual_pose;
            geometry_msgs::Pose actual_pose_msg;
            geometry_msgs::PoseStamped actual_posestamped_msg;

            dt = (final_t_vp-t_start)/(NUM_COLS-1);
            ros::Rate rate(1/dt);

            ROS_INFO("Trajectory Publishing");
            for (int i = 0; i < 2*NUM_COLS; i++)
            {
                if(i==NUM_COLS)
                {
                    dt = (t_end-final_t_vp)/(NUM_COLS-1);
                    ros::Rate rate(1/dt);
                }
                actual_pose = final_trajectory.col(i);
                actual_pose_msg = convert_vector_to_pose(actual_pose);
                actual_posestamped_msg.pose = actual_pose_msg;
                actual_posestamped_msg.header.stamp = ros::Time::now();
                if (i==0)
                {
                    std::cout << "First Frame" << std::endl;
                    std::cout << actual_pose_msg << std::endl;
                }
                pub.publish(actual_posestamped_msg);
                rate.sleep();
            }

            break;
        }
        ROS_INFO("WAITING INITIAL POSE");
        
    }
    
    ROS_INFO("TASK COMPLETED");

}