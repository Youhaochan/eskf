//
// Created by yhc on 23/02/2022.
//

#ifndef ROS_WS_ESKF_H
#define ROS_WS_ESKF_H
#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <yaml-cpp/yaml.h>

#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>

#include <string>
#include <vector>
#include <math.h>

#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <algorithm>
#include <numeric>
#include <mutex>
struct Pose
{
    float x;
    float y;
    float theta;
    float V; // linear velocity
    float W; // angular velocity
    std::vector<double> quaternion;
};

class ESKF
{
public:
    ESKF(const ros::NodeHandle &nodeHandle, const YAML::Node &config_param, const ros::Publisher &odom_pub)
        : odom_pub(odom_pub),
          config_param(config_param),
          delta_xt(18), P(18, 18), x_t(6){};
    void eskfNominalStatePredict(const Eigen::Vector3d &acc_data, const Eigen::Vector3d &gyro_data);
    void eskfErrStatePredict(const Eigen::Vector3d &acc_data, const Eigen::Vector3d &gyro_data);
    void eskfMeasurementUpdate(const Eigen::MatrixXd &Hx, const Eigen::MatrixXd &V, const Eigen::VectorXd &y);
    void eskfInitialise();
    void state_correction();
    void state_reset();

    //
    void encoderCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void imuInitialisedCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void showRobotPose();
    void publish_updated_odometory(std_msgs::Header header);
    void bias_estimation(const Eigen::Vector3d &acce_data, const Eigen::Vector3d &gyro_data, const Eigen::Quaterniond &qua_data);
private:
    double delta_t;
    YAML::Node config_param;
    nav_msgs::Odometry encoder_msg;
    bool is_robot_static = false;
    std::mutex eskf_mutex;
    Pose pose_robot;
    ros::NodeHandle nh;
    ros::Publisher odom_pub;
    Eigen::VectorXd delta_xt;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> x_t;
    Eigen::MatrixXd P;
    Eigen::Vector3d last_acce_data_, last_gyro_data_;
    Eigen::Matrix3d robot_T_imu; //
    double last_imu_time = 0;
    bool is_imu_first_capture = true, is_initialise_done = false;
    std::vector<double> bias_data;
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
    double ros_start, ros_end;
    // file read and wriite
    std::ofstream file;
    std::ofstream acc_origin;
    std::ofstream gyrofile;
    std::ofstream quafile;

    // sensor noise
    double acc_white_noise;
    double gyro_white_noise;
    double acc_bias_instability;
    double gyro_bias_instability; 
    double param_acc_x_bias= -0.0014;
    double param_acc_y_bias= -0.0007;
    double param_acc_z_bias= 0.0026;
    double param_gyro_x_bias= -0.0002;
    double param_gyro_y_bias= -0.0026;
    double param_gyro_z_bias= -0.0009;
    double imu_qw;
    double imu_qx;
    double imu_qy;
    double imu_qz;
    double encoder_x;
    double encoder_y;
    double encoder_z;
    double encoder_vx;
    double encoder_vy;
    double encoder_vz;
    double encoder_qw;
    double encoder_qz;

    // bias estimation
    std::list<double> acc_x_bias, acc_y_bias, acc_z_bias, gyro_x_bias, gyro_y_bias, gyro_z_bias, qw, qx, qy, qz;
};

#endif // ROS_WS_ESKF_H
