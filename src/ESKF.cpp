//
// Created by yhc on 23/02/2022.
//

#include "../include/ESKF/ESKF.h"
#include "glog/logging.h"

void ESKF::eskfNominalStatePredict(const Eigen::Vector3d &acc_data, const Eigen::Vector3d &gyro_data)
{
    Eigen::VectorXd &p = x_t[0];
    Eigen::VectorXd &v = x_t[1];
    Eigen::VectorXd &q = x_t[2];
    Eigen::VectorXd &ab = x_t[3];
    Eigen::VectorXd &wb = x_t[4];
    Eigen::VectorXd &g = x_t[5];

    // R = quat2rotm(q);
    Eigen::Quaterniond qua(q(0), q(1), q(2), q(3));
    Eigen::Matrix3d R;
    R = qua.toRotationMatrix();
    Eigen::Vector3d temp = (R * (acc_data - ab) + g);
    // ROS_WARN("acc x: %f, y: %f, z: %f", temp(0), temp(1), temp(2));
    // if (!acc_origin.is_open())
    // {
    //     acc_origin.open("gyro_data_origin.txt", std::ios::in | std::ios::out | std::ios::trunc);
    // }
    // acc_origin << gyro_data(0) << ", " << gyro_data(1) << ", " << gyro_data(2) << std::endl;

    // if (!file.is_open())
    // {
    //     file.open("imuacc.txt", std::ios::in | std::ios::out | std::ios::trunc);
    // }
    // file << temp(0) << ", " << temp(1) << ", " << temp(2) << std::endl;
    // if (!ros::ok())
    //     file.close();

    p = p + v * delta_t + 0.5 * temp * delta_t * delta_t;
    v = v + temp * delta_t;
    Eigen::Vector3d rotvec = (gyro_data - wb) * delta_t;
    if (rotvec.norm() != 0)
    {
        // double alpha = rotvec.norm();
        // Eigen::Vector3d unit_rotvec = rotvec.normalized();
        Eigen::Quaterniond temp_q;
        temp_q = Eigen::AngleAxisd(rotvec.norm(), rotvec.normalized());
        qua = qua * temp_q;
        qua.normalize();
        q(0) = qua.w();
        q(1) = qua.x();
        q(2) = qua.y();
        q(3) = qua.z();
    }
}

void ESKF::eskfErrStatePredict(const Eigen::Vector3d &acc_data, const Eigen::Vector3d &gyro_data)
{
    Eigen::MatrixXd Fx(18, 18);
    Fx.setIdentity();
    Eigen::Matrix3d identity_matrix = Eigen::Matrix3d::Identity();
    Fx.block<3, 3>(0, 3) = identity_matrix * delta_t;
    // skeu_matrix = @(x)[0, -x(3), x(2);
    //                    x(3), 0, -x(1);
    //                    - x(2), x(1), 0];
    auto skeu_matrix = [](const Eigen::Vector3d &x) -> Eigen::Matrix3d
    {
        Eigen::Matrix3d temp;
        temp << 0, -x(2), x(1),
            x(2), 0, -x(0),
            -x(1), x(0), 0;
        return temp;
    };
    const Eigen::VectorXd &ab = x_t[3];
    const Eigen::VectorXd &wb = x_t[4];
    const Eigen::VectorXd &q = x_t[2];
    Eigen::Quaterniond qua(q(0), q(1), q(2), q(3));
    Eigen::Matrix3d R = qua.toRotationMatrix();
    Fx.block<3, 3>(3, 6) = -R * skeu_matrix(acc_data - ab) * delta_t;
    Fx.block<3, 3>(3, 9) = -R * delta_t;
    Fx.block<3, 3>(3, 15) = identity_matrix * delta_t;

    {
        Eigen::Vector3d rotvec = (gyro_data - wb) * delta_t;
        Eigen::AngleAxisd temp(rotvec.norm(), rotvec.normalized());
        Eigen::Matrix3d rotationMatrix = temp.toRotationMatrix();
        Fx.block<3, 3>(6, 6) = rotationMatrix.transpose();
    }
    Fx.block<3, 3>(6, 12) = -identity_matrix * delta_t;

    Eigen::MatrixXd Fi(18, 12);
    Fi.setZero();
    Fi.block<3, 3>(3, 0) = identity_matrix;
    Fi.block<3, 3>(6, 3) = identity_matrix;
    Fi.block<3, 3>(9, 6) = identity_matrix;
    Fi.block<3, 3>(12, 9) = identity_matrix;

    Eigen::MatrixXd Qi(12, 12);
    Qi.setZero();

    Qi.block<3, 3>(0, 0) = acc_white_noise * acc_white_noise * delta_t * delta_t * identity_matrix;
    Qi.block<3, 3>(3, 3) = gyro_white_noise * gyro_white_noise * delta_t * delta_t * identity_matrix;
    Qi.block<3, 3>(6, 6) = acc_bias_instability * acc_bias_instability * delta_t * identity_matrix;
    Qi.block<3, 3>(9, 9) = gyro_bias_instability * gyro_bias_instability * delta_t * identity_matrix;

    P = Fx * P * Fx.transpose() + Fi * Qi * Fi.transpose();
    delta_xt = Fx * delta_xt;
}

void ESKF::eskfMeasurementUpdate(const Eigen::MatrixXd &Hx, const Eigen::MatrixXd &V, const Eigen::VectorXd &y)
{

    //
    Eigen::Matrix3d identity_matrix = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd X_Deltax(19, 18);
    X_Deltax.setZero();
    X_Deltax.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    const Eigen::VectorXd &q = x_t[2];
    double qx = q(1);
    double qy = q(2);
    double qz = q(3);
    double qw = q(0);
    Eigen::MatrixXd Q_DeltaTheta(4, 3);
    Q_DeltaTheta << -qx, -qy, -qz,
        qw, -qz, qy,
        qz, qw, -qx,
        -qy, qx, qw;
    Q_DeltaTheta = Q_DeltaTheta * 0.5;

    X_Deltax.block<4, 3>(6, 6) = Q_DeltaTheta;
    X_Deltax.block<9, 9>(10, 9) = Eigen::MatrixXd::Identity(9, 9);
    Eigen::MatrixXd H = Hx * X_Deltax;
    Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + V).inverse();
    Eigen::VectorXd state_x(19);

    // // transform the prediction state in imu frame to robot frame
    // Eigen::Vector4d position_in_imu_frame, velocity_in_imu_frame;
    // position_in_imu_frame << x_t[0](0), x_t[0](1), x_t[0](2), 1;
    // velocity_in_imu_frame << x_t[1](0), x_t[1](1), x_t[1](2), 1;
    // Eigen::Vector4d position_in_robot_frame, velocity_in_robot_frame;
    // position_in_robot_frame = imu_T_robot_.inverse() * position_in_imu_frame;
    // velocity_in_robot_frame = imu_T_robot_.inverse() * velocity_in_imu_frame;

    state_x << x_t[0](0), x_t[0](1), x_t[0](2),     // position
        x_t[1](0), x_t[1](1), x_t[1](2),            // velocity
        x_t[2](0), x_t[2](1), x_t[2](2), x_t[2](3), // quaternion
        x_t[3](0), x_t[3](1), x_t[3](2),            // acc bias
        x_t[4](0), x_t[4](1), x_t[4](2),            // gyro bias
        x_t[5](0), x_t[5](1), x_t[5](2);            // g

    delta_xt = delta_xt + K * (y - Hx * state_x);

    P = (Eigen::MatrixXd::Identity(P.rows(), P.rows()) - K * H) * P;
    state_correction();
}

void ESKF::state_correction()
{

    x_t[0] = x_t[0] + delta_xt.block<3, 1>(0, 0);
    x_t[1] = x_t[1] + delta_xt.block<3, 1>(3, 0);
    x_t[3] = x_t[3] + delta_xt.block<3, 1>(9, 0);
    x_t[4] = x_t[4] + delta_xt.block<3, 1>(12, 0);
    x_t[5] = x_t[5] + delta_xt.block<3, 1>(15, 0);

    Eigen::Vector3d rotvec = delta_xt.block<3, 1>(6, 0);
    double alpha = rotvec.norm();

    if (alpha != 0)
    {
        Eigen::Vector3d unit_rotvec = rotvec.normalized();
        Eigen::Quaterniond temp_q;
        Eigen::Quaterniond qua(x_t[2](0), x_t[2](1), x_t[2](2), x_t[2](3));
        temp_q = Eigen::AngleAxisd(alpha, unit_rotvec);
        qua = qua * temp_q;
        qua.normalize();

        x_t[2](0) = qua.w();
        x_t[2](1) = qua.x();
        x_t[2](2) = qua.y();
        x_t[2](3) = qua.z();
    }
    publish_updated_odometory(encoder_msg.header);
    state_reset();
}

void ESKF::publish_updated_odometory(std_msgs::Header header)
{
    nav_msgs::Odometry odom;

    odom.header.stamp = header.stamp;
    // odom.header.frame_id = odom_frame_;
    // odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_t[0](0);
    odom.pose.pose.position.y = x_t[0](1);
    odom.pose.pose.position.z = x_t[0](2);
    odom.pose.pose.orientation.x = x_t[2](1);
    odom.pose.pose.orientation.y = x_t[2](2);
    odom.pose.pose.orientation.z = x_t[2](3);
    odom.pose.pose.orientation.w = x_t[2](0);
    odom.twist.twist.linear.x = x_t[1](0);
    odom.twist.twist.linear.y = x_t[1](1);
    odom.twist.twist.angular.z = x_t[1](2);
    odom_pub.publish(odom);
}

void ESKF::state_reset()
{
    delta_xt.setZero();
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(P.rows(), P.rows());
    P.setZero();

    P(0, 0) = 0.001;
    P(1, 1) = 0.001;
    P(2, 2) = 0.001;
    P(3, 3) = 0.001;
    P(4, 4) = 0.001;
    P(5, 5) = 0.001;
    P(6, 6) = 0.001;
    P(7, 7) = 0.001;
    P(8, 8) = 0.001;
    P(9, 9) = 0.01;
    P(10, 10) = 0.01;
    P(11, 11) = 0.05; // ab
    P(12, 12) = 0.02;
    P(13, 13) = 0.01;
    P(14, 14) = 0.001; // wb
    P(15, 15) = 0.01;
    P(16, 16) = 0.01;
    P(17, 17) = 0.05; // g
}
void ESKF::eskfInitialise()
{

    acc_white_noise = config_param["acc_white_noise"].as<double>();
    gyro_white_noise = config_param["gyro_white_noise"].as<double>();
    acc_bias_instability = config_param["acc_bias_instability"].as<double>();
    gyro_bias_instability = config_param["gyro_bias_instability"].as<double>();
    ROS_INFO("acc_white_noise: %f", acc_white_noise);
    ROS_INFO("gyro_white_noise: %f", gyro_white_noise);
    imu_qw = config_param["imu_qw"].as<double>();
    imu_qx = config_param["imu_qx"].as<double>();
    imu_qy = config_param["imu_qy"].as<double>();
    imu_qz = config_param["imu_qz"].as<double>();

    encoder_x = config_param["encoder_x"].as<double>();
    encoder_y = config_param["encoder_y"].as<double>();
    encoder_z = config_param["encoder_z"].as<double>();
    encoder_vx = config_param["encoder_vx"].as<double>();
    encoder_vy = config_param["encoder_vy"].as<double>();
    encoder_vz = config_param["encoder_vz"].as<double>();
    encoder_qw = config_param["encoder_qw"].as<double>();
    encoder_qz = config_param["encoder_qz"].as<double>();
    LOG(INFO) << "sensor noise parameter\n";
    LOG(INFO) << "acc_white_noise: " << acc_white_noise << ", gyro_white_noise: "
              << gyro_white_noise << ", acc_bias_instability: " << acc_bias_instability
              << ", gyro_bias_instability: " << gyro_bias_instability
              << ", imu_qw:" << imu_qw
              << ", imu_qx:" << imu_qx
              << ", imu_qy:" << imu_qy
              << ", imu_qz:" << imu_qz
              << ", encoder_x:" << encoder_x
              << ", encoder_y:" << encoder_y
              << ", encoder_z:" << encoder_z
              << ", encoder_vx:" << encoder_vx
              << ", encoder_vy:" << encoder_vy
              << ", encoder_vz:" << encoder_vz
              << ", encoder_qw:" << encoder_qw
              << ", encoder_qz:" << encoder_qz;
    // Eigen::Vector3d ab(0.0726, -0.0869, 0);
    try
    {
        param_acc_x_bias = config_param["acc_x_bias"].as<double>();
        param_acc_y_bias = config_param["acc_y_bias"].as<double>();
        param_acc_z_bias = config_param["acc_z_bias"].as<double>();
        param_gyro_x_bias = config_param["gyro_x_bias"].as<double>();
        param_gyro_y_bias = config_param["gyro_y_bias"].as<double>();
        param_gyro_z_bias = config_param["gyro_z_bias"].as<double>();
    }
    catch (const std::exception &e)
    {
        LOG(WARNING) << "can not find imu bias in config file, use the default values";
    }

    Eigen::Vector3d ab(param_acc_x_bias, param_acc_y_bias, param_acc_z_bias);
    Eigen::Vector3d wb(param_gyro_x_bias, param_gyro_y_bias, param_gyro_z_bias);
    Eigen::Vector3d p(0, 0, 0);
    Eigen::Vector3d v(0, 0, 0);
    Eigen::VectorXd q(4);
    q.setZero();
    // q(0) = 1;
    Eigen::Vector3d g(0, 0, -9.7803);

    x_t[0] = p;
    x_t[1] = v;
    x_t[2] = q;
    x_t[3] = ab;
    x_t[4] = wb;
    x_t[5] = g;
    delta_xt.setZero();
    P.setZero();

    P(0, 0) = 0;
    P(1, 1) = 0;
    P(2, 2) = 0;
    P(3, 3) = 0;
    P(4, 4) = 0;
    P(5, 5) = 0;
    P(6, 6) = 0;
    P(7, 7) = 0;
    P(8, 8) = 0;
    P(9, 9) = 0;
    P(10, 10) = 0.01;
    P(11, 11) = 0.05; // ab
    P(12, 12) = 0.02;
    P(13, 13) = 0.01;
    P(14, 14) = 0.001; // wb
    P(15, 15) = 0.01;
    P(16, 16) = 0.01;
    P(17, 17) = 0.05; // g
    // publish_updated_odometory();
}

void ESKF::encoderCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    eskf_mutex.lock();
    encoder_msg = *msg;
    geometry_msgs::PoseWithCovariance Pose = msg->pose;
    Eigen::Quaterniond qua;
    qua.w() = Pose.pose.orientation.w;
    qua.x() = Pose.pose.orientation.x;
    qua.y() = Pose.pose.orientation.y;
    qua.z() = Pose.pose.orientation.z;
    qua.normalize();
    pose_robot.quaternion.push_back(qua.w());
    pose_robot.quaternion.push_back(qua.x());
    pose_robot.quaternion.push_back(qua.y());
    pose_robot.quaternion.push_back(qua.z());
    pose_robot.x = Pose.pose.position.x;
    pose_robot.y = Pose.pose.position.y;

    double encoder_time = msg->header.stamp.toSec();
    // if (!file.is_open())
    // {
    //     file.open("timediff.txt", std::ios::in | std::ios::out | std::ios::trunc);
    //     std::cout << "11111\n";
    // }
    // file << fabs(ros_start - encoder_time) << std::endl;

    pose_robot.V = msg->twist.twist.linear.x;
    pose_robot.W = msg->twist.twist.angular.z;
    Eigen::VectorXd y(6);
    y << pose_robot.x, pose_robot.y, 0, pose_robot.V, 0, 0; // x,y,z vx, vy, vz
    Eigen::MatrixXd Hx(6, 19), V(6, 6);
    Hx.setZero();
    Hx.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    if (fabs(pose_robot.V) <= 0.0001 && fabs(pose_robot.W) < 0.0001)
    {
        is_robot_static = true;
    }
    else
    {
        is_robot_static = false;
        ROS_INFO("encoder data readed");
        Eigen::VectorXd encoder_noise(6);
        encoder_noise << encoder_x, encoder_y, encoder_z,
            encoder_vx, encoder_vy, encoder_vz;
        V.setZero();
        V.diagonal() = encoder_noise;
        eskfMeasurementUpdate(Hx, V, y);
    }
    showRobotPose();
    eskf_mutex.unlock();
}

void ESKF::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

    if (is_initialise_done)
    {
        Eigen::Quaterniond qua;

        qua.w() = msg->orientation.w;
        qua.x() = msg->orientation.x;
        qua.y() = msg->orientation.y;
        qua.z() = msg->orientation.z;
        qua.normalize();
        if (is_imu_first_capture)
        {
            is_imu_first_capture = false;
            start = std::chrono::system_clock::now();
            // ros_start = msg->header.stamp.toSec();
            last_acce_data_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
            last_gyro_data_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
            return;
        }

        end = std::chrono::system_clock::now();
        // ros_end = msg->header.stamp.toSec();
        // delta_t = ros_end - ros_start;
        // ros_start = msg->header.stamp.toSec();
        std::chrono::duration<double> dt = end - start;
        delta_t = dt.count();
        start = std::chrono::system_clock::now();

        // ROS_INFO("IMU data elapse time: %f", delta_t);
        // use the last imu data to predict the motion
        Eigen::Vector3d acce_data = last_acce_data_;
        Eigen::Vector3d gyro_data = last_gyro_data_;
        last_acce_data_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        last_gyro_data_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        eskf_mutex.lock();
        if (is_robot_static == false)
        {
            eskfErrStatePredict(acce_data, gyro_data);
            eskfNominalStatePredict(acce_data, gyro_data);
            // imu quaternion update
            try
            {
                bool use_imu_quaternion = config_param["use_imu_quaternion"].as<bool>();
                if (use_imu_quaternion)
                {
                    Eigen::Matrix3d matrix_measured_transformed = robot_T_imu * qua.toRotationMatrix();
                    Eigen::Quaterniond qua_measured_transformed(matrix_measured_transformed);
                    Eigen::Vector4d y(qua_measured_transformed.w(), qua_measured_transformed.x(), qua_measured_transformed.y(), qua_measured_transformed.z());
                    tf::Quaternion quat(qua_measured_transformed.x(), qua_measured_transformed.y(), qua_measured_transformed.z(), qua_measured_transformed.w());
                    Eigen::MatrixXd Hx(4, 19), V(4, 4);
                    Hx.setZero();
                    Hx.block<4, 4>(0, 6) = Eigen::MatrixXd::Identity(4, 4);

                    Eigen::VectorXd imu_qua_noise(4);
                    imu_qua_noise << imu_qw, imu_qx, imu_qy, imu_qz;
                    V.setZero();
                    V.diagonal() = imu_qua_noise;
                    eskfMeasurementUpdate(Hx, V, y);
                }
            }
            catch (...)
            {
                LOG(WARNING)<<"no parameter use_imu_quaternion";
            }

            // imu quaternion update
        }
        // else
        // {
        //     bias_estimation(acce_data, gyro_data, qua);
        // }
        publish_updated_odometory(msg->header);
        showRobotPose();
        eskf_mutex.unlock();
    }
}

void ESKF::showRobotPose()
{

    double w = x_t[2](0);
    double x = x_t[2](1);
    double y = x_t[2](2);
    double z = x_t[2](3);
    double roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    double pitch = asin(2 * (w * y - z * x));
    double yaw = atan2(2 * ((w * z + x * y)), 1 - 2 * (y * y + z * z));
    LOG(INFO) << "x: " << x_t[0](0) << ", y: " << x_t[0](1) << ", z: " << x_t[0](2) << " , yaw(deg): " << yaw * 180 / M_PI;
}

void ESKF::bias_estimation(const Eigen::Vector3d &acce_data, const Eigen::Vector3d &gyro_data, const Eigen::Quaterniond &qua_data)
{

    Eigen::Matrix3d R;
    R = qua_data.toRotationMatrix();
    Eigen::Vector3d temp = R * acce_data + x_t[5];
    acc_x_bias.push_back(temp(0));
    acc_y_bias.push_back(temp(1));
    acc_z_bias.push_back(temp(2));

    gyro_x_bias.push_back(gyro_data(0));
    gyro_y_bias.push_back(gyro_data(1));
    gyro_z_bias.push_back(gyro_data(2));

    qw.push_back(qua_data.w());
    qx.push_back(qua_data.x());
    qy.push_back(qua_data.y());
    qz.push_back(qua_data.z());
    if (acc_x_bias.size() > 20)
    {
        acc_x_bias.pop_front();
        acc_y_bias.pop_front();
        acc_z_bias.pop_front();

        gyro_x_bias.pop_front();
        gyro_y_bias.pop_front();
        gyro_z_bias.pop_front();
        qw.pop_front();
        qx.pop_front();
        qy.pop_front();
        qz.pop_front();
    }
    Eigen::Vector3d ab(0.0726, -0.0869, 0);
    Eigen::Vector3d wb(-0.001, -0.001, -0.001);
    Eigen::VectorXd q(4);

    // ab(0) = std::accumulate(acc_x_bias.begin(), acc_x_bias.end(), 0.0) / acc_x_bias.size();
    // ab(1) = std::accumulate(acc_y_bias.begin(), acc_y_bias.end(), 0.0) / acc_y_bias.size();
    // ab(2) = std::accumulate(acc_z_bias.begin(), acc_z_bias.end(), 0.0) / acc_z_bias.size();

    // wb(0) = std::accumulate(gyro_x_bias.begin(), gyro_x_bias.end(), 0.0) / gyro_x_bias.size();
    // wb(1) = std::accumulate(gyro_y_bias.begin(), gyro_y_bias.end(), 0.0) / gyro_y_bias.size();
    // wb(2) = std::accumulate(gyro_z_bias.begin(), gyro_z_bias.end(), 0.0) / gyro_z_bias.size();

    // q(0) = std::accumulate(qw.begin(), qw.end(), 0.0) / qw.size();
    // q(1) = std::accumulate(qx.begin(), qx.end(), 0.0) / qx.size();
    // q(2) = std::accumulate(qy.begin(), qy.end(), 0.0) / qy.size();
    // q(3) = std::accumulate(qz.begin(), qz.end(), 0.0) / qz.size();
    // x_t[3] = ab;
    // // x_t[2] = q;
    // x_t[4] = wb;
    // // ROS_WARN("ax: %f, ay: %f, az: %f, gx %f, gy: %f, gz: %f", ab(0), ab(1), ab(2), wb(0), wb(1), wb(2));
    // ROS_WARN("ax %f, ay: %f, az: %f", ab(0), ab(1), ab(2));
    // ROS_WARN("gx %f, gy: %f, gz: %f", wb(0), wb(1), wb(2));
    // ROS_WARN("qw: %f, qx %f, qy: %f, qz: %f", q(0), q(1), q(2), q(3));
}

void ESKF::imuInitialisedCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

    if (!is_initialise_done)
    {

        Eigen::Matrix3d R;
        Eigen::Quaterniond qua_data(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        Eigen::Vector3d acce_data(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        Eigen::Vector3d gyro_data(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        acc_x_bias.push_back(acce_data(0));
        acc_y_bias.push_back(acce_data(1));
        acc_z_bias.push_back(acce_data(2));

        gyro_x_bias.push_back(gyro_data(0));
        gyro_y_bias.push_back(gyro_data(1));
        gyro_z_bias.push_back(gyro_data(2));

        qw.push_back(msg->orientation.w);
        qx.push_back(msg->orientation.x);
        qy.push_back(msg->orientation.y);
        qz.push_back(msg->orientation.z);
        if (qw.size() > 20)
        {

            Eigen::Vector3d ab(0.0726, -0.0869, 0);
            Eigen::Vector3d wb(-0.001, -0.001, -0.001);

            Eigen::Quaterniond q;
            q.w() = std::accumulate(qw.begin(), qw.end(), 0.0) / qw.size();
            q.x() = std::accumulate(qx.begin(), qx.end(), 0.0) / qx.size();
            q.y() = std::accumulate(qy.begin(), qy.end(), 0.0) / qy.size();
            q.z() = std::accumulate(qz.begin(), qz.end(), 0.0) / qz.size();
            q.normalize();

            double w = msg->orientation.w;
            double x = msg->orientation.x;
            double y = msg->orientation.y;
            double z = msg->orientation.z;
            double roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
            double pitch = asin(2 * (w * y - z * x));
            double yaw = atan2(2 * ((w * z + x * y)), 1 - 2 * (y * y + z * z));

            LOG(INFO)<<"roll(deg): " <<roll * 180 / M_PI<<", picth: "<<pitch * 180 / M_PI<<", yaw: " <<yaw * 180 / M_PI;

            Eigen::AngleAxisd yawAngle(-yaw, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
            Eigen::Quaterniond robot_q_imu = yawAngle * pitchAngle * rollAngle;
            robot_T_imu = robot_q_imu.toRotationMatrix();

            Eigen::Matrix3d current_orientation = robot_T_imu * q.toRotationMatrix();
            Eigen::Quaterniond current_q;
            current_q = current_orientation;
            x_t[2](0) = current_q.w();
            x_t[2](1) = current_q.x();
            x_t[2](2) = current_q.y();
            x_t[2](3) = current_q.z();

            ab(0) = std::accumulate(acc_x_bias.begin(), acc_x_bias.end(), 0.0) / acc_x_bias.size();
            ab(1) = std::accumulate(acc_y_bias.begin(), acc_y_bias.end(), 0.0) / acc_y_bias.size();
            ab(2) = std::accumulate(acc_z_bias.begin(), acc_z_bias.end(), 0.0) / acc_z_bias.size();

            wb(0) = std::accumulate(gyro_x_bias.begin(), gyro_x_bias.end(), 0.0) / gyro_x_bias.size();
            wb(1) = std::accumulate(gyro_y_bias.begin(), gyro_y_bias.end(), 0.0) / gyro_y_bias.size();
            wb(2) = std::accumulate(gyro_z_bias.begin(), gyro_z_bias.end(), 0.0) / gyro_z_bias.size();
            R = current_q.toRotationMatrix();
            Eigen::Vector3d temp = R * ab + x_t[5];
            // x_t[3] = temp;
            // x_t[4] = wb;
            LOG(INFO) << "acc_bias: " << temp << ", gyro bias:" << wb << std::endl;
            qw.clear();
            qx.clear();
            qy.clear();
            qz.clear();
            is_initialise_done = true;
        }
    }
}
