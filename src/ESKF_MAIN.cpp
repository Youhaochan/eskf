//
// Created by yhc on 23/02/2022.
//

#include "../include/ESKF/ESKF.h"
#include <ros/console.h>
#include "glog/logging.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ESKF_NODE");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p_("~");
    std::string working_space;
    // 这些参数后面都是带着默认值，如果在服务器中找不到对应的参数，就使用默认值，否则就使用服务其中的参数
    nh_p_.param<std::string>("working_space", working_space, std::string("/home/yhc/ros_ws"));
    std::string configPath;
    configPath = working_space + "/eskf_parameter.yaml";
    // LOG(IFNO)<<"config address"<<configPath;
    YAML::Node config_param;
    try
    {
        config_param = YAML::LoadFile(configPath);
    }
    catch (...)
    {
        std::cout<<"config address"<<configPath;
        ROS_ERROR("sensor config address is not correct!");
    }

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = working_space + "/log";
    FLAGS_alsologtostderr = 1;

    FLAGS_logtostderr = false; //设置日志消息是否转到标准输出而不是日志文件
    FLAGS_alsologtostderr = true; //设置日志消息除了日志文件之外是否去标准输出
    FLAGS_colorlogtostderr = true; //设置记录到标准输出的颜色消息（如果终端支持）
    FLAGS_log_prefix = true; //设置日志前缀是否应该添加到每行输出
    FLAGS_logbufsecs = 0; //设置可以缓冲日志的最大秒数，0指实时输出
    FLAGS_max_log_size = 10;                //设置最大日志文件大小（以MB为单位）
    FLAGS_stop_logging_if_full_disk = true; //设置是否在磁盘已满时避免日志记录到磁盘


    ros::Publisher odom = nh.advertise<nav_msgs::Odometry>("/odom_updated", 100);
    ESKF eskf(nh, config_param, odom);
    eskf.eskfInitialise();
    ros::Subscriber encoderSub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, &ESKF::encoderCallback, &eskf);
    ros::Subscriber ImuSub = nh.subscribe<sensor_msgs::Imu>("/imu", 1, &ESKF::imuCallback, &eskf);
    ros::Subscriber ImuSubInitialise = nh.subscribe<sensor_msgs::Imu>("/imu", 1, &ESKF::imuInitialisedCallback, &eskf);
    ros::Rate t(50);
    // while(ros::ok())
    // {
    //     eskf.showRobotPose();
    //     t.sleep();
    //     ros::spinOnce();
    // }
    ros::spin();
    google::ShutdownGoogleLogging();
    return 0;
}