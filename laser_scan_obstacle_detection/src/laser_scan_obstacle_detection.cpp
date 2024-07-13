// src/laser_scan_obstacle_detection.cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <string>
using namespace std;

ros::Publisher cmd_vel_robot_pub;
geometry_msgs::Twist current_cmd_vel;
double stop_distance;
string pub_topic_name;
double front_min_distance = std::numeric_limits<double>::infinity();
double behind_min_distance = std::numeric_limits<double>::infinity();


void ActionControl()
{
    geometry_msgs::Twist cmd_vel_robot;
    if (front_min_distance <= stop_distance || behind_min_distance <= stop_distance)
    {
        cmd_vel_robot.linear.x = 0.0;
        cmd_vel_robot.linear.y = 0.0;
        cmd_vel_robot.linear.z = 0.0;
        cmd_vel_robot.angular.x = 0.0;
        cmd_vel_robot.angular.y = 0.0;
        cmd_vel_robot.angular.z = 0.0;
    }
    else
    {
        cmd_vel_robot = current_cmd_vel;
    }
    cmd_vel_robot_pub.publish(cmd_vel_robot);
}

void FrontScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // float min_distance = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        if (scan->ranges[i] < front_min_distance)
        {
            front_min_distance = scan->ranges[i];
        }
    }
    // ROS_INFO("Closest obstacle distance: %f", min_distance);
}

void BehindScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // float min_distance = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        if (scan->ranges[i] < behind_min_distance)
        {
            behind_min_distance = scan->ranges[i];
        }
    }
    // ROS_INFO("Closest obstacle distance: %f", min_distance);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    current_cmd_vel = *cmd_vel;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan_obstacle_detection");
    ros::NodeHandle nh, private_nh("~");

    private_nh.param<double>("stop_distance", stop_distance, 0.5);
    private_nh.param<string>("publisher_topic_name", pub_topic_name, "cmd_vel_robot");

    ros::Subscriber front_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("front_scan_filtered", 1000, FrontScanCallback);
    ros::Subscriber behind_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("behind_scan_filtered", 1000, BehindScanCallback);
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, cmdVelCallback);
    cmd_vel_robot_pub = nh.advertise<geometry_msgs::Twist>(pub_topic_name, 1000);
    
    while(ros::ok())
    {
        ros::spinOnce();    //执行回調處理函式，完后後繼續往下執行
        ActionControl();
    }

    return 0;
}
