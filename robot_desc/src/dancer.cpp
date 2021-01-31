#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <tf/tf.h>
#include "smart_robot/robot_planner.hpp"

enum class currEv{rotating, movingForward};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dancer");

  ros::NodeHandle n;

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Publisher pid_pub = n.advertise<std_msgs::Float64>("pid_info", 100);

  RobotController rc(cmd_vel_pub, pid_pub);
  RobotPlanner rp(rc);
  
  auto odom_call = boost::function<void (const nav_msgs::Odometry::ConstPtr& msg)>(boost::bind(&RobotController::odom_info, &rc, _1));
  ros::Subscriber odom = n.subscribe("odom", 1000, odom_call);

  auto scan_call = boost::function<void (const sensor_msgs::LaserScan::ConstPtr& msg)>(boost::bind(&RobotPlanner::lidar_info, &rp, _1));
  ros::Subscriber scan = n.subscribe("base_scan",100,scan_call);

  rp.goTo(tf::Vector3(10,10,0));
  rp.goTo(tf::Vector3(0,0,0));

  while(ros::ok())
  {
    rp.update();

    ros::spinOnce();
    ros::Duration(0.2).sleep();
  }

  ROS_INFO("dancer done");

  return 0;
}
