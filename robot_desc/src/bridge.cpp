#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include <string>

static ros::Publisher left_wheel_pub;
static ros::Publisher right_wheel_pub;

float left = 0;
float right = 0;

void callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  left+=msg->angular.z<0?msg->linear.x:0;
  right+=msg->angular.z>0?msg->linear.x:0;
  std_msgs::Float64 m;
  m.data = left;
  left_wheel_pub.publish(m);
  m.data = right;
  right_wheel_pub.publish(m);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bridge");

  ros::NodeHandle n;

  left_wheel_pub = n.advertise<std_msgs::Float64>("robot/left_wheel_position_controller/command", 100);
  right_wheel_pub = n.advertise<std_msgs::Float64>("robot/right_wheel_position_controller/command", 100);

  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback);

  ros::spin();

  return 0;
}