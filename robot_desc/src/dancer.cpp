#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

float angle = 0;
float pos_x = 0;
float pos_y = 0;
float lastTime = 0;

void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  float currMsgTime = msg->header.stamp.toSec();
  float delta = currMsgTime-lastTime;
  float linear = msg->twist.twist.linear.y*delta;
  angle+=msg->twist.twist.angular.z*delta;
  float v_pos_x = cos(angle)*linear;
  float v_pos_y = -sin(angle)*linear;
  pos_x+=v_pos_x;
  pos_y+=v_pos_y;
  ROS_INFO("coords %f %f angle %f",pos_x,pos_y, angle*180/3.1415);
  lastTime = currMsgTime;
}

enum class currEv{rotating, movingForward};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dancer");

  ros::NodeHandle n;
  
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

  ros::Subscriber sub = n.subscribe("odom", 1000, callback);
 

  const float square_s = 3;
  const float ang = 3.1415/2;
  float curr_dist = 0;
  float lastPos_x = 0;
  float lastPos_y = 0;
  float lastAngle = 0;
  currEv currState = currEv::movingForward;

  while(ros::ok())
  {
    if(currState==currEv::movingForward)
    {
        float currLen = sqrt(pow(pos_x-lastPos_x,2)+pow(pos_y-lastPos_y,2));
        float err = square_s-currLen;
        float speed = err * 0.5;
        if(err<0.01)
        {
            speed = 0;
            lastPos_x = pos_x;
            lastPos_y = pos_y;
            currState = currEv::rotating;
        }
        geometry_msgs::Twist pub_msg;
        pub_msg.linear.x = speed;
        cmd_vel_pub.publish(pub_msg);
    }
    else if(currState==currEv::rotating)
    {
        float currAngle = angle-lastAngle;
        float err = ang-currAngle;
        float speed = err*0.33;
        if(err<0.01)
        {
            speed = 0;
            lastAngle = angle;
            currState = currEv::movingForward;
        }
        geometry_msgs::Twist pub_msg;
        pub_msg.angular.z = speed;
        cmd_vel_pub.publish(pub_msg);
        
    }

    ros::spinOnce();
    ros::Duration(0.2).sleep();
  }

  return 0;
}
