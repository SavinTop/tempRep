#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <tf/tf.h>
#include <vector>
#include <functional>
#include <list>

enum class move_type{
linear, rotate_add, rotate_set, coordinate_set
};

struct move_el{
move_type mt;
float linear;
float angular;
short angularSign;
tf::Vector3 loc;
move_el(){};
move_el(move_type mt, float linear, float angular, short a_sign=1, tf::Vector3 loc = tf::Vector3()){
    this->mt = mt;
    this->linear = linear;
    this->angular = angular;
    this->loc = loc;
    this->angularSign = a_sign;
};
};

class RobotController{
public:
  RobotController(ros::Publisher& cmd_pub_p, ros::Publisher& pid_pub_p);
  void update();
  void odom_info(const nav_msgs::Odometry::ConstPtr& msg);
  void setRotation(float rot);
  void addRotation(float rot);
  void goForward(float dist);
  void goBackwards(float dist);
  void goTo(tf::Vector3 loc);
  void abort();

  void perform_linear();
  void perform_angular();
  void dec_coord_move();

  bool done(){return moves.empty() && !busy;};
  tf::Vector3 getPos(){return curr_position;}
  float getRot(){return curr_angle;}
  move_el& getCurrMove(){return curr_move;}
private:
  tf::Vector3 curr_position;
  float curr_angle;
  ros::Publisher& cmd_pub;
  ros::Publisher& pid_pub;
  float lastTime;

  std::list<move_el> moves;
  move_el curr_move;
  bool abort_set;

  tf::Vector3 lastPos;
  float lastAngle = 0;

  std::function<void(void)> curr_perf;
  float time;
  bool busy;
  float curr_speed;
};
