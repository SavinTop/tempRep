#include "robot_controller.hpp"

void RobotController::update()
{
    if (busy)
        curr_perf();
    else if (!moves.empty())
    {
        curr_move = moves.front();
        moves.pop_front();
        lastPos = curr_position;
        lastAngle = curr_angle;
	if(curr_move.mt == move_type::coordinate_set){
	    dec_coord_move();
	    update();
            return;
	}
        if (curr_move.mt == move_type::linear)
        {
            curr_perf = std::bind(&RobotController::perform_linear, this);
            ROS_INFO("current move set to linear %f",curr_move.linear);
        }
        else
        {
            if (curr_move.mt == move_type::rotate_add)
                curr_move.angular += curr_angle;
            curr_perf = std::bind(&RobotController::perform_angular, this);
            ROS_INFO("current move set to angular %f",curr_move.angular);
        }
        busy = true;
        
    }

    if (abort_set)
    {
        moves.clear();
        abort_set = false;
        busy = false;
        geometry_msgs::Twist pub_msg;
    	pub_msg.angular.z = 0;
        pub_msg.linear.x = 0;
    	cmd_pub.publish(pub_msg);
    }
}

void RobotController::odom_info(const nav_msgs::Odometry::ConstPtr &msg)
{
    float currMsgTime = msg->header.stamp.toSec();
    float dt = currMsgTime - lastTime;
    time = currMsgTime;
    float linear = msg->twist.twist.linear.y * dt;
    curr_angle += msg->twist.twist.angular.z * dt;
    if(curr_angle>M_PI*2)
        curr_angle-=M_PI*2;
    else if(curr_angle<0)
        curr_angle+=M_PI*2;
    tf::Vector3 vel{-cos(curr_angle) * linear, -sin(curr_angle) * linear, 0};
    curr_position += vel;
    curr_speed=tf::tfDistance(tf::Vector3(), vel);
    lastTime = currMsgTime;
    //ROS_INFO("x %f y %f z %f r %f",curr_position.getX(),curr_position.getY(),curr_position.getZ(), curr_angle);
}

void RobotController::setRotation(float rot)
{
    moves.push_back(move_el(move_type::rotate_set, 0, rot,1));
}

void RobotController::addRotation(float rot)
{
    moves.push_back(move_el(move_type::rotate_add, 0, rot,1));
}

void RobotController::goForward(float dist)
{
    moves.push_back(move_el(move_type::linear, +dist, 0));
}

void RobotController::goBackwards(float dist)
{
    moves.push_back(move_el(move_type::linear, -dist, 0));
}

void RobotController::goTo(tf::Vector3 loc)
{
    moves.push_back(move_el(move_type::coordinate_set, 0, 0, 1,loc));
}

void RobotController::abort()
{
    abort_set = true;
}

void RobotController::perform_linear()
{
    float target = curr_move.linear;
    float currLen = (target<0?-1:1) * tf::tfDistance(lastPos, curr_position);
    float err = target - currLen;
    float speed = err * 0.33;
    if(speed>1.0)
        speed = 1;
    if (abs(err) < 0.1)
    {
        speed = 0;
        busy = false;
    }
    geometry_msgs::Twist pub_msg;
    pub_msg.linear.x = speed;
    cmd_pub.publish(pub_msg);
    std_msgs::Float64 pid_msg;
    pid_msg.data = 1-abs(err/(target+0.00001));
    pid_pub.publish(pid_msg);
}

void RobotController::perform_angular()
{
    ROS_INFO("%f %f %i\n",curr_angle, curr_move.angular,curr_move.angularSign);
    float ang = curr_move.angular;
    float err = curr_move.angularSign*abs(ang - curr_angle);

    float speed = err*0.33;
    if(abs(speed)>0.45)
        speed = curr_move.angularSign*0.45;
    if (abs(err) < 0.05)
    {
        speed = 0;
        busy = false;
    }
    geometry_msgs::Twist pub_msg;
    pub_msg.angular.z = speed;
    cmd_pub.publish(pub_msg);
    std_msgs::Float64 pid_msg;
    pid_msg.data = 1-abs(err/(ang+0.00001));
    pid_pub.publish(pid_msg);
}

void RobotController::dec_coord_move(){
    auto loc = curr_move.loc; 
    float ang = atan2(loc.getY()-curr_position.getY(), loc.getX()-curr_position.getX());
    if(ang>M_PI*2)
        ang-=M_PI*2;
    else if(ang<0)
        ang+=M_PI*2;
    float len = tf::tfDistance(curr_position, loc);
    ROS_INFO("current dist set to %f %f %f",loc.x(), loc.y(), loc.z());
    moves.push_front(move_el(move_type::linear, len, 0));
    short sign = 1;
    if(ang-curr_angle<0 || abs(ang-curr_angle)>M_PI)
        sign = -1;
    moves.push_front(move_el(move_type::rotate_set, 0, ang, sign));
}

RobotController::RobotController(ros::Publisher &cmd_pub_p,ros::Publisher& pid_pub_p)
    : cmd_pub(cmd_pub_p), pid_pub(pid_pub_p)
{
    lastTime = 0;
    abort_set = false;
    busy = false;
    curr_position = tf::Vector3();
    curr_angle = 0;
}
