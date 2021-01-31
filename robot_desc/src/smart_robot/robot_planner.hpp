#pragma once

#include "robot_controller.hpp"
#include <sensor_msgs/LaserScan.h>

class RobotPlanner{
public:
    RobotPlanner(RobotController& rc_p);
    void lidar_info(const sensor_msgs::LaserScan::ConstPtr& msg);
    void update();
    void goTo(tf::Vector3 loc);
    bool done(){return done_;}
private:
    RobotController& rc;
    std::list<tf::Vector3> general_path;
	std::vector<tf::Vector3> blocks;
    tf::Vector3 curr_target;
    bool addSwitch = false;
    bool done_ = false;
	static const int s = 20;
	char l_map[s][s];
    const float robot_radius = 0.7;
    const float obj_radius = 0.2;
};
