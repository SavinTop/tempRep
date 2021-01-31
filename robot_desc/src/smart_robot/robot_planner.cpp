#include "robot_planner.hpp"

RobotPlanner::RobotPlanner(RobotController &rc_p)
    : rc(rc_p)
{
}

void RobotPlanner::lidar_info(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    blocks.clear();
    const float angle_s = msg->angle_increment;
    const float angle_range = abs(msg->angle_max - msg->angle_min);
    auto r_pos = rc.getPos();
    float dist_to_loc = tf::tfDistance(r_pos, curr_target);
    float currAngle = rc.getRot();
    for (auto &el : msg->ranges)
    {
        float x = cos(msg->angle_min+currAngle) * el;
        float y = sin(msg->angle_min+currAngle) * el; 
        float dist_to_el = tf::tfDistance(tf::Vector3(x,y,0), tf::Vector3(0,0,0));
        
        currAngle += angle_s;
    
    if(dist_to_el<dist_to_loc+robot_radius)
        blocks.push_back(tf::Vector3(x,y,0)+r_pos);
    }
}

void RobotPlanner::update()
{
    if(rc.done() && !general_path.empty())
    {
        curr_target = general_path.front();
        general_path.pop_front();
        rc.goTo(curr_target);
        addSwitch = false;
    }
    rc.update();
    if(rc.getCurrMove().mt == move_type::linear)
    {
         bool coll = false;
         unsigned collId;
         float angle = rc.getRot();
         const int stepsAhead = 3;
         tf::Vector3 circles[stepsAhead];
         for(int i=0;i<stepsAhead;i++)
             circles[i] = tf::Vector3(rc.getPos()+tf::Vector3(cos(angle)*robot_radius*(i+1),sin(angle)*robot_radius*(i+1),0));
         for(int i=0;i<stepsAhead && !coll;i++)
         {
             for(int j=0;j<blocks.size();j++)
             {
                 float dist = tf::tfDistance(circles[i], blocks[j]);
                 if(dist<obj_radius+robot_radius)
                 {
                     coll = true; 
                     collId = j;
                     break;
                 }
             }
         }
          
         if(coll)
         {
             rc.abort();
             rc.update();
             if(!addSwitch)
             {
                 general_path.push_front(curr_target);
                 addSwitch = true;
             }
             auto collPos = blocks[collId];
             float l = tf::tfDistance(rc.getPos(),collPos)+robot_radius;
             float angle = atan2(collPos.y()-rc.getPos().y(), collPos.x()-rc.getPos().x());
             ROS_INFO("%f %f %f \n",collPos.x(), collPos.y(), angle*180/M_PI);
             auto next = rc.getPos()+tf::Vector3(cos(angle+M_PI_4)*l, sin(angle+M_PI_4)*l, 0);
             float err = tf::tfDistance(curr_target,next);
             if(err<0.3)
                 next = rc.getPos()+tf::Vector3(cos(angle+M_PI_2)*l, sin(angle+M_PI_2)*l, 0);
             curr_target = next;
             rc.goTo(curr_target);
         }
            
    }
}

void RobotPlanner::goTo(tf::Vector3 loc)
{
    general_path.push_back(loc);
}
