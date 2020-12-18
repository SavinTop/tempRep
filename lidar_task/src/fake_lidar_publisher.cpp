#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include <random>

const float def_angle_min = 0;
const float def_angle_max = M_PI;
const float def_angle_increment = 1.0f/180;
const float def_time_increment = 1.0f/10;
const float def_scan_time = 1.0f/10;
const float def_range_min = 0;
const float def_range_max = 180;

int main(int argc, char** argv){
    ros::init(argc, argv, "fake_lidar");

    ros::NodeHandle nh;

    ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>("fake_lidar_info", 1000);

    ros::Rate loop_rate{10};

    std::random_device r;
    std::default_random_engine dre(r());
    std::uniform_real_distribution<> uniform_dist(def_range_min, def_range_max);

    std::vector<float> ranges(180, 0.0f);

    while(ros::ok())
    {
        for(auto& el:ranges)
            el = uniform_dist(dre);

        sensor_msgs::LaserScan msg;

        msg.angle_min = def_angle_min;
        msg.angle_max = def_angle_max;
        msg.angle_increment = def_angle_increment;
        msg.time_increment = def_time_increment;
        msg.scan_time = def_scan_time;
        msg.range_min = def_range_min;
        msg.range_max = def_range_max;
        msg.ranges = ranges;

        lidar_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}