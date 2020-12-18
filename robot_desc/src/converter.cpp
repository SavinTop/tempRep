#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <string>

static ros::Publisher out_pub;

void callback(const std_msgs::String::ConstPtr &msg)
{
    int numCount = 0;
    geometry_msgs::Pose out_msg;
    std::string s = msg->data;
    std::string delimiter = ";";
    float nums[3] = {};
    auto handleToken = [&numCount, &nums](const std::string& token){
        float val;
        try{ val = std::stof(token); } catch (...) { return; }
        nums[numCount++] = val;
    };
    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos && numCount<3)
    {
        token = s.substr(0, pos);
        handleToken(token);
        s.erase(0, pos + delimiter.length());
    }
    if(numCount<3)
        handleToken(s);
    out_msg.position.x = nums[0];
    out_msg.position.y = nums[1];
    out_msg.position.z = nums[2];
    out_pub.publish(out_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "strtoxyz");

    ros::NodeHandle n;

    out_pub = n.advertise<geometry_msgs::Pose>("poseFromString", 100);

    ros::Subscriber sub = n.subscribe("lector620readings", 1000, callback);

    ros::spin();

    return 0;
}