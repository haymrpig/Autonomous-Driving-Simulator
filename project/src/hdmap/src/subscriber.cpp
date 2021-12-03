#include "ros/ros.h"
#include "std_msgs/String.h"
#include "adasone_msgs/ObdParam.h"
#include "adasone_msgs/Steering.h"

void chatterCallback(const adasone_msgs::Steering::ConstPtr& msg)
{
    ROS_INFO("steering from hdmap : [%f]", msg->map_steering);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/hdmap_steering", 1000, chatterCallback);

    ros::spin();

    return 0;
}
