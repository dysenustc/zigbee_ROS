#include "ros/ros.h"
#include "std_msgs/String.h"
 
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    //订阅read主题（即接受串口数据）
    ros::Subscriber sub = nh.subscribe("read", 1000, chatterCallback);
    ros::spin();
    return 0;
}