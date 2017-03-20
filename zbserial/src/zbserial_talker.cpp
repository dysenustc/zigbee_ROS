#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
 
static int counter = 0;
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    //订阅write主题（即发送串口数据）
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("write", 1000);
    ros::Rate loop_rate(10);
    
    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        
        std::stringstream ss;
        ++counter;
        ss << "hello world " << counter << count;
        msg.data = ss.str();        
        ROS_INFO("%s", msg.data.c_str());       
        chatter_pub.publish(msg);       
        ros::spinOnce();        
        loop_rate.sleep();
        ++count;
    }
    
    return 0;
}