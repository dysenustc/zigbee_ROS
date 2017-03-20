/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

serial::Serial ser;  //声明串口对象

// 回调函数
void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data); //发送串口数据
}

int main (int argc, char** argv){
    // 初始化节点
    ros::init(argc, argv, "zbserial_node");
    // 声明节点句柄
    ros::NodeHandle nh;
    // 订阅主题，并配置回调函数
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    // 发布主题
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        // 设置串口属性
        ser.setPort("/dev/ttyUSB3");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    // 检测串口是否已经打开，并给出提示信息
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    // 指定循环的频率
    ros::Rate loop_rate(5);
    while(ros::ok()){

        // 处理ROS信息，比如订阅消息，并调用回调函数
        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}



// #include <string>
// #include <ros/ros.h>                           // 包含ROS的头文件
// #include <sensor_msgs/JointState.h>
// #include <tf/transform_broadcaster.h>
// #include <nav_msgs/Odometry.h>
// #include <boost/asio.hpp>                  //包含boost库函数
// #include <boost/bind.hpp>
// #include <math.h>
// #include "std_msgs/String.h"              //ros定义的String数据类型

// using namespace std;
// using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

// unsigned char buf[24];                      //定义字符串长度

// int main(int argc, char** argv) {

//     ros::init(argc, argv, "zbserial_node");       //初始化节点
//     ros::NodeHandle n;
    
//     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);      //定义发布消息的名称及sulv

//     ros::Rate loop_rate(10);


//     io_service iosev;
//     serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口
//     sp.set_option(serial_port::baud_rate(115200));   
//     sp.set_option(serial_port::flow_control());
//     sp.set_option(serial_port::parity());
//     sp.set_option(serial_port::stop_bits());
//     sp.set_option(serial_port::character_size(8));

//     while (ros::ok()) {
//       // write(sp, buffer(buf1, 6));  //write the speed for cmd_val    
//      //write(sp, buffer("Hello world", 12));  
//         read (sp,buffer(buf));
//         string str(&buf[0],&buf[22]);            //将数组转化为字符串
//   //if(buf[0]=='p' && buf[21] == 'a')
//    // {
//         std_msgs::String msg;
//         std::stringstream ss;
//         ss <<str;
     
//         msg.data = ss.str();
     
//         ROS_INFO("%s", msg.data.c_str());//打印接受到的字符串
//         chatter_pub.publish(msg);   //发布消息

//         ros::spinOnce();

//         loop_rate.sleep();
//   //  }
//     }

//     iosev.run(); 
//     return 0;
// }