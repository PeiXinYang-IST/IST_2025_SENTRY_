#ifndef _SERIAL_NODE_
#define _SERIAL_NODE_

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>   
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <sstream> 
#include <std_msgs/Bool.h>   
//#include <boost/asio.hpp>

//创建窗口对象
serial::Serial ser;

uint8_t test_data[22];
uint8_t data_len = 22;
std::string smooth_cmd_vel;
std::string cmd_vel_topic;

union Serial_Package
{
    struct
    {
        uint8_t  header = 0xAA;
        double linear_x;
        double linear_y;
        double angular_z;
    };
    uint8_t Send_Buffer[18];
};


#endif