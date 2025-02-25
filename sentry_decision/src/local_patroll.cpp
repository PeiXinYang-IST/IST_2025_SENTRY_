#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <ros/ros.h>
#include <thread>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath> // For sqrt and pow
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <decision.h>

//这里判断原地巡逻的条件，电控下位机发送
class local_patroll
{
private:
    /* data */
    
public:
    local_patroll(/* args */);
    ~local_patroll();
};


local_patroll::local_patroll(/* args */)
{

}

local_patroll::~local_patroll()
{

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "local_patroll");
    ROS_INFO("begin");
    ros::NodeHandle nh;
    // local_patroll local_patroll_(nh);
    ros::spin();
    return 0;
}