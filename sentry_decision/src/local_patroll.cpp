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
    void publishGoal(ros::Publisher& pub, double x, double y, double z);
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

void local_patroll::publishGoal(ros::Publisher& pub, double x, double y, double z) {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.z = z;
    goal.pose.orientation.w = 1;
    pub.publish(goal);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_patroll");
    ROS_INFO("begin");
    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    size_t current_waypoint = 0;
    ros::Rate rate(0.1); // 0.1 Hz -> 10 seconds

    ros::spin();
    return 0;
}