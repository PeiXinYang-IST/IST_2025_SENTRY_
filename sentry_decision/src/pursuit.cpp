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

ros::Subscriber pursuit_position_sub;
ros::Publisher fast_planner_pursuit_pub;

geometry_msgs::Point pursuit_position;
geometry_msgs::Point robot_position;

class Pursuit {
public:
    Pursuit(ros::NodeHandle& nh) {
        pursuit_position_sub = nh.subscribe<geometry_msgs::Point>("pursuit_position", 10, &Pursuit::pursuitPositionCallback, this);
        move_base_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
        fast_planner_pursuit_pub = nh.advertise<geometry_msgs::PoseStamped>("fast_planner_pursuit", 10);
        global_path_sub = nh.subscribe<nav_msgs::Path>("/move_base1/NavfnROS/plan", 10, &Pursuit::globalPathCallback, this);
        odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, &Pursuit::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Get the robot's current position
        robot_position.x = msg->pose.pose.position.x;
        robot_position.y = msg->pose.pose.position.y;
        robot_position.z = 0;
    }

    void pursuitPositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
        pursuit_position = *msg;
        // Publish the pursuit position to fast_planner_pursuit topic
        geometry_msgs::PoseStamped pursuit_target;
        pursuit_target.header.stamp = ros::Time::now();
        pursuit_target.header.frame_id = "odom";
        pursuit_target.pose.position.x = pursuit_position.x;
        pursuit_target.pose.position.y = pursuit_position.y;
        pursuit_target.pose.position.z = 0;
        pursuit_target.pose.orientation.w = 1.0; // Assuming no rotation
        
        move_base_goal_pub.publish(pursuit_target);
    }

    geometry_msgs::Point findTargetPoint(const nav_msgs::Path& path, const geometry_msgs::Point& target, double min_distance) {
        for (const auto& pose : path.poses) {
            double distance = sqrt(pow(pose.pose.position.x - target.x, 2) + pow(pose.pose.position.y - target.y, 2));
            if (distance <= min_distance) {
                geometry_msgs::Point target_point;
                target_point.x = pose.pose.position.x;
                target_point.y = pose.pose.position.y;
                target_point.z = 0;
                return target_point;
                break;
            }
        }
        return robot_position; // Return the original target if no point is found
    }

    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (msg->poses.empty()) return;

            double distance = sqrt(pow(robot_position.x - pursuit_position.x, 2) + pow(robot_position.y - pursuit_position.y, 2));
            double min_distance = 0.6; // Minimum distance to the target point
            if (distance >= min_distance) {
                geometry_msgs::Point target_point = findTargetPoint(*msg, pursuit_position, min_distance);
                geometry_msgs::PoseStamped pursuit_target;
                pursuit_target.header.stamp = ros::Time::now();
                pursuit_target.header.frame_id = "odom";
                pursuit_target.pose.position.x = target_point.x;
                pursuit_target.pose.position.y = target_point.y;
                pursuit_target.pose.position.z = 0;
                pursuit_target.pose.orientation.w = 1.0; // Assuming no rotation
                
                fast_planner_pursuit_pub.publish(pursuit_target);
            }
    }

private:
    ros::Subscriber pursuit_position_sub;
    ros::Publisher fast_planner_pursuit_pub;    
    ros::Subscriber global_path_sub;
    ros::Publisher move_base_goal_pub;
    ros::Subscriber odom_sub;
    geometry_msgs::Point pursuit_position;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pursuit_node");
    ros::NodeHandle nh;

    Pursuit pursuit(nh);

    ros::spin();
    return 0;
}