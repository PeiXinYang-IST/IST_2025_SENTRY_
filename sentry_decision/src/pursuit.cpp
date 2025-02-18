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

ros::Subscriber pursuit_position_sub;
ros::Publisher fast_planner_pursuit_pub;
ros::Publisher marker_pub;

geometry_msgs::PoseStamped pursuit_position;
geometry_msgs::Point robot_position;


class Pursuit {
public:
Pursuit(ros::NodeHandle& nh) : nh(nh), last_publish_time(ros::Time::now()) {
    last_target_point.x = 0.0;
    last_target_point.y = 0.0;
    last_target_point.z = 0.0;
    pursuit_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("Obstacle_cloudget/pursuit_position", 10, &Pursuit::pursuitPositionCallback, this);
    move_base_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    fast_planner_pursuit_pub = nh.advertise<geometry_msgs::PoseStamped>("fast_planner_pursuit", 10);
    global_path_sub = nh.subscribe<nav_msgs::Path>("/move_base1/NavfnROS/plan", 10, &Pursuit::globalPathCallback, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, &Pursuit::odomCallback, this);
    marker_pub = nh.advertise<visualization_msgs::Marker>("pursuit_target_marker", 10);
    navigation_mode_sub = nh.subscribe<std_msgs::Bool>("navigation_mode", 10, &Pursuit::navigationModeCallback, this);
}

void navigationModeCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data == PURSUIT) {
        ROS_INFO("PURSUIT mode enabled");
        pursuit_mode_enabled = true;
    }
    else
    {
        ROS_INFO("PURSUIT mode disabled");
        pursuit_mode_enabled = false;
    }
}

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Get the robot's current position
        robot_position.x = msg->pose.pose.position.x;
        robot_position.y = msg->pose.pose.position.y;
        robot_position.z = 0;
        // ROS_INFO("Robot position: (%.2f, %.2f)", robot_position.x, robot_position.y);
    }

    void pursuitPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        ROS_INFO("begin");

        pursuit_position = *msg;
        // Publish the pursuit position to fast_planner_pursuit topic
        geometry_msgs::PoseStamped pursuit_target;
        pursuit_target.header.stamp = ros::Time::now();
        pursuit_target.header.frame_id = "odom";
        pursuit_target.pose.position.x = pursuit_position.pose.position.x;
        pursuit_target.pose.position.y = pursuit_position.pose.position.y;
        pursuit_target.pose.position.z = 0;
        pursuit_target.pose.orientation.w = 1.0; // Assuming no rotation
        
        move_base_goal_pub.publish(pursuit_target);
        ROS_INFO("Pursuit target: (%.2f, %.2f)", pursuit_position.pose.position.x, pursuit_position.pose.position.y);
    }

    geometry_msgs::Point findTargetPoint(const nav_msgs::Path& path, const geometry_msgs::PoseStamped& target, double min_distance) {
        for (const auto& pose : path.poses) {
            double distance = sqrt(pow(pose.pose.position.x - target.pose.position.x, 2) + pow(pose.pose.position.y - target.pose.position.y, 2));
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

        double distance = sqrt(pow(robot_position.x - pursuit_position.pose.position.x, 2) + pow(robot_position.y - pursuit_position.pose.position.y, 2));
        double min_distance = 0.4; // Minimum distance to the target point
        if (distance >= min_distance) {
            geometry_msgs::Point target_point = findTargetPoint(*msg, pursuit_position, min_distance);
            double distance_to_last = sqrt(pow(target_point.x - last_target_point.x, 2) + pow(target_point.y - last_target_point.y, 2));
            ros::Time now = ros::Time::now();
            if ((now - last_publish_time).toSec() >= 0.1 || distance_to_last >= 0.2) {
                last_publish_time = now;
                last_target_point = target_point;

                geometry_msgs::PoseStamped pursuit_target;
                pursuit_target.header.stamp = now;
                pursuit_target.header.frame_id = "odom";
                pursuit_target.pose.position.x = target_point.x;
                pursuit_target.pose.position.y = target_point.y;
                pursuit_target.pose.position.z = 0;
                pursuit_target.pose.orientation.w = 1.0; // Assuming no rotation

                // Visualize the pursuit target with a marker
                visualization_msgs::Marker marker;
                marker.header.frame_id = "odom";
                marker.header.stamp = now;
                marker.ns = "pursuit_target";
                marker.id = 0;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = target_point.x;
                marker.pose.position.y = target_point.y;
                marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.2;
                marker.scale.y = 0.2;
                marker.scale.z = 0.2;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                // Publish the marker
                marker_pub.publish(marker);
                fast_planner_pursuit_pub.publish(pursuit_target);
                ROS_INFO("Pursuit target: (%.2f, %.2f)", target_point.x, target_point.y);
            }
        }
    }

private:
    ros::Subscriber pursuit_position_sub;
    ros::Publisher fast_planner_pursuit_pub;    
    ros::Subscriber global_path_sub;
    ros::Publisher move_base_goal_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber navigation_mode_sub;
    geometry_msgs::PoseStamped pursuit_position;
    bool pursuit_mode_enabled = false;
    ros::NodeHandle& nh; // 使用引用
    ros::Time last_publish_time; // 上一次发布消息的时间
    geometry_msgs::Point last_target_point; // 上一次发布的目标点位置
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pursuit_node");
    ROS_INFO("begin");
    ros::NodeHandle nh;
    Pursuit pursuit(nh);
    ros::spin();
    return 0;
}