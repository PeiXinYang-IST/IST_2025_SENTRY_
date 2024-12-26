#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <boost/thread/mutex.hpp>
#include <thread>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

class APFController {
public:
    APFController() {
        nh_ = ros::NodeHandle("~");
        apf_cmd_vel_pub_ =  nh_.advertise<geometry_msgs::Twist>("apf_cmd_vel", 10);
        costmap_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/move_base1/local_costmap/costmap", 10,
                                                         &APFController::costmapCallback, this);
        odom_sub_ = nh_.subscribe("/Odometry", 10, &APFController::odom_callback, this);
        apfThread_ = std::thread(&APFController::apf_thread, this);

    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 直接将里程计数据中的 pose 部分作为机器人位置
        robot_pose_.pose.position = msg->pose.pose.position;
        robot_pose_.pose.orientation = msg->pose.pose.orientation;
    }

    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        potential_map_ = *msg;
    }

    void apf_thread()
    {
        calculate_avoidence();
    }

    //斥力场函数
    void calculate_avoidence()
    {
        ros::Rate loop_rate(30); // 设置循环频率为30Hz
        while(ros::ok())
        {
            // 获取当前时间
        ros::Time time = ros::Time::now();
        
        // 等待坐标变换可用
        listener.waitForTransform("robot_foot_init", "camera_init", time, ros::Duration(1.0));
        
        // 将 robot_pose 变换到目标坐标系
        geometry_msgs::PoseStamped robot_pose_transformed;
        try {
            // 确保 robot_pose 的 header.frame_id 被设置为 camera_init
            robot_pose_.header.frame_id = "camera_init";
            
            // 将 robot_pose 变换到 robot_foot_init 坐标系
            listener.transformPose("robot_foot_init", robot_pose_, robot_pose_transformed);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("Received an exception trying to transform the robot pose: %s", ex.what());
            // 如果发生异常，选择跳过当前循环
            continue;
        }

            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher apf_cmd_vel_pub_;
    ros::Subscriber costmap_sub_;
    ros::Subscriber odom_sub_;
    tf::TransformListener listener;
    geometry_msgs::Twist apf_cmd_vel;
    geometry_msgs::PoseStamped robot_pose_;
    nav_msgs::OccupancyGrid potential_map_;  // local potential field map
    std::thread apfThread_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "APFController");
    APFController apfcontroller;
    ros::spin();
    return 0;
}