#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ctime>
#include <std_msgs/Bool.h> 
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>

ros::Subscriber odom_sub_;
ros::Publisher odom_pub_;

// 里程计数据的回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {

    odom_pub_.publish(odom_msg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "Odom2odom");
    ros::NodeHandle nh_;
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    odom_sub_ = nh_.subscribe("Odometry", 1, odomCallback);
    ros::spin();
    return 0;
}
