#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void pub_map_to_camera_init() {
    // 创建一个TransformBroadcaster对象
    tf::TransformBroadcaster broadcaster;

    // 创建一个TransformStamped消息
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "camera_init";

    // 设置变换的平移和旋转部分
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1; // 单位四元数，表示没有旋转
    ROS_WARN("11111111111111111111111111111111111111111111111");
    // 发布变换
    broadcaster.sendTransform(transformStamped);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher_node");
    ros::NodeHandle nh;

    // 设置循环频率
    ros::Rate rate(10.0); // 10Hz

    while (ros::ok()) {
        pub_map_to_camera_init();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}