#include <sentry_serial/goal_send_node.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "goal_send_node");

    // 创建ROS节点句柄
    ros::NodeHandle nh;

    // 创建一个Publisher，用于发布导航目标点消息
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);

    // 设置循环的频率（1Hz）
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        // 创建一个导航目标点消息
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "map"; // 导航目标点相对于地图坐标系

        // 设置导航目标点的位置和姿态
        goal_msg.pose.position.x = -2.8;
        goal_msg.pose.position.y = -0.4;
        goal_msg.pose.position.z = 0.0;
        goal_msg.pose.orientation.x = 0.0;
        goal_msg.pose.orientation.y = 0.0;
        goal_msg.pose.orientation.z = 0.0;
        goal_msg.pose.orientation.w = 1.0;

        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp    = ros::Time::now(); 

        // 发布导航目标点消息
        goal_pub.publish(goal_msg);

        // 打印发布的导航目标点信息
        ROS_INFO("Published goal: x=%.2f, y=%.2f", goal_msg.pose.position.x, goal_msg.pose.position.y);

        // 按照指定频率进行循环
        loop_rate.sleep();
    }

    return 0;
}