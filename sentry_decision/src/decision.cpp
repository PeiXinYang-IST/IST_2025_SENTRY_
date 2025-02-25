// Created by: Peixin Yang
/* pursuit 模式  通过障碍物的位置信息进行追踪,获得障碍物聚类坐标之后,
将其作为move_base全局路径规划的目标点,之后路径上选取距离目标点恰当距离点进行追踪 */

/* local_Patrolling (原地范围内随机巡逻) 模式 在机器人周围搜寻合法目标点进行巡逻 */

/* global_Patrolling (全局范围内随机巡逻) 模式 在地图范围内搜寻合法目标点进行巡逻 */

#include <ros/ros.h>
#include <std_msgs/String.h>
ros::Publisher mode_pub;

enum NavigationMode {
    PURSUIT, //0
    LOCAL_PATROLLING, //1
    GLOBAL_PATROLLING //2
};

NavigationMode current_mode;

void setNavigationMode(NavigationMode mode) {
    current_mode = mode;
    switch (mode) {
        case PURSUIT:
            // 设置追踪模式的相关参数
            break;
        case LOCAL_PATROLLING:
            // 设置本地巡逻模式的相关参数
            break;
        case GLOBAL_PATROLLING:
            // 设置全局巡逻模式的相关参数
            break;
        default:
            // 处理未知模式
            break;
    }
}

void publishMode(NavigationMode mode) {
    std_msgs::String msg;
    switch (mode) {
        case PURSUIT:
            msg.data = "PURSUIT";
            break;
        case LOCAL_PATROLLING:
            msg.data = "LOCAL_PATROLLING";
            break;
        case GLOBAL_PATROLLING:
            msg.data = "GLOBAL_PATROLLING";
            break;
        default:
            msg.data = "UNKNOWN";
            break;
    }
    mode_pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sentry_decision");
    ros::NodeHandle nh;

    mode_pub = nh.advertise<std_msgs::String>("navigation_mode", 10);

    // Example usage
    setNavigationMode(PURSUIT);
    publishMode(current_mode);

    ros::spin();
    return 0;
}