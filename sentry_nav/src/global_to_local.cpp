#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <std_msgs/Bool.h>

class Global_to_Local
{
private:
    ros::NodeHandle nh_;
    nav_msgs::Path global_path_;
    ros::Subscriber global_path_sub_;
    ros::Publisher goal_pose_pub_;
    ros::Publisher start_pose_pub_;
    geometry_msgs::PoseStamped goal_pose;  // 局部目标点
    geometry_msgs::PoseStamped start_pose; // 车体当前点，即全局路径开始点

    // 计算两点间的欧氏距离
    double euclideanDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
        return std::sqrt(std::pow(p2.pose.position.x - p1.pose.position.x, 2) +
                         std::pow(p2.pose.position.y - p1.pose.position.y, 2));
    }

public:
    // 处理全局路径的回调函数
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        global_path_ = *msg;  // 更新全局路径
        if (!global_path_.poses.empty()) {
            get_goal_pose();  // 获取目标点
        }
    }

    // 获取目标点：在 2 米范围内选最远的点
    void get_goal_pose() {
        static geometry_msgs::PoseStamped current_goal_pose;
        if (global_path_.poses.empty()) return;

        start_pose = global_path_.poses[0];  // 设定起始点为路径的第一个点
        double max_distance = 0.0;  // 最大距离初始化

        // 遍历路径点，寻找 2 米内最远的点
        for (int i = 1; i < global_path_.poses.size(); ++i) {
            double dist = euclideanDistance(start_pose, global_path_.poses[i]);
            if (dist <= 2.0 && dist > max_distance) {  // 只考虑 2 米内的点
                max_distance = dist;
                goal_pose = global_path_.poses[i];  // 更新最远点
            }
        }

        // 如果找到了有效的目标点
        if (max_distance > 0) {
            goal_pose = goal_pose;  // 更新目标点
            if(euclideanDistance(goal_pose,current_goal_pose)>0.15)
            {
            goal_pose_pub_.publish(goal_pose);
            current_goal_pose = goal_pose;
            }
            ROS_INFO("Goal Pose set at distance: %f meters", max_distance);
        } else {
            ROS_WARN("No valid goal pose found within 2 meters.");
        }
    }

    Global_to_Local();
    ~Global_to_Local();
};

Global_to_Local::Global_to_Local()
{
    nh_ = ros::NodeHandle("~");
    global_path_sub_ = nh_.subscribe("/move_base1/NavfnROS/plan", 10, &Global_to_Local::pathCallback, this);
    goal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal_pose", 10);  // 发布目标点
}

Global_to_Local::~Global_to_Local()
{
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_to_local");

    Global_to_Local global_to_local;

    ros::spin();  // 保持节点运行
    return 0;
}
