#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

class GoalPublisher {
public:
    GoalPublisher() : nh_("~") {
        // 初始化发布订阅
        pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        sub_ = nh_.subscribe("/ros2_game_state", 10, &GoalPublisher::gameStateCallback, this);
        
        // 初始化路径点
        initializeWaypoints();
    }

private:
    void initializeWaypoints() {
        geometry_msgs::PoseStamped waypoint;
        waypoint.header.frame_id = "odom";
        // 中心增益区 0
        waypoint.pose.position.x = -1.8;
        waypoint.pose.position.y = 4.7;
        waypoint.pose.orientation.z = 0.760;
        waypoint.pose.orientation.w = 0.650;
        waypoints_.push_back(waypoint);       
        // waypoint.pose.position.x = -1.8;
        // waypoint.pose.position.y = 4.7;
        // waypoint.pose.orientation.z = 0.760;
        // waypoint.pose.orientation.w = 0.650;`
        // waypoints_.push_back(waypoint);     
        // 补给区 1
        waypoint.pose.position.x = 0.0;
        waypoint.pose.position.y = 0.0;
        waypoint.pose.orientation.z = 0.760;
        waypoint.pose.orientation.w = 0.650;
        waypoints_.push_back(waypoint);
        // waypoint.pose.position.x = 1.05;
        // waypoint.pose.position.y = -0.55;
        // waypoint.pose.orientation.z = 0.760;
        // waypoint.pose.orientation.w = 0.650;
        // waypoints_.push_back(waypoint);        //击打增益区 2
        waypoint.pose.position.x = 0.2;
        waypoint.pose.position.y = 5.0;
        waypoint.pose.orientation.z = 0.760;
        waypoint.pose.orientation.w = 0.650;
        waypoints_.push_back(waypoint);
    }

    void gameStateCallback(const std_msgs::UInt8::ConstPtr& msg) {
        // 解析状态
        const uint8_t pos_state = msg->data;

        // 状态切换处理
        handleStateTransition(pos_state);
    }

    void handleStateTransition(uint8_t new_pos_state) {
        // 状态未变化时不做处理
        if (new_pos_state == current_pos_state_) return;

        // 立即发布新状态对应目标点
        publishWaypoint(new_pos_state);
        
        // 停止之前的定时器
        stopStateTimer();
        
        // 启动新状态对应的定时器
        startStateTimer(new_pos_state);
        
        // 更新当前状态记录
        current_pos_state_ = new_pos_state;
    }

    void publishWaypoint(uint8_t state) {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "odom";
        
        // 根据状态选择目标点索引
        int index;
        if (state == 0) {
            index = 0;
        } else if (state == 1) {
            index = 1;
        } else if (state == 2) {
            index = 2;
        } else {
            ROS_WARN("Invalid state: %d", state);
            return;
        }
        
        pose_msg.pose = waypoints_[index].pose;
        pub_.publish(pose_msg);
    }

    void startStateTimer(uint8_t state) {
        // 创建循环定时器（每8秒触发一次）
        state_timer_ = nh_.createTimer(ros::Duration(8.0), 
            [this, state](const ros::TimerEvent&) {
                // 验证状态是否仍然有效
                if (current_pos_state_ == state) {
                    publishWaypoint(state);  // 持续状态则重复发布
                }
            }, 
            false  // 循环触发
        );
    }


    void stopStateTimer() {
        if (state_timer_.isValid()) {
            state_timer_.stop();
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Timer state_timer_;  // 状态持续定时器
    
    // 状态记录变量
    uint8_t current_pos_state_ = 10;
    
    std::vector<geometry_msgs::PoseStamped> waypoints_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_publisher");
    GoalPublisher goal_publisher;
    ros::spin();
    return 0;
}

// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <vector>

// class GoalPublisher
// {
// public:
//     GoalPublisher()
//     {
//         // Initialize ROS node handle and publisher
//         pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

//         // Initialize waypoints
//         initializeWaypoints();

//         // Set timer to publish waypoints every 8 seconds
//         timer_ = nh_.createTimer(ros::Duration(8.0), &GoalPublisher::timerCallback, this);
//     }
//         private:
//             void initializeWaypoints()
//             {
//                 geometry_msgs::PoseStamped waypoint;
//                 waypoint.header.frame_id = "odom";

//                 // Add first waypoint
//                 waypoint.pose.position.x = -3.377;
//                 waypoint.pose.position.y = 0.485;
//                 waypoint.pose.orientation.w = 1.0;
//                 waypoints_.push_back(waypoint);

//                 // Add second waypoint
//                 waypoint.pose.position.x = -0.306;
//                 waypoint.pose.position.y = -1.304;
//                 waypoint.pose.orientation.w = 1.0;
//                 waypoints_.push_back(waypoint);
//             }

//     void timerCallback(const ros::TimerEvent&)
//     {
//         geometry_msgs::PoseStamped pose_msg;
//         pose_msg.header.stamp = ros::Time::now();
//         pose_msg.header.frame_id = "odom";
//         pose_msg.pose = waypoints_[current_waypoint_].pose;
//         pose_msg.pose.orientation.w = 1.0;
//         // Publish the current waypoint
//         pub_.publish(pose_msg);

//         // Move to the next waypoint, loop back to the first if at the end
//         current_waypoint_ = (current_waypoint_ + 1) % waypoints_.size();
//     }

//     ros::NodeHandle nh_;
//     ros::Publisher pub_;
//     ros::Timer timer_;
//     std::vector<geometry_msgs::PoseStamped> waypoints_;
//     size_t current_waypoint_ = 0;
// };

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "goal_publisher");
//     GoalPublisher goal_publisher;
//     ros::spin();
//     return 0;
// }