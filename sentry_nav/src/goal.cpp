#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

class GoalPublisher
{
public:
    GoalPublisher()
    {
        // Initialize ROS node handle and publisher
        pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

        // Initialize waypoints
        initializeWaypoints();

        // Set timer to publish waypoints every 8 seconds
        timer_ = nh_.createTimer(ros::Duration(8), &GoalPublisher::timerCallback, this);
    }
        private:
            void initializeWaypoints()
            {
                geometry_msgs::PoseStamped waypoint;

                // Add first waypoint
                // waypoint.pose.position.x = 3.1;
                // waypoint.pose.position.y = 5.685;
                // waypoint.pose.orientation.w = 1.0;
                // waypoints_.push_back(waypoint);

                // waypoint.pose.position.x = 3.198;
                // waypoint.pose.position.y = 7.8;
                // waypoint.pose.orientation.w = 1.0;
                // waypoints_.push_back(waypoint);

                // // Add second waypoint
                // waypoint.pose.position.x = 1.706;
                // waypoint.pose.position.y = -0.304;
                // waypoint.pose.orientation.w = 1.0;
                // waypoints_.push_back(waypoint);

                // Add second waypoint
                waypoint.pose.position.x = 0;
                waypoint.pose.position.y = 0;
                waypoint.pose.orientation.w = 1.0;
                waypoints_.push_back(waypoint);

                waypoint.pose.position.x = -0.414;
                waypoint.pose.position.y = -3.81;
                waypoint.pose.orientation.w = 1.0;
                waypoints_.push_back(waypoint);

                waypoint.pose.position.x = -1.438;
                waypoint.pose.position.y = -6.51;
                waypoint.pose.orientation.w = 1.0;
                waypoints_.push_back(waypoint);

                waypoint.pose.position.x = -2.396;
                waypoint.pose.position.y = -4.19;
                waypoint.pose.orientation.w = 1.0;
                waypoints_.push_back(waypoint);                
            }

    void timerCallback(const ros::TimerEvent&)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "odom";
        pose_msg.pose = waypoints_[current_waypoint_].pose;
        pose_msg.pose.orientation.w = 1.0;
        // Publish the current waypoint
        pub_.publish(pose_msg);

        // Move to the next waypoint, loop back to the first if at the end
        current_waypoint_ = (current_waypoint_ + 1) % waypoints_.size();
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Timer timer_;
    std::vector<geometry_msgs::PoseStamped> waypoints_;
    size_t current_waypoint_ = 0;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_publisher");
    GoalPublisher goal_publisher;
    ros::spin();
    return 0;
}