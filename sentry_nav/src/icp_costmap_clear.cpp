// rosservice call /move_base1/clear_costmaps 

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>  // 确保引入正确的服务类型
#include <thread>

class ClearGlobalCostmapNode {
public:
    ClearGlobalCostmapNode() {
        // 初始化节点
        ros::NodeHandle nh;


        // 订阅 move_base_start 话题
        move_base_start_sub_ = nh.subscribe("MY_ICP/move_base_start", 10, &ClearGlobalCostmapNode::moveBaseStartCallback, this);
        pid_clear_costmap_sub_ = nh.subscribe("/pid_follow/pid_clear_costmap", 10, &ClearGlobalCostmapNode::pidclearcostmapCallback, this);
        // 创建 ClearCostmap 服务客户端
        clear_costmap_client_ = nh.serviceClient<std_srvs::Empty>("/move_base1/clear_costmaps");
    }

    void pidclearcostmapCallback(const std_msgs::Bool::ConstPtr& msg) {

        if (msg->data) {  // 如果消息为 true
            clearCostmap();
            ROS_WARN("pid clear costmap!!!");
        }
    }


    void moveBaseStartCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data) {  // 如果消息为 true
            clearCostmap();
            ROS_WARN("icp clear costmap!!!");
        }
    }

    void clearCostmap() {
        std::thread clear_thread(&ClearGlobalCostmapNode::map_clear_thread, this);
        clear_thread.join(); //阻塞主线程
    }

    void map_clear_thread()
    {
        while(ros::ok())
        {
            std_srvs::Empty srv;  // 使用 std_srvs::Empty
        if (clear_costmap_client_.call(srv)) {
            ROS_INFO("Global costmap cleared.");
        } else {
            ROS_ERROR("Failed to clear global costmap.");
        }
            ros::Duration(0.5).sleep();  //1hz
        }
    }
    
private:
    ros::Subscriber move_base_start_sub_;
    ros::Subscriber pid_clear_costmap_sub_;
    ros::ServiceClient clear_costmap_client_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "clear_global_costmap_node");

    ClearGlobalCostmapNode node;

    ros::spin();  // 保持节点运行
    return 0;
}
