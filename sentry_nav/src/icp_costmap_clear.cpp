// rosservice call /move_base1/clear_costmaps 

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>  // 确保引入正确的服务类型
#include <thread>
#include <nav_msgs/Path.h>

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
        global_path_sub = nh.subscribe<nav_msgs::Path>("/move_base1/NavfnROS/plan", 10, &ClearGlobalCostmapNode::globalPathCallback, this);
    }

    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        global_path_ = *msg;
        plan_ = true;
    }

    void pidclearcostmapCallback(const std_msgs::Bool::ConstPtr& msg) {

        if (msg->data) {  // 如果消息为 true
            clearCostmap();
            ROS_WARN("pid clear costmap!!!");
        }
    }

    void moveBaseStartCallback(const std_msgs::Bool::ConstPtr& msg) {
        clear_costmap_client_.call(srv);  //第一次清理全局代价地图
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
            clear_costmap_client_.call(srv);
        //     static int check_path;
        //     static int time_count;
        //     if(++time_count>=10) //1s检查一次
        //     {
        //         //clear_costmap_client_.call(srv);
        //         time_count=0;
        //     }
        //     if(++check_path>=10 && plan_) //1s检查一次
        // {
        //     if(global_path_.poses.empty())
        //     {
        //         clear_costmap_client_.call(srv);
        //         check_path=0;
        //         // ROS_WARN("global_path CHECK!!!");  //全局路径看门狗叫
        //     }
        // }
            ros::Duration(0.01).sleep();  //100hz检查
        }
    }
    
private:
    ros::Subscriber move_base_start_sub_;
    ros::Subscriber pid_clear_costmap_sub_;
    ros::ServiceClient clear_costmap_client_;
    ros::Subscriber global_path_sub;
    std_srvs::Empty srv;  // 使用 std_srvs::Empty
    nav_msgs::Path global_path_;
    bool plan_=false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "clear_global_costmap_node");

    ClearGlobalCostmapNode node;

    ros::spin();  // 保持节点运行
    return 0;
}
