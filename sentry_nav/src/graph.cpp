#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <ros/ros.h>
#include <thread>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath> // For sqrt and pow
#include <std_msgs/Int32.h>

// 定义边结构体
struct Edge {
    int to;
    double weight;
};

// 定义节点结构体
struct Node {
    int id;
    geometry_msgs::Point position; // 节点位置
    std::vector<Edge> edges;
};

// 定义图结构
class Graph {
public:
    Graph() : nh_("~"),path_update(false) {

        // 添加节点
        ////////////////////  3v3 //////////////////////
        // addNode(1,-1.061, -1.141, 0.000);
        // addNode(2,-2.667, -3.356, 0.000);
        // addNode(3,-0.622, -5.781, 0.000);
        // addNode(4,1.726, -5.785, 0.000);
        // addNode(5,1.366, -3.350, 0.000);
        // addNode(6,1.096, -0.966, 0.000);
        // addNode(7,3.598, -5.903, 0.000);
        // addNode(8,5.761, -3.379, 0.000);
        // addNode(9,3.037, -1.014, 0.000);
        // addNode(10,-3.407, -1.046, 0.000);
        // addNode(11,-3.212, -5.912, 0.000);
        // addNode(12,6.736, -5.912, 0.000);
        // addNode(13,6.806, -0.916, 0.000);
        // addNode(14,-2.497, -1.042, 0.000);
        // addNode(15,5.630, -5.536, 0.000);
        // addNode(16,6.651, -3.653, 0.000);

        ////////////////////  7v7 //////////////////////
        // addNode(1, 7.413, 5.873, 1.565);
        // addNode(2, 10.574, 8.712, 1.224);
        // addNode(3, 3.572, 8.419, 2.613);
        // addNode(4, 1.841, 9.794, 2.304);
        // addNode(5, 11.666, 9.827, 1.329);
        // addNode(6, 10.249, 2.959, -0.528);
        // addNode(7, 3.472, 3.155, -2.618);
        // addNode(8, 11.791, 13.232, 1.635);
        // addNode(9, 10.266, 13.840, 2.182);
        // addNode(10, 7.293, 11.793, 2.975);
        // addNode(11, 7.843, 16.136, -3.101);
        // addNode(12, 4.229, 14.377, 1.834);
        // addNode(13, 2.557, 16.344, 1.688);
        // addNode(14, 3.308, 17.697, 1.485);
        // addNode(15, 4.518, 19.295, 0.865);
        // addNode(16, 7.982, 20.364, 0.083);
        // addNode(17, 10.365, 20.025, -0.759);
        // addNode(18, 13.069, 17.983, -0.827);
        // addNode(19, 7.551, 22.791, 1.674);
        // addNode(20, 5.489, 24.585, 2.488);
        // addNode(21, 9.630, 24.650, 0.938);

        //TEST
        addNode(0, 0.5, 0.5, 0);
        addNode(1, 0.5, 0.5, 0);
        addNode(2, 10.574, 8.712, 0);
        addNode(3, 3.572, 8.419, 0);
        addNode(4, 1.841, 9.794, 0);


        for (const auto& edge : edges) {
            addEdge(edge.first, edge.second.first, edge.second.second);
        }
        // 开启主线程
        try {
            std::thread MAINTHREAD(&Graph::main_thread, this);
            MAINTHREAD.detach(); 
        } catch (const std::exception& e) {
            std::cerr << "Failed to create thread: " << e.what() << std::endl;
        }
        start_pose_sub_ = nh_.subscribe("/odom", 10, &Graph::start_pose_callback, this);
        nav_points_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("nav_pose",10);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("path_markers", 10);
        line_points_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("line_pose",10);
        goal_pose_sub_ = nh_.subscribe("/ros1_game_state", 10, &Graph::goal_pose_callback, this);
}

void goal_pose_callback(const std_msgs::Int32::ConstPtr& msg)
{
    end_node.id = msg->data;
}

// 计算两点之间的欧几里得距离
double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// 查找最近的节点
int findNearestNode(const std::vector<Node>& nodes, const geometry_msgs::PoseStamped& robot_pose) {
    double min_distance = std::numeric_limits<double>::max();
    int nearest_node_id = -1;

    for (const auto& node : nodes) {
        double distance = calculateDistance(node.position, robot_pose.pose.position);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_node_id = node.id;
        }
    }
    return nearest_node_id;
}
 
    void start_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        static bool get_path_=false;
        static int current_id;
        robot_pose_.pose.position = msg->pose.pose.position;
        robot_pose_.pose.orientation = msg->pose.pose.orientation;
        start_node.id=findNearestNode(nodes,robot_pose_);

        // static ros::Time last_time = ros::Time::now();
        // ros::Duration interval(8.0); // 8 seconds interval

        // if (ros::Time::now() - last_time >= interval) {
        //     // std::cout << "Enter the target node ID: ";
        //     // std::cin >> end_node.id;
        //     end_node.id = (++end_node.id)%(nodes.size());

        //     // 调用路径搜索方法
        //     findShortestPath(start_node.id, end_node.id);
        //     get_path_ = true;
        //     last_time = ros::Time::now();
        // }

        if (current_id != start_node.id)
        findShortestPath(start_node.id, end_node.id);

        if(start_node.id==end_node.id)
        {
        get_path_=false;
        path_update=false;
        }
        current_id = start_node.id;
    }

    void main_thread() {
        ros::Rate rate(3.0); // 3Hz发布路径point
        while (ros::ok()) {
            static geometry_msgs::PoseStamped current_pose;
            geometry_msgs::PoseStamped pose_msg;
                if (path_update) {
                std::cout << " PUB" << std::endl;   
                // 检查当前节点是否是起始节点
            if (start_node.id == current_path.front()) {
                // 发布到下一个节点的 pose
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.header.frame_id = "map";
                pose_msg.pose.position = nodes[current_path[1] - 1].position; // 获取下一个节点的位置

                publishPointsInFrontOfRobot();
            }
            if(current_pose!=pose_msg)
                nav_points_pub_.publish(pose_msg);

            current_pose=pose_msg;

            if(start_node.id==end_node.id)
            path_update = false; // Reset the flag after publishing
            
            publishPathAsLines(); // 发布路径直线
        }
        rate.sleep(); // Sleep to maintain the rate
    }
}

void publishPathAsLines() {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    // 设置 marker 的属性
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.12; // 线条宽度
    marker.color.a = 1.0; // 不透明度
    marker.color.b = 1.0; // 蓝色
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);        
    for (size_t i = 0; i < current_path.size() - 1; ++i) {
        marker.points.clear();
        nodes[current_path[i] - 1].position.z = 0.1;
        nodes[current_path[i + 1] - 1].position.z = 0.1;

        marker.points.push_back(nodes[current_path[i] - 1].position);
        marker.points.push_back(nodes[current_path[i + 1] - 1].position);
        marker.id = i; // 每个 Marker 的 ID
     
        marker_array.markers.push_back(marker);
    }

    // 发布 MarkerArray
    marker_pub_.publish(marker_array);
}

    void addNode(int id, double x, double y, double z) {
    geometry_msgs::Point position;
    position.x = x;
    position.y = y;
    position.z = z;
    nodes.push_back({id, position, {}});
}

    void addEdge(int from, int to, double weight) {
        for (auto& node : nodes) {
            if (node.id == from) {
                node.edges.push_back({to, weight});
                break;
            }
        }
    }
    
    void publishPointsInFrontOfRobot() {
    if (current_path.size() < 2) return; // 需要至少两个点来确定方向

    int current_node_id = current_path[0];
    int next_node_id = current_path[1];

    geometry_msgs::Point next_pos = nodes[next_node_id].position;

    // 计算方向向量
    double dx = next_pos.x - robot_pose_.pose.position.x;
    double dy = next_pos.y - robot_pose_.pose.position.y;
    double len = std::sqrt(dx * dx + dy * dy); // 向量长度

    // 归一化方向向量
    dx /= len;
    dy /= len;

    static geometry_msgs::Point last_point;
    static bool first_get_point;
    geometry_msgs::Point point;
    geometry_msgs::Point next_point;
    geometry_msgs::Point next_next_point;

    line_pose_msg.header.stamp = ros::Time::now();
    line_pose_msg.header.frame_id = "odom";

    if(len>4.0)
    {
        point.x = robot_pose_.pose.position.x + dx*4.0;
        point.y = robot_pose_.pose.position.y + dy*4.0;
        point.z = 0; 

        line_pose_msg.pose.position = point;
    }

    else if(len<4.0 && next_node_id!=end_node.id)
    {
        int next_next_node_id = current_path[2];
        geometry_msgs::Point next_next_pos = nodes[next_next_node_id - 1].position;
        
        double d2x=next_next_pos.x - next_pos.x;
        double d2y=next_next_pos.y - next_pos.y;
        double len_2 = std::sqrt(d2x * d2x + d2y * d2y); // 向量长度

        // 归一化方向向量
        d2x /= len_2;
        d2y /= len_2;

        next_point.x = next_pos.x + d2x*(4.0-len);
        next_point.y = next_pos.y + d2y*(4.0-len);
        next_point.z = 0; 
        line_pose_msg.pose.position = next_point;
    }

    if(current_node_id==end_node.id)
    {
        line_pose_msg.pose.position = end_node.position;
    }

    if(!first_get_point)
    {
        last_point=line_pose_msg.pose.position;
        first_get_point=true;
        line_points_pub_.publish(line_pose_msg);
    }

        if(calculateDistance(last_point,point)>0.5 && first_get_point)
        {
        last_point=line_pose_msg.pose.position;
        line_points_pub_.publish(line_pose_msg);
        }
        else
        line_pose_msg.pose.position=last_point;

}

void findShortestPath(int start, int end) {
    std::vector<double> dist(nodes.size(), INT_MAX);
    std::vector<int> prev(nodes.size(), -1);
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

    dist[start - 1] = 0;
    pq.push({0, start - 1});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (dist[u] > dist[pq.top().second]) continue;

        for (const auto& edge : nodes[u].edges) {
            int v = edge.to - 1;
            if (dist[u] + edge.weight < dist[v]) {
                dist[v] = dist[u] + edge.weight;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    if (dist[end - 1] == INT_MAX) {
        std::cout << "No path found between node " << start << " and node " << end << std::endl;
        return;
    }

    std::cout << "Shortest path from node " << start << " to node " << end << " is: ";
    printPath(prev, start - 1, end - 1);
    std::cout << " with total weight " << dist[end - 1] << std::endl;   

    // Reconstruct the path
    current_path.clear();
    int current = end - 1;
    while (current != -1) {
        current_path.push_back(current + 1); // Store the node ID
        current = prev[current];
    }
    std::reverse(current_path.begin(), current_path.end()); // Reverse to start from the beginning

    path_update = true; // Mark that a new path has been found
}

private:
    // 递归打印路径
    void printPath(const std::vector<int>& prev, int start, int end) {
        if (end == start) {
            std::cout << start + 1;
            return;
        }
        printPath(prev, start, prev[end]);
        std::cout << " -> " << end + 1;
    }

    std::vector<std::pair<int, std::pair<int, double>>> edges = {
        //3v3
        // {1, {6, 1}}, {1, {5, 1}}, {1, {10, 1}}, {1, {2, 1}},{1, {9, 1.5}},{1, {13, 2.5}},
        // {2, {11, 1}}, {2, {10, 1}}, {2, {3, 1}}, {2, {1, 1}},
        // {3, {11, 1}}, {3, {4, 1}}, {3, {5, 1}}, {3, {2, 1}},{3, {7, 1.5}},{3, {12, 2.5}},{3, {9, 3}},
        // {4, {7, 1}}, {4, {5, 1}}, {4, {3, 1}},{4, {12 , 2}},
        // {5, {4, 1}}, {5, {6, 1}}, {5, {7, 1.5}}, {5, {3, 1.5}}, {5, {1, 1.5}}, {5, {9, 1.5}},
        // {6, {1, 1}}, {6, {10, 2.5}}, {6, {9, 1}},{6, {13, 2}},
        // {7, {4, 1}}, {7, {12, 1.5}}, {7, {8, 1}},{7, {3, 1.5}},{7, {11, 2.5}},{7, {1, 2.7}},
        // {8, {12, 1}}, {8, {13, 1}}, {8, {9, 1}}, {8, {7, 1.5}},
        // {9, {8, 1}}, {9, {13, 1}}, {9, {6, 1}}, {9, {5, 1.5}}, {9, {12, 2.5}},
        // {10, {2, 1}}, {10, {11, 1.3}}, {10, {1, 1.5}},{10, {13, 3}},{10, {3, 1.7}},
        // {11, {2, 1}}, {11, {3, 1}}, {11, {10, 1.3}},{11, {12, 3}},{11, {1, 1.7}},
        // {12, {8, 1}}, {12, {7, 2}}, {12, {13, 1.3}},{12, {11, 3}},{12, {9, 1.7}},
        // {13, {12, 1.3}}, {13, {8, 1}}, {13, {9, 1}},{13, {10, 3}},{13, {9, 2}},{13, {7, 1.7}},
        // {14, {10, 0.5}}, {14, {1, 0.5}}, {14, {2, 0.7}}, {14, {6, 1.5}}, {14, {9, 2.5}}, {14, {13, 3.5}},
        // {15, {7, 0.5}}, {15, {12, 0.5}}, {15, {8, 0.7}}, {15, {13, 2}}, {15, {3, 2.5}}, {15, {11, 3.5}},{15, {16, 1}},
        // {16, {12, 1}}, {16, {13, 1}}, {16, {8, 0.5}}, {16, {15, 1}},{16, {9, 1}},

        //7v7
        // {1, {3, 1}}, {1, {2, 1}}, {1, {5, 1.5}}, {1, {3, 1}},{1, {4, 1.5}},{1, {7, 1}},{1, {6, 1}},
        // {2, {1, 1}}, {2, {3, 1.5}}, {2, {5, 1}}, {2, {4, 2}},{2, {6, 1.5}},{2, {7, 2}},{2, {8, 2}},
        // {3, {2, 1.5}}, {3, {1, 1}}, {3, {7, 1.5}}, {3, {4, 1}},{3, {5, 2.5}},{3, {6, 2}},
        // {4, {3, 1}}, {4, {3, 1.5}}, {4, {2, 1.5}},{4, {6 , 2}},{4, {5 , 2.5}},
        // {5, {4, 2.5}}, {5, {2, 1}}, {5, {1, 2}}, {5, {7, 2.5}}, {5, {6, 2.5}}, {5, {8, 1}},{5, {9, 1.5}},
        // {6, {1, 1}}, {6, {2, 1.5}}, {6, {7, 1.5}},{6, {3, 2}},{6, {4, 2.5}},{6, {8, 3}},
        // {7, {1, 1}}, {7, {6, 1.5}}, {7, {3, 1.5}},{7, {5, 2.5}},{7, {2, 2}},
        // {8, {5, 0.5}}, {8, {9, 0.5}}, {8, {2, 1.5}}, {8, {1, 3}},{8, {11, 2}},{8, {10, 2.5}},{8, {12, 3}},
        // {9, {8, 0.5}}, {9, {10, 1.5}}, {9, {5, 1}}, {9, {11, 1}}, {9, {12, 2.5}},
        // {10, {12, 1}}, {10, {9, 1}}, {10, {11, 1.5}},{10, {8, 1.3}},{10, {13, 1.5}},
        // {11, {12, 1}}, {11, {9, 1}}, {11, {10, 1.5}},{11, {13, 1.5}},
        // {12, {13, 0.5}}, {12, {14, 1}},{12, {15, 2}},{12, {10, 1}},{12, {11, 1}},{12, {9, 2.5}},
        // {13, {14, 0.5}}, {13, {15, 1}}, {13, {16, 1.5}},{13, {19, 2.5}},{13, {21, 3}},{13, {12, 0.5}},
        // {14, {15, 0.5}}, {14, {16, 1}}, {14, {19, 1.5}},{14, {21, 2.5}}, {14, {17, 3}},{14, {13, 0.5}},
        // {15, {16, 0.5}}, {15, {19, 1}}, {15, {17, 1.5}}, {15, {18, 2}}, {15, {21, 2}}, {15, {20, 1.7}},{15, {14, 0.5}},
        // {16, {19, 0.5}}, {16, {17, 1}}, {16, {18, 1}}, {16, {20, 1.4}},{16, {21, 1.4}},
        // {17, {18, 0.5}}, {17, {16, 0.5}}, {17, {19, 1}},{17, {20, 2}},{17, {21, 1.5}},{17, {15, 1.5}},
        // {18, {21, 2.5}}, {18, {20, 2}}, {18, {17, 1}},{18, {16, 2}},{18, {19, 2}},   
        // {19, {20, 1}},  {19, {21, 1}}, {19, {15, 1}}, {19, {16, 0.5}}, {19, {17, 1}}, 
        // {20, {19, 1}}, {20, {21, 1.5}}, {20, {16, 1.5}}, {20, {17, 2}}, 
        // {21, {19, 1}}, {21, {20, 1.5}}, {21, {15, 2}}, {21, {16, 1.7}}, {21, {17, 2}}, 

        //TEST
        {0, {1, 1}}, {0, {2, 1}},{0, {3, 1}},
        {1, {2, 1}}, {1, {3, 1}},
        {2, {1, 1}}, {2, {4, 1}},
        {3, {1, 1}}, {3, {4, 1}},
        {4, {2, 1}}, {4, {3, 1}},

        
    };

    ros::Subscriber start_pose_sub_;
    ros::Subscriber goal_pose_sub_; 
    ros::Publisher marker_pub_;
    ros::Publisher nav_points_pub_; 
    ros::Publisher line_points_pub_; 
    std::vector<int> current_path; // 存储当前路径的节点 ID
    std::vector<Node> nodes;
    ros::NodeHandle nh_;
    geometry_msgs::PoseStamped robot_pose_;
    geometry_msgs::PoseStamped line_pose_msg;
    Node start_node,end_node;
    bool path_update;
    int start,end;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Graph");

    Graph graph;

    ros::spin();
    return 0;
}
