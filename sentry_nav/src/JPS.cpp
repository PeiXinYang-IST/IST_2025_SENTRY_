#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <thread>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <functional>

struct Node {
    int row, col;
    double g, h;
    Node* parent;
    std::pair<int, int> direction;

    Node(int r, int c) : row(r), col(c), g(0), h(0), parent(nullptr), direction(0,0) {}
    double f() const { return g + h; }
    bool operator<(const Node& other) const { return this->f() > other.f(); }
};

class JPSPlanner {
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    
    std::vector<std::vector<int>> grid_;
    double resolution_;
    double origin_x_, origin_y_;
    bool map_ready_ = false;
    
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped goal_pose_;
    geometry_msgs::PoseStamped last_goal_pose_;
    std::mutex data_mutex_;
    bool pose_ready_ = false;
    bool goal_ready_ = false;

    // JPS核心算法实现
    bool isValid(int row, int col) const {
        return row >= 0 && row < grid_.size() && 
               col >= 0 && col < grid_[0].size() && 
               grid_[row][col] == 0;
    }

    double heuristic(const Node* a, const Node* b) const {
        return std::abs(a->col - b->col) + std::abs(a->row - b->row);
    }

    double distance(const Node* a, const Node* b) const {
        int dx = std::abs(a->col - b->col);
        int dy = std::abs(a->row - b->row);
        return (dx == dy) ? 1.414 * dx : (dx + dy);
    }

    Node* jump(Node* current, std::pair<int, int> direction, const Node* goal) {
        int next_row = current->row + direction.first;
        int next_col = current->col + direction.second;

        if (!isValid(next_row, next_col)) return nullptr;

        Node* next_node = new Node(next_row, next_col);
        next_node->parent = current;
        next_node->direction = direction;

        if (next_row == goal->row && next_col == goal->col) 
            return next_node;

        // 水平/垂直方向检查
        if (direction.first == 0 || direction.second == 0) {
            if (direction.first == 0) { // 水平
                if ((isValid(next_row+1, next_col-1) && !isValid(next_row+1, next_col)) ||
                    (isValid(next_row-1, next_col-1) && !isValid(next_row-1, next_col))) {
                    return next_node;
                }
            } else { // 垂直
                if ((isValid(next_row-1, next_col+1) && !isValid(next_row, next_col+1)) ||
                    (isValid(next_row-1, next_col-1) && !isValid(next_row, next_col-1))) {
                    return next_node;
                }
            }
        } 
        // 对角线方向检查
        else {
            if (jump(next_node, {direction.first, 0}, goal) || 
                jump(next_node, {0, direction.second}, goal)) {
                return next_node;
            }
        }

        return jump(next_node, direction, goal);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        resolution_ = msg->info.resolution;
        origin_x_ = msg->info.origin.position.x;
        origin_y_ = msg->info.origin.position.y;

        grid_.resize(msg->info.height);
        for (int i = 0; i < msg->info.height; ++i) {
            grid_[i].resize(msg->info.width);
            for (int j = 0; j < msg->info.width; ++j) {
                grid_[i][j] = msg->data[i * msg->info.width + j] > 65 ? 1 : 0;
            }
        }
        map_ready_ = true;
        ROS_INFO("Map initialized (%.2fx%.2fm)", 
                grid_[0].size()*resolution_, 
                grid_.size()*resolution_);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_pose_.header = msg->header;
        current_pose_.pose = msg->pose.pose;
        pose_ready_ = true;
    }
    
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        goal_pose_ = *msg;
        goal_ready_ = true;
        ROS_INFO("New goal received at (%.2f, %.2f)", msg->pose.position.x, msg->pose.position.y);
    }

public:
    JPSPlanner() : goal_ready_(false) {
        map_sub_ = nh_.subscribe("/prior_map", 1, &JPSPlanner::mapCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &JPSPlanner::odomCallback, this);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &JPSPlanner::goalCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/jps_path", 1);
    }

    void planningLoop() {
        ros::Rate rate(10);
        while (ros::ok()) {
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (!map_ready_ || !pose_ready_ || !goal_ready_) {
                    if (!goal_ready_) {
                        ROS_WARN_THROTTLE(1, "Waiting for goal...");
                    } else {
                        ROS_WARN_THROTTLE(1, "Waiting for map or pose data...");
                    }
                    continue;
                }

                auto [start_row, start_col] = worldToGrid(
                    current_pose_.pose.position.x, 
                    current_pose_.pose.position.y);
                
                auto [goal_row, goal_col] = worldToGrid(
                    goal_pose_.pose.position.x,
                    goal_pose_.pose.position.y);

                if (!isValid(start_row, start_col)) {
                    ROS_WARN_THROTTLE(1, "Start position occupied!");
                    continue;
                }

                if (!isValid(goal_row, goal_col)) {
                    ROS_WARN_THROTTLE(1, "Goal position occupied!");
                    continue;
                }

                std::vector<std::pair<int, int>> path = findPath(
                    Node(start_row, start_col),
                    Node(goal_row, goal_col)
                );

                if (!path.empty()) {
                    publishPath(path);
                    ROS_WARN("Path found!");
                }
            }
            rate.sleep();
        }
    }

private:
    std::pair<int, int> worldToGrid(double x, double y) const {
        return {
            static_cast<int>((y - origin_y_) / resolution_),
            static_cast<int>((x - origin_x_) / resolution_)
        };
    }

    std::vector<std::pair<int, int>> findPath(const Node& start, const Node& goal) {
        std::priority_queue<Node*, std::vector<Node*>, 
            std::function<bool(Node*, Node*)>> open(
                [](Node* a, Node* b){ return a->f() > b->f(); });
        
        std::unordered_map<int, Node*> closed;
        auto hash = [](int r, int c){ return r * 10000 + c; };

        Node* start_node = new Node(start.row, start.col);
        start_node->h = heuristic(start_node, &goal);
        open.push(start_node);

        while (!open.empty()) {
            Node* current = open.top();
            open.pop();

            int current_hash = hash(current->row, current->col);
            if (closed.count(current_hash)) {
                delete current;
                continue;
            }
            closed[current_hash] = current;

            if (current->row == goal.row && current->col == goal.col) {
                return reconstructPath(current);
            }

            const std::vector<std::pair<int, int>> dirs = {
                {0,1}, {0,-1}, {1,0}, {-1,0}, {1,1}, {-1,1}, {1,-1}, {-1,-1}
            };

            for (const auto& dir : dirs) {
                if (Node* jump_node = jump(current, dir, &goal)) {
                    int node_hash = hash(jump_node->row, jump_node->col);
                    if (!closed.count(node_hash)) {
                        jump_node->g = current->g + distance(current, jump_node);
                        jump_node->h = heuristic(jump_node, &goal);
                        open.push(jump_node);
                    }
                }
            }
        }

        ROS_WARN("No path found!");
        return {};
    }

    std::vector<std::pair<int, int>> reconstructPath(Node* node) {
        std::vector<std::pair<int, int>> path;
        while (node) {
            path.emplace_back(node->row, node->col);
            Node* prev = node->parent;
            delete node;
            node = prev;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    void publishPath(const std::vector<std::pair<int, int>>& path) {
        nav_msgs::Path msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "odom"; // 确保与地图坐标系一致

        for (const auto& p : path) {
            geometry_msgs::PoseStamped pose;
            pose.header = msg.header;
            pose.pose.position.x = origin_x_ + (p.second + 0.5) * resolution_;
            pose.pose.position.y = origin_y_ + (p.first + 0.5) * resolution_;
            msg.poses.push_back(pose);
        }
        path_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "jps_planner");
    JPSPlanner planner;
    
    std::thread planning_thread([&planner]() {
        planner.planningLoop();
    });

    ros::spin();
    planning_thread.join();
    return 0;
}