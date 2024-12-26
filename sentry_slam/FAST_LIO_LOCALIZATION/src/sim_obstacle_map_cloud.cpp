#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <pcl/registration/gicp.h>
#include <pcl_ros/transforms.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <tf2_ros/transform_broadcaster.h>
#include <ctime>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <std_msgs/Bool.h> 
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <pcl/segmentation/extract_clusters.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointField.h>
#include <vector>

class sim_Obstacle_map_cloud
{
private:
    /* data */
    bool get_lidar_cloud_,get_map_;
    ros::NodeHandle nh_;
    sensor_msgs::PointCloud2 world_obstacle_msg;
    sensor_msgs::PointCloud2 prior_map_msg;
    ros::Publisher world_Obstacle_pub_;
    ros::Subscriber grid_map_sub_;
    ros::Subscriber point_cloud_cloud_sub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_Obstacle_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr prior_map_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_yaml_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_map_;
public:
    sim_Obstacle_map_cloud();
    ~sim_Obstacle_map_cloud();
    void main_thread();
    void pointcloudmapcallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
};

sim_Obstacle_map_cloud::sim_Obstacle_map_cloud():
nh_("~"),get_lidar_cloud_(false),get_map_(false),
prior_map_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
map_yaml_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
world_Obstacle_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
point_cloud_map_(new pcl::PointCloud<pcl::PointXYZ>)
{   
    grid_map_sub_ = nh_.subscribe("/prior_map",  1, &sim_Obstacle_map_cloud::gridmapCallback, this);
    point_cloud_cloud_sub_ = nh_.subscribe("/processed_cloud",1,&sim_Obstacle_map_cloud::pointcloudmapcallback, this);
    world_Obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("world_obstacle_cloud", 1);
}

void sim_Obstacle_map_cloud::pointcloudmapcallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, *point_cloud_map_);
    // ROS_WARN("LIDAR");    
    get_lidar_cloud_=true;
}

void sim_Obstacle_map_cloud::main_thread()
{
    ros::Rate rate(30.0); // 30Hz
    while(ros::ok())
    {
    if(get_lidar_cloud_ && get_map_)
    {
        // ROS_WARN("go map");    
        map_yaml_cloud_->clear();
        *map_yaml_cloud_ = *world_Obstacle_cloud_+*point_cloud_map_;
        pcl::toROSMsg(*map_yaml_cloud_, world_obstacle_msg);
        world_obstacle_msg.header.stamp = ros::Time::now();
        world_obstacle_msg.header.frame_id = "map";  // 坐标系
        world_Obstacle_pub_.publish(world_obstacle_msg);
        rate.sleep();
    }
    }
}

sim_Obstacle_map_cloud::~sim_Obstacle_map_cloud()
{

}

void sim_Obstacle_map_cloud::gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) // 获取地图信息
{
        // 获取地图信息
        float resolution = msg->info.resolution;
        float origin_x = msg->info.origin.position.x;
        float origin_y = msg->info.origin.position.y;
        int width = msg->info.width;
        int height = msg->info.height;

        const std::vector<int8_t>& data = msg->data;

        // 清空点云数据
        world_Obstacle_cloud_->points.clear();
        world_Obstacle_cloud_->width = 0;
        world_Obstacle_cloud_->height = 2;  

        // 遍历地图数据，找到值为100的点（障碍物）
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                if (data[idx] == 100) {  // 如果是障碍物
                    // 计算点的世界坐标
                    pcl::PointXYZ point;
                    point.x = origin_x + x * resolution;
                    point.y = origin_y + y * resolution;
                    point.z = 0.0;  // 假设障碍物的高度为0
                    world_Obstacle_cloud_->points.push_back(point); 

                    point.z = 0.05;  
                    world_Obstacle_cloud_->points.push_back(point); 
                    world_Obstacle_cloud_->width += 1;
                }
            }
        }

        // 如果找到了障碍物，则发布点云
        if (world_Obstacle_cloud_->width > 0) {
            ROS_INFO("get map cloud");
            get_map_=true;
                try {
                    std::thread MAINTHREAD(&sim_Obstacle_map_cloud::main_thread, this);
                    MAINTHREAD.detach(); 
                } catch (const std::exception& e) {
                    std::cerr << "Failed to create thread: " << e.what() << std::endl;
                }
        } else {
            ROS_WARN("No obstacles found in the map!");
        }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sim_Obstacle_map_cloud");
    ROS_INFO("begin");
    sim_Obstacle_map_cloud sim_obstacle_map_cloud;
    ros::spin();
    return 0;
}