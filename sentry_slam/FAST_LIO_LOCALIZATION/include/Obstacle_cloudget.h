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

class Obstacle_cloud_get
{
    public:
    Obstacle_cloud_get();
    ~Obstacle_cloud_get();
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
    void mappointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
    void PointCloudObstacleRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_map_msg, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_msg, 
                              double Distance_Threshold);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void transformCallback(const geometry_msgs::TransformStampedConstPtr& input);
    void Init_params();
    void gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    private:
    // 创建RadiusOutlierRemoval对象
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    bool getmap_;
    bool get_transform;
    float clear_distance_x;
    float clear_distance_y;
    float clear_distance_z;
    float filter_search_radius_;
    float kdtree_search_radius_;
    clock_t start_time,end_time;
    ros::Publisher pub_prior_map_;
    ros::NodeHandle nh_;
    ros::Publisher removal_pointcloud_publisher_;
    ros::Publisher incoming_pub_;
    ros::Publisher Obstacle_cloud_pub_;
    ros::Publisher Obstacle_cloud_pub_odom_;
    ros::Publisher world_obstacle_pub_;
    ros::Subscriber map_to_odom_sub_;
    ros::Subscriber incoming_cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber prior_map_sub_;
    ros::Subscriber grid_map_sub_;

    sensor_msgs::PointCloud2 prior_map_msg;
    sensor_msgs::PointCloud2 incoming_cloud_msg;
    sensor_msgs::PointCloud2 cloud_removed_msg;
    sensor_msgs::PointCloud2 obstacle_cloud_msg;
    sensor_msgs::PointCloud2 world_obstacle_msg;
    sensor_msgs::PointCloud2 fast_planner_obstacle_msg;

    Eigen::Matrix4f transform ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr incoming_cloud_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget;
	pcl::PointCloud<pcl::PointXYZ>::Ptr Obstacle_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_Obstacle_cloud_;
    Eigen::Vector3f radar_position; // 雷达位置
    Eigen::Matrix4f icp_transform_; 
};
