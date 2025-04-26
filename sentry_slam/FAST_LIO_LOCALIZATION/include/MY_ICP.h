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
#include <pcl/registration/ndt.h>
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
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"

using namespace pcl;
using namespace std;

class MY_ICP {
public:
    MY_ICP();
    ~MY_ICP();
    void publish_map_msg();
    void Initparams();
    void publish_map_msg_thread();
    void perform_icp_thread();
    void get_lidar_cloud();
    void findBestYawAngle_thread();
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
    void timerCallback(const ros::TimerEvent& event);
    void performRelocalization(const Eigen::Matrix4f& initial_pose);
    void publishTransform(const Eigen::Matrix4f& transform);
    void findBestYawAngle();
    void pub_map_to_camera_init();
    void PointCloudObstacleRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_map_msg, 
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_msg, 
                                    double Distance_Threshold);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPointsByDistance(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
        float distance_threshold);
    void mappointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr prior_map_deal(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_node_;          // ros中的私有句柄,加载参数服务器
    pcl::PointCloud<pcl::PointXYZ>::Ptr prior_map_; // 先验地图点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_lidar_cloud_; // 先验地图旋转地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr incoming_cloud_; // 实时雷达点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_; // 存储后读取的雷达点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_; // 转换之后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_; // 雷达融合的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_; // 雷达融合的点云
    
    std::mutex cloud_mutex_;
    sensor_msgs::PointCloud2 prior_map_msg;
    sensor_msgs::PointCloud2 rotated_lidar_map_msg;
    sensor_msgs::PointCloud2 incoming_cloud_msg;
    sensor_msgs::PointCloud2 transformed_cloud_msg_; //转换之后的点云msg
    sensor_msgs::PointCloud2 global_pointcloud_msg_; //转换之后的点云msg
    sensor_msgs::PointCloud2 cloud_removed_msg;
    sensor_msgs::PointCloud2 local_pointcloud_msg;
    std_msgs::Bool move_base_start_msg;
    std_msgs::Bool fast_planner_start_msg;
    pcl::GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> gicp;
    pcl::NormalDistributionsTransform<PointXYZ, PointXYZ> ndt;
    pcl::GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> icp;

    std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;
    
    std::mutex global_mutex_;
    Eigen::Matrix4f initial_pose;
    Eigen::Matrix4f icp_transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    std::mutex icp_thread_mutex;  // protects icp
    std::mutex findbestyaw_thread_mutex;  // protects findbestyaw
    float best_yaw_angle_; // 存储最佳旋转角度
    float icp_correct;
    float local_pointcloud_x_;
    float local_pointcloud_y_;
    float local_pointcloud_z_;
    float local_ground_pointcloud_z_;
    bool need_relocalization; //如果需要进行重定位，则为true
    bool start_find;
    bool find_times;
    bool transformed;
    bool saved_PCD;
    bool purely_localization_;
    bool finish_cut_cloud_;
    int find_min_angle;
    float min_get_score;
    float big_jump_yaw_score;
    float small_jump_yaw_score;
    float find_best_yaw_icp_Iterations;
    float icp_Iterations;
    float re_icp_score;
    float save_lidar_times;
    float remove_cloud_length;
    float clear_icp_global_costmap;
    int icp_transform_update;

    int icp_over;
    int icp_start;
    bool restart;
    std_msgs::Bool restart_msg;
    std_msgs::Float32 icp_msg;
    Eigen::Vector3f radar_position; // 雷达位置
    float clear_distance_x;
    float clear_distance_y;
    float clear_distance_z;
    ros::Publisher pub_prior_map;
    ros::Publisher restart_all_pub;
    ros::Publisher pub_rotated_lidar_cloud;
    ros::Publisher pub_transformed_cloud;
    ros::Publisher icp_pub_;
    ros::Publisher pub_incoming_cloud;
    ros::Publisher map_to_odom_pub;
    ros::Publisher removal_pointcloud_publisher_;
    ros::Publisher move_base_start_pub_;
    ros::Publisher local_pointcloud_pub_;
    ros::Publisher fast_planner_pub_;
    ros::Publisher initial_pose_pub;
    ros::Subscriber pointcloud_sub;
    ros::Subscriber map_sub_;
    ros::Subscriber odom_sub_;
    std::thread publishThread_;
};
