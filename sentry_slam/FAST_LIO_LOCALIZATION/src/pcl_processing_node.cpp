#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <nav_msgs/Odometry.h>

// 全局变量
ros::Publisher pub;
ros::Subscriber sub;
ros::Subscriber odom_sub;
ros::Time last_time;
ros::Time current_time;
int count = 0; // 计数器
ros::Time last_second_time; // 上一次计算频率的时间
Eigen::Vector3f radar_position; // 雷达位置

double clear_distance_x;
double clear_distance_y;
double clear_distance_z;

float clear_z;
float clear_x;
float clear_y;

// 里程计数据的回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    radar_position = Eigen::Vector3f(odom_msg->pose.pose.position.x,
                                     odom_msg->pose.pose.position.y,
                                     odom_msg->pose.pose.position.z);
    clear_distance_x = radar_position[0];
    clear_distance_y = radar_position[1];
    clear_distance_z = radar_position[2];
}

// 回调函数
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg) {
    // 更新当前时间
    current_time = ros::Time::now();

    // 累加计数器
    count++;

    // 如果已经过了1秒，计算频率并重置计数器
    if (current_time - last_second_time > ros::Duration(1.0)) {
        double frequency = count; // 频率为每秒计数器的值
        last_second_time = current_time; // 更新上一次计算频率的时间
        count = 0; // 重置计数器
    }

    // 将sensor_msgs::PointCloud2转换为pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud_msg, *cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_cylinder(new pcl::PointCloud<pcl::PointXYZ>);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(clear_distance_x-clear_x, clear_distance_x+clear_x); // x轴半径限制
    pass.filter(*cloud_filtered_cylinder);

    pass.setInputCloud(cloud_filtered_cylinder);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(clear_distance_y-clear_y, clear_distance_y+clear_y); // y轴半径限制
    pass.filter(*cloud_filtered_cylinder);


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_cylinder_removed_xy(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_cylinder_removed_z_1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_cylinder_removed_z_2(new pcl::PointCloud<pcl::PointXYZ>);

// 从原始点云中删除圆柱体内的点
std::vector<int> indices_to_remove;

for (size_t i = 0; i < cloud_filtered_cylinder->points.size(); ++i) {
    // 查找原始点云中与滤波后的点相匹配的点，并标记它们的索引
    pcl::PointXYZ point = cloud_filtered_cylinder->points[i];
    for (size_t j = 0; j < cloud->points.size(); ++j) {
        if (cloud->points[j].x == point.x &&
            cloud->points[j].y == point.y &&
            cloud->points[j].z == point.z) {
            indices_to_remove.push_back(j); // 记录该点的索引
        }
    }
}

// 从原始点云中删除这些点
for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (std::find(indices_to_remove.begin(), indices_to_remove.end(), i) == indices_to_remove.end()) {
        cloud_filtered_cylinder_removed_xy->points.push_back(cloud->points[i]);
    }
}

    // 滤除z轴恰当位置的点云
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_filtered_cylinder_removed_xy);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-std::numeric_limits<float>::max(),clear_distance_z+clear_z); // 低于雷达z坐标的点云滤除
    pass_z.filter(*cloud_filtered_cylinder_removed_xy); // 更新


for (size_t i = 0; i < cloud_filtered_cylinder_removed_xy->points.size(); ++i) {
    // 查找原始点云中与滤波后的点相匹配的点，并标记它们的索引
    pcl::PointXYZ point = cloud_filtered_cylinder_removed_xy->points[i];
    for (size_t j = 0; j < cloud->points.size(); ++j) {
        if (cloud->points[j].x == point.x &&
            cloud->points[j].y == point.y &&
            cloud->points[j].z == point.z) {
            indices_to_remove.push_back(j); // 记录该点的索引
        }
    }
}

// 从原始点云中删除这些点
for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (std::find(indices_to_remove.begin(), indices_to_remove.end(), i) == indices_to_remove.end()) {
        cloud_filtered_cylinder_removed_z_1->points.push_back(cloud->points[i]);
    }
}

    // 滤除z轴恰当位置的点云
    pass_z.setInputCloud(cloud_filtered_cylinder_removed_z_1);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(clear_distance_z+3.0,std::numeric_limits<float>::max()); // 距离雷达z坐标过高的点云滤除
    pass_z.filter(*cloud_filtered_cylinder_removed_z_1); // 更新


for (size_t i = 0; i < cloud_filtered_cylinder_removed_z_1->points.size(); ++i) {
    // 查找原始点云中与滤波后的点相匹配的点，并标记它们的索引
    pcl::PointXYZ point = cloud_filtered_cylinder_removed_z_1->points[i];
    for (size_t j = 0; j < cloud->points.size(); ++j) {
        if (cloud->points[j].x == point.x &&
            cloud->points[j].y == point.y &&
            cloud->points[j].z == point.z) {
            indices_to_remove.push_back(j); // 记录该点的索引
        }
    }
}

// 从原始点云中删除这些点
for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (std::find(indices_to_remove.begin(), indices_to_remove.end(), i) == indices_to_remove.end()) {
        cloud_filtered_cylinder_removed_z_2->points.push_back(cloud->points[i]);
    }
}

    // 创建StatisticalOutlierRemoval滤波器对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_filtered_cylinder_removed_z_2);
    sor.setMeanK(15); // 设置每个点的邻近点数
    sor.setStddevMulThresh(1.0); // 设置标准偏差乘数阈值
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_sor(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_filtered_sor);

    // 创建VoxelGrid滤波器对象
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud_filtered_sor);
    voxel_grid.setLeafSize(0.03f, 0.03f, 0.03f); // 单位：m
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.filter(*cloud_filtered_voxel);

    // 将pcl::PointCloud<pcl::PointXYZ>转换回sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*cloud_filtered_voxel, output_cloud_msg);
    output_cloud_msg.header.frame_id = "camera_init";
    output_cloud_msg.header.stamp = ros::Time::now();
    // 发布处理后的点云
    pub.publish(output_cloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_processing_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_node_("~"); // 使用私有命名空间
    private_node_.param<float>("clear_z",clear_z,0.70); 
    private_node_.param<float>("clear_x",clear_x,0.50); 
    private_node_.param<float>("clear_y",clear_y,0.10); 

    // 创建Subscriber来订阅原始点云
    sub = nh.subscribe("cloud_registered", 1, cloudCallback);
    odom_sub = nh.subscribe("odom", 1, odomCallback);
    // 创建Publisher来发布处理后的点云
    pub = nh.advertise<sensor_msgs::PointCloud2>("processed_cloud", 1);

    last_second_time = ros::Time::now(); // 初始化上一次计算频率的时间

    ros::spin();
    return 0;
}