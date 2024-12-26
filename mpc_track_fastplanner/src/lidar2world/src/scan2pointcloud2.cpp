#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types_conversion.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class LaserScanToPointCloud {
public:
    LaserScanToPointCloud() {
        // 初始化ROS节点句柄
        ros::NodeHandle node;

        // 订阅LaserScan数据
        scan_sub_ = node.subscribe<sensor_msgs::LaserScan>("/scan", 100, &LaserScanToPointCloud::scanCallback, this);

        // 发布PointCloud2数据
        point_cloud_pub_ = node.advertise<sensor_msgs::PointCloud2>("/point_cloud_map", 100);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        sensor_msgs::PointCloud2 cloud;
        laser_geometry::LaserProjection projector_;
        projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tf_listener_);
        cloud.header.frame_id="odom";
        point_cloud_pub_.publish(cloud);
    }

private:
    ros::Subscriber scan_sub_;
    ros::Publisher point_cloud_pub_;
    tf::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_to_point_cloud");
    LaserScanToPointCloud converter;
    ros::spin();
    return 0;
}