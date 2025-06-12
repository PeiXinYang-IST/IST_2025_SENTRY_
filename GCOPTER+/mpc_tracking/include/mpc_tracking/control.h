#ifndef CONTROL_H
#define CONTROL_H

#include <gcopter/CoeffRow.h>
#include <gcopter/CoeffMatrix.h>
#include <gcopter/PolyTrajectory.h>
#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <Eigen/Dense>
#include <thread>
#include <cppad/ipopt/solve.hpp>
#include <mpc_tracking/mpc.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <sentry_serial/navigation.h>
#include "std_msgs/Empty.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

unique_ptr<Mpc> mpc_ptr;

struct MPC_Config
{
    std::string trajtopic;

    MPC_Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("trajtopic", trajtopic);
    }
};

class controller
{
private:
    const int N = 30;
    const double dt = 0.1;
    vector<double> weights = {10,10,1,1,1,0}; //Q,R
    /* data */
    MPC_Config config;
    ros::Subscriber trajsub;
    ros::Subscriber odom_sub;
    ros::Publisher predict_path_pub;
    ros::Publisher navigation_pub;
    ros::Publisher predict_pos_pub_;
    ros::Publisher desired_pos_pub_;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber map_to_odom_sub;
    ros::Subscriber global_path_sub;
    ros::Subscriber replan_sub;
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    std::vector<Eigen::Vector3d> dijstra_pos;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory_cloud;
    std::vector<double> trajectory_times;  // 新增轨迹时间存储
    ros::NodeHandle nh;
    Trajectory<5> currentTraj_;
    ros::Time trajStartTime_; // 轨迹开始时间
    ros::Timer control_cmd_pub;
    bool get_traj;
    Eigen::Vector3d current_state;
    Eigen::Vector3d current_state_vel;
    std::mutex traj_mutex_;
    unique_ptr<Mpc> mpc_ptr;
    nav_msgs::Path predict_path;
    sentry_serial::navigation navigation;
    bool get_global_path = false;
    double traj_duration = 0.0;
public:
    controller(const MPC_Config &conf,ros::NodeHandle &nh_)
    : config(conf),
      nh(nh_),
      get_traj(false)
{
    trajsub = nh.subscribe("trajectory", 1, &controller::trajCallBack, this,
                              ros::TransportHints().tcpNoDelay());
    control_cmd_pub = nh.createTimer(
            ros::Duration(0.05), 
            &controller::publish_control_cmd, 
            this
        );
    odom_sub = nh.subscribe("/odom", 1, &controller::odomCallback,this);
    predict_path_pub = nh.advertise<nav_msgs::Path>("/predict_path", 1);
    predict_pos_pub_ = nh.advertise<visualization_msgs::MarkerArray>("mpc_pos", 100);
    desired_pos_pub_ = nh.advertise<visualization_msgs::MarkerArray>("desired_state", 100);
    map_to_odom_sub = nh.subscribe("MY_ICP/map_to_odom", 1, &controller::mapToOdomCallback, this);
    navigation_pub = nh.advertise<sentry_serial::navigation>("navigation",10);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mpc_cmd_vel", 10);
    global_path_sub = nh.subscribe("/move_base1/NavfnROS/plan", 10, &controller::globalPathCallback, this);
    replan_sub = nh.subscribe("/mpc_path_update",1, &controller::replanCallback, this);
    trajectory_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    mpc_ptr.reset(new Mpc());
}

void replanCallback(const std_msgs::Empty::ConstPtr &msg)
{
  const double time_out = 0.01;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - trajStartTime_).toSec() + time_out;
//    traj_duration = min(t_stop, traj_duration);
}

void globalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    if (msg->poses.size() > 0)
    {
        dijstra_pos.clear();
        for (size_t i = 0; i < msg->poses.size(); i += 5)
        {
            const auto &pose = msg->poses[i];
            Eigen::Vector3d point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            dijstra_pos.push_back(point);
        }
        get_global_path = true;
    }
}

void mapToOdomCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
        // 正确解析TF变换
        tf::Transform tf_transform;
        tf::transformMsgToTF(msg->transform, tf_transform);
        
        // 转换为Eigen Affine3d
        transform.matrix() = Eigen::Matrix4d::Identity();
        transform.translation() << msg->transform.translation.x, 
                                  msg->transform.translation.y,
                                  msg->transform.translation.z;
        transform.linear() = Eigen::Quaterniond(1,
                                                0,
                                                0,
                                                0).toRotationMatrix();   
}

    double roll, pitch, yaw;
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!get_global_path)
    {
        return;
    }
    current_state(0) = msg->pose.pose.position.x + transform.translation().x();
    current_state(1) = msg->pose.pose.position.y + transform.translation().y();
    current_state(2) = msg->pose.pose.position.z + transform.translation().z();
    geometry_msgs::Quaternion current_orientation = msg->pose.pose.orientation;
    
    // 将当前四元数转换为 tf2::Quaternion 对象
    tf2::Quaternion quat;
    tf2::fromMsg(current_orientation, quat);

    // 将四元数转换为欧拉角 (roll, pitch, yaw)
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_state(0) = dijstra_pos[0].x();
    current_state(1) = dijstra_pos[0].y();
}

void trajCallBack(const gcopter::PolyTrajectory::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    trajectory_cloud->points.clear();
    trajectory_times.clear();
    // 校验数据有效性
    if (msg->durations.empty() || 
        msg->coeffs_x.rows.size() != msg->durations.size() ||
        msg->coeffs_y.rows.size() != msg->durations.size() ||
        msg->coeffs_z.rows.size() != msg->durations.size()) {
        ROS_ERROR("Invalid trajectory received!");
        return;
    }

    // 转换为 std::vector<double> 类型的持续时间
    const int pieceNum = msg->durations.size();
    std::vector<double> durations;
    durations.reserve(pieceNum);
    for (const auto &dur : msg->durations) {
        durations.push_back(dur);
    }

    // 填充系数矩阵（保持 3x6 形状）
    std::vector<Eigen::Matrix<double, 3, 6>> coeffMats;
    coeffMats.reserve(pieceNum);

    for (int i = 0; i < pieceNum; ++i) {
        Eigen::Matrix<double, 3, 6> coeffMat;

        // X方向
        for (int j = 0; j < 6; ++j) {
            coeffMat(0, j) = msg->coeffs_x.rows[i].data[j];
        }

        // Y方向
        for (int j = 0; j < 6; ++j) {
            coeffMat(1, j) = msg->coeffs_y.rows[i].data[j];
        }

        // Z方向
        for (int j = 0; j < 6; ++j) {
            coeffMat(2, j) = msg->coeffs_z.rows[i].data[j];
        }

        // 无需转置，直接添加
        coeffMats.push_back(coeffMat);
    }

    // 构造轨迹对象 
    currentTraj_ = Trajectory<5>(durations, coeffMats);
    for (double t = 0; t < currentTraj_.getTotalDuration(); t += 0.01) {
            Eigen::Vector3d pos = currentTraj_.getPos(t);
            pcl::PointXYZ point;
            point.x = pos.x();
            point.y = pos.y();
            point.z = pos.z();
            trajectory_cloud->points.push_back(point);
            trajectory_times.push_back(t);
        }
    
    traj_duration = currentTraj_.getTotalDuration();
    kdtree->setInputCloud(trajectory_cloud);
    trajStartTime_ = ros::Time::now(); // 记录轨迹开始时间
    get_traj = true;
    }

    void publish_control_cmd(const ros::TimerEvent &e) {
    std::lock_guard<std::mutex> traj_lock(traj_mutex_);
    if(!get_traj) return;

    std::vector<Eigen::Vector3d> tar_pos,tar_vel,tar_acc;
    //预分配内存
    tar_pos.reserve(N);
    tar_vel.reserve(N);
    tar_acc.reserve(N);
    double currentTime = (ros::Time::now() - trajStartTime_).toSec();

    pcl::PointXYZ search_point;
        search_point.x = current_state.x();
        search_point.y = current_state.y();
        search_point.z = current_state.z();

        std::vector<int> pointIdx(1);
        std::vector<float> pointDist(1);
        if (kdtree->nearestKSearch(search_point, 1, pointIdx, pointDist) == 0) {
            ROS_WARN("No nearest point found");
            return;
        }

    const int nearest_index = pointIdx[0];
    const int adjusted_index = std::min(nearest_index + 10, static_cast<int>(trajectory_times.size() - 1));
    const double t_start = trajectory_times[adjusted_index];
    Eigen::MatrixXd desired_state = Eigen::MatrixXd::Zero(N, 3);

        //kdtree搜索基准
        // for (int i = 0; i < N; ++i) {
        //     const double t = std::min(t_start + i*dt, traj_duration);
            
        //     const Eigen::Vector3d pos = currentTraj_.getPos(t);
        //     const Eigen::Vector3d vel = currentTraj_.getVel(t);
        //     const Eigen::Vector3d acc = currentTraj_.getAcc(t);

        //     desired_state(i, 0) = pos.x();
        //     desired_state(i, 1) = pos.y();
        //     desired_state(i, 2) = pos.z();

        //     tar_pos.push_back(pos);
        //     tar_vel.push_back(vel);
        //     tar_acc.push_back(acc);
        // }
    
    ////////////////////////////////////////////////////////////////////
    //时间基准
    if(t_start + (N-1)*dt <= traj_duration) {
        // 计算目标位置
        for (int i = 0; i < N; ++i) {
            const double t = t_start + i * dt;
            desired_state(i, 0) = currentTraj_.getPos(t)(0);
            desired_state(i, 1) = currentTraj_.getPos(t)(1);
            desired_state(i, 2) = currentTraj_.getPos(t)(2);
            // ROS_WARN("desired_state: %f %f %f", desired_state(i, 0), desired_state(i, 1), desired_state(i, 2));
            tar_pos.push_back(currentTraj_.getPos(t));
            tar_vel.push_back(currentTraj_.getVel(t));
            tar_acc.push_back(currentTraj_.getAcc(t));
        }
    }else if(t_start + (N-1)*dt > traj_duration
    && t_start < traj_duration) {
        // 轨迹未结束，使用当前时间到轨迹结束的部分
        for (int i = 0; i < N; ++i) {
            const double t = t_start + i * dt;
            if (t <= traj_duration) {
                desired_state(i, 0) = currentTraj_.getPos(t)(0);
                desired_state(i, 1) = currentTraj_.getPos(t)(1);
                desired_state(i, 2) = currentTraj_.getPos(t)(2);
                tar_pos.push_back(currentTraj_.getPos(t));
                tar_vel.push_back(currentTraj_.getVel(t));
                tar_acc.push_back(currentTraj_.getAcc(t));
            } else {
                desired_state(i, 0) = currentTraj_.getPos(traj_duration)(0);
                desired_state(i, 1) = currentTraj_.getPos(traj_duration)(1);
                desired_state(i, 2) = currentTraj_.getPos(traj_duration)(2);
                tar_pos.push_back(currentTraj_.getPos(traj_duration));
                tar_vel.push_back(currentTraj_.getVel(traj_duration));
                tar_acc.push_back(currentTraj_.getAcc(traj_duration));
            }
        }
    }   
     else {
        // 轨迹结束，使用最后一个位置
        for (int i = 0; i < N; ++i) {
            desired_state(i, 0) = currentTraj_.getPos(traj_duration)(0);
            desired_state(i, 1) = currentTraj_.getPos(traj_duration)(1);
            desired_state(i, 2) = currentTraj_.getPos(traj_duration)(2);
            tar_pos.push_back(currentTraj_.getPos(traj_duration));
            tar_vel.push_back(currentTraj_.getVel(traj_duration));
            tar_acc.push_back(currentTraj_.getAcc(traj_duration));
        }
    }

//////////////////////////////////////////////////////////////////////////////move base 
    // if (dijstra_pos.size() >= N) {
    //     for (int i = 0; i < N; ++i) {
    //         desired_state(i, 0) = dijstra_pos[i].x();
    //         desired_state(i, 1) = dijstra_pos[i].y();
    //         desired_state(i, 2) = dijstra_pos[i].z();
    //     }
    // } else {
    //     for (int i = 0; i < dijstra_pos.size(); ++i) {
    //         desired_state(i, 0) = dijstra_pos[i].x();
    //         desired_state(i, 1) = dijstra_pos[i].y();
    //         desired_state(i, 2) = dijstra_pos[i].z();
    //     }
    //     for (int i = dijstra_pos.size(); i < N; ++i) {
    //         desired_state(i, 0) = dijstra_pos.back().x();
    //         desired_state(i, 1) = dijstra_pos.back().y();
    //         desired_state(i, 2) = dijstra_pos.back().z();
    //     }
    // }

    
    // Visualize desired state
    visualization_msgs::MarkerArray desired_marker_array;
    for (int i = 0; i < N; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "desired_state";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = desired_state(i, 0);
        marker.pose.position.y = desired_state(i, 1);
        marker.pose.position.z = desired_state(i, 2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.4;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        desired_marker_array.markers.push_back(marker);
    }
    desired_pos_pub_.publish(desired_marker_array);
    
    auto result = mpc_ptr->solve(current_state, desired_state,0,0);
    // predict_path.header.frame_id = "odom";
    // predict_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::Point pt;
    visualization_msgs::MarkerArray marker_array;

    for (int i = 2; i < result.size(); i += 2) {
        pose_msg.pose.position.x = result[i];
        pose_msg.pose.position.y = result[i + 1];

        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "mpc_pos";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = result[i];
        marker.pose.position.y = result[i + 1];
        marker.pose.position.z = 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.4; // Height of the cylinder
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);

        // predict_path.poses.push_back(pose_msg);
    }
    predict_pos_pub_.publish(marker_array);

    geometry_msgs::Twist cmd;
    cmd.linear.x = result[0]*0.47;
    cmd.linear.y = result[1]*0.47;
    cmd.linear.z = result[2];

    double dx = current_state(0) -dijstra_pos.back().x();
    double dy = current_state(1) - dijstra_pos.back().y();

    if(std::sqrt(dx * dx + dy * dy) < 0.3)
    {
    cmd.linear.x = 0.5 * cmd.linear.x;   
    cmd.linear.y = 0.5 * cmd.linear.y;   
    }

    cmd_vel_pub.publish(cmd);

    navigation.x.data=cmd.linear.y;
    navigation.y.data=cmd.linear.x;
    navigation.z.data=yaw;            
    navigation.yaw.data=yaw;
	navigation_pub.publish(navigation);

    // predict_path_pub.publish(predict_path);
    // predict_path.poses.clear();
    }

    ~controller();
};


controller::~controller()
{

}


#endif // MPC_H


