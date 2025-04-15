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

using namespace std;

unique_ptr<Mpc> mpc_ptr;

struct Config
{

    std::string trajtopic;

    Config(const ros::NodeHandle &nh_priv)
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
    Config config;
    ros::Subscriber trajsub;
    ros::Subscriber odom_sub;
    ros::Publisher predict_path_pub;
    ros::Publisher navigation_pub;
    ros::Publisher predict_pos_pub_;
    ros::Publisher desired_pos_pub_;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber map_to_odom_sub;
    ros::Subscriber global_path_sub;
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    std::vector<Eigen::Vector3d> dijstra_pos;
    ros::NodeHandle nh;
    Trajectory<5> currentTraj_;
    ros::Time trajStartTime_; // 轨迹开始时间
    ros::Timer control_cmd_pub;
    bool get_traj;
    Eigen::Vector3d current_state;
    std::mutex traj_mutex_;
    unique_ptr<Mpc> mpc_ptr;
    nav_msgs::Path predict_path;
    sentry_serial::navigation navigation;
    bool get_global_path = false;
public:
    controller(const Config &conf,ros::NodeHandle &nh_)
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
    mpc_ptr.reset(new Mpc());
}

void globalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    if (msg->poses.size() > 0)
    {
        dijstra_pos.clear();
        for (size_t i = 0; i < msg->poses.size(); i += 10)
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

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!get_global_path)
    {
        return;
    }
    current_state(0) = msg->pose.pose.position.x + transform.translation().x();
    current_state(1) = msg->pose.pose.position.y + transform.translation().y();
    current_state(2) = msg->pose.pose.position.z + transform.translation().z();
    current_state(0) = dijstra_pos[0].x();
    current_state(1) = dijstra_pos[0].y()+0.105;

}

void trajCallBack(const gcopter::PolyTrajectory::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(traj_mutex_);
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
    Eigen::MatrixXd desired_state = Eigen::MatrixXd::Zero(N, 3);

    if(currentTime + (N-1)*dt <= currentTraj_.getTotalDuration()) {
        // 计算目标位置
        for (int i = 0; i < N; ++i) {
            const double t = currentTime + i * dt;
            desired_state(i, 0) = currentTraj_.getPos(t)(0);
            desired_state(i, 1) = currentTraj_.getPos(t)(1);
            desired_state(i, 2) = currentTraj_.getPos(t)(2);
            // ROS_WARN("desired_state: %f %f %f", desired_state(i, 0), desired_state(i, 1), desired_state(i, 2));
            tar_pos.push_back(currentTraj_.getPos(t));
            tar_vel.push_back(currentTraj_.getVel(t));
            tar_acc.push_back(currentTraj_.getAcc(t));
        }
    }else if(currentTime + (N-1)*dt > currentTraj_.getTotalDuration() 
    && currentTime < currentTraj_.getTotalDuration()) {
        // 轨迹未结束，使用当前时间到轨迹结束的部分
        for (int i = 0; i < N; ++i) {
            const double t = currentTime + i * dt;
            if (t <= currentTraj_.getTotalDuration()) {
                desired_state(i, 0) = currentTraj_.getPos(t)(0);
                desired_state(i, 1) = currentTraj_.getPos(t)(1);
                desired_state(i, 2) = currentTraj_.getPos(t)(2);
                tar_pos.push_back(currentTraj_.getPos(t));
                tar_vel.push_back(currentTraj_.getVel(t));
                tar_acc.push_back(currentTraj_.getAcc(t));
            } else {
                desired_state(i, 0) = currentTraj_.getPos(currentTraj_.getTotalDuration())(0);
                desired_state(i, 1) = currentTraj_.getPos(currentTraj_.getTotalDuration())(1);
                desired_state(i, 2) = currentTraj_.getPos(currentTraj_.getTotalDuration())(2);
                tar_pos.push_back(currentTraj_.getPos(currentTraj_.getTotalDuration()));
                tar_vel.push_back(currentTraj_.getVel(currentTraj_.getTotalDuration()));
                tar_acc.push_back(currentTraj_.getAcc(currentTraj_.getTotalDuration()));
            }
        }
    }   
     else {
        // 轨迹结束，使用最后一个位置
        for (int i = 0; i < N; ++i) {
            desired_state(i, 0) = currentTraj_.getPos(currentTraj_.getTotalDuration())(0);
            desired_state(i, 1) = currentTraj_.getPos(currentTraj_.getTotalDuration())(1);
            desired_state(i, 2) = currentTraj_.getPos(currentTraj_.getTotalDuration())(2);
            tar_pos.push_back(currentTraj_.getPos(currentTraj_.getTotalDuration()));
            tar_vel.push_back(currentTraj_.getVel(currentTraj_.getTotalDuration()));
            tar_acc.push_back(currentTraj_.getAcc(currentTraj_.getTotalDuration()));
        }
    }
    
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
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.4;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        desired_marker_array.markers.push_back(marker);
    }
    desired_pos_pub_.publish(desired_marker_array);
    
    auto result = mpc_ptr->solve(current_state, desired_state);
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
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
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
    cmd.linear.x = result[0]*0.4;
    cmd.linear.y = -result[1]*0.4;
    cmd.linear.z = result[2];


    cmd_vel_pub.publish(cmd);

    navigation.x.data=cmd.linear.x;
    navigation.y.data=cmd.linear.y;
    navigation.z.data=cmd.linear.z;            

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


