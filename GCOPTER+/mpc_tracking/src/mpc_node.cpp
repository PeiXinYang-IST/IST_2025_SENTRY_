// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
// #include "bspline/non_uniform_bspline.h"
// #include "mpc_tracking/Bspline.h"
// #include "std_msgs/Empty.h"
// #include "std_msgs/Bool.h"
// #include "mpc_tracking/mpc.h"
// #include "math.h"
// #include <std_msgs/Float64.h>
// #include <sentry_serial/navigation.h>
// #include <geometry_msgs/Vector3.h>
// #include "cubic_spline/cubic_spline_ros.h"
// #include <pcl/kdtree/kdtree_flann.h>
// #include <control.h>

// ros::Publisher navigation_pub;
// ros::Publisher desired_pub_;
// sentry_serial::navigation navigation;
// ros::Publisher MPC_TRACK_PATH_pub_;
// // #define BACKWARD_HAS_DW 1
// // #include "backward.hpp"
// // namespace backward{
// //     backward::SignalHandling sh;
// // }

// double yaw_angle;
// using fast_planner::NonUniformBspline;
// double dist;
// ros::Publisher cmd_vel_pub, motion_path_pub, predict_path_pub;
// nav_msgs::Path predict_path, motion_path;
// nav_msgs::Odometry odom;
// geometry_msgs::Vector3 grab;
// Eigen::Vector3f cur_vel;
// nav_msgs::Path global_path_;
// bool get_path_=false;
// bool mpc_start=false;
// bool receive_traj = false;
// vector<NonUniformBspline> traj;
// double traj_duration;
// Eigen::Vector2d gradient(grab.x, grab.y);
// ros::Time start_time;
// double last_yaw;
// double time_forward;

// vector<Eigen::Vector3d> traj_cmd, traj_real;

// ros::Timer control_cmd_pub, path_pub;

// const int N = 30;
// const double dt = 0.1;

// Eigen::Vector3d current_state;

// unique_ptr<Mpc> mpc_ptr;

// void bsplineCallback(mpc_tracking::BsplineConstPtr msg) {
//   // parse pos traj
//   Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);

//   Eigen::VectorXd knots(msg->knots.size());
//   for (int i = 0; i < msg->knots.size(); ++i) {
//     knots(i) = msg->knots[i];
//   }

//   for (int i = 0; i < msg->pos_pts.size(); ++i) {
//     pos_pts(i, 0) = msg->pos_pts[i].x;
//     pos_pts(i, 1) = msg->pos_pts[i].y;
//     pos_pts(i, 2) = msg->pos_pts[i].z;
//   }

//   NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
//   pos_traj.setKnot(knots);

//   // parse yaw traj

//   Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
//   for (int i = 0; i < msg->yaw_pts.size(); ++i) {
//     yaw_pts(i, 0) = msg->yaw_pts[i];
//   }

//   NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

//   start_time = msg->start_time;

//   traj.clear();
//   traj.push_back(pos_traj);
//   traj.push_back(traj[0].getDerivative());
//   traj.push_back(traj[1].getDerivative());
//   traj.push_back(yaw_traj);
//   traj.push_back(yaw_traj.getDerivative());

//   traj_duration = traj[0].getTimeSum();

//   receive_traj = true;
// }

// void replanCallback(std_msgs::Empty msg) {
//   /* reset duration */
//   const double time_out = 0.01;
//   ros::Time time_now = ros::Time::now();
//   double t_stop = (time_now - start_time).toSec() + time_out;
//   traj_duration = min(t_stop, traj_duration);
// }

// nav_msgs::Path mpc_path_;

// void path_update_callback(std_msgs::Empty msg) {
//   mpc_path_ = global_path_;
// }

// void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
//   nav_msgs::Path smoothed_path;
//   global_path_ = *msg;
//   // GenTraj(global_path_, smoothed_path);
//   // global_path_ = smoothed_path;
//   get_path_ = true;

//   // Check if mpc_path_ is empty
//   if (mpc_path_.poses.empty()) {
//     mpc_path_ = global_path_;
//   }

// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   mpc_path_ = global_path_;
// }

// void start_task(const std_msgs::Bool::ConstPtr& msg){
//   mpc_start = true;
// }

// void odomCallback(const nav_msgs::Odometry &msg) {
//   if(get_path_ && mpc_start){
//   odom = msg;
//   cur_vel.x() = msg.twist.twist.linear.x;
//   cur_vel.y() = msg.twist.twist.linear.y;

// //sim
//     // current_state(0) = msg.pose.pose.position.x;
//     // current_state(1) = msg.pose.pose.position.y;
//     // current_state(2) = tf2::getYaw(msg.pose.pose.orientation);
// //real
//     global_path_.header.frame_id = "odom";
//     global_path_.header.stamp = ros::Time::now();
//     current_state(0) = global_path_.poses[0].pose.position.x;
//     current_state(1) = global_path_.poses[0].pose.position.y+0.105;
//     current_state(2) = 0.0;

//     // current_state(2) = tf2::getYaw(msg.pose.pose.orientation);
//     //double yaw1 = tf2::getYaw(msg.pose.pose.orientation);
//     // Eigen::Quaterniond quaternion;
//     // quaternion.x() = msg.pose.pose.orientation.x;
//     // quaternion.y() = msg.pose.pose.orientation.y;
//     // quaternion.z() = msg.pose.pose.orientation.z;
//     // quaternion.w() = msg.pose.pose.orientation.w;


//     // Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();
//     // double yaw2 = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
//     // cout << "x:" << current_state(0) << " " << "y:" << current_state(1) << endl;
//     // cout << "yaw1:" << current_state(2) << endl;
//     //cout << "yaw2:" << yaw2 << endl;
//   }
// }

// void publish_control_cmd(const ros::TimerEvent &e) {

//     if(!get_path_) return;
//     mpc_path_.header.frame_id = "odom";
//     mpc_path_.header.stamp = ros::Time::now();
//     MPC_TRACK_PATH_pub_.publish(mpc_path_);

//     // if (!receive_traj) return;

//     // ros::Time time_now = ros::Time::now();
//     // double t_cur = (time_now - start_time).toSec();
//     // t_cur = std::max(t_cur, 0.0);  // 保证时间不小于0

//     // Eigen::Vector3d pos, vel, acc, pos_f;
//     // double yaw, yawdot;

//     Eigen::MatrixXd desired_state = Eigen::MatrixXd::Zero(N, 3);

//   //   if (t_cur + (N-1) * dt <= traj_duration && t_cur > 0) {
//   //     for (int i = 0; i < N; ++i) {
//   //       pos = traj[0].evaluateDeBoorT(t_cur + i * dt);
//   //       vel = traj[1].evaluateDeBoorT(t_cur + i * dt);
//   //       acc = traj[2].evaluateDeBoorT(t_cur + i * dt);
//   //       yaw = traj[3].evaluateDeBoorT(t_cur + i * dt)[0];
//   //       yawdot = traj[4].evaluateDeBoorT(t_cur + i * dt)[0];

//   //       desired_state(i, 0) = pos[0];
//   //       desired_state(i, 1) = pos[1];
//   //       desired_state(i, 2) = yaw;
//   //     }
//   //   } else if (t_cur + (N-1) * dt > traj_duration && t_cur < traj_duration) {
//   //       int more_num = (t_cur + (N-1) * dt - traj_duration) / dt;
//   //       for (int i = 0; i < N - more_num; ++i) {
//   //         pos = traj[0].evaluateDeBoorT(t_cur + i * dt);
//   //         vel = traj[1].evaluateDeBoorT(t_cur + i * dt);
//   //         acc = traj[2].evaluateDeBoorT(t_cur + i * dt);
//   //         yaw = traj[3].evaluateDeBoorT(t_cur + i * dt)[0];
//   //         yawdot = traj[4].evaluateDeBoorT(t_cur + i * dt)[0];

//   //         desired_state(i, 0) = pos(0);
//   //         desired_state(i, 1) = pos(1);
//   //         desired_state(i, 2) = yaw;          
//   //       }
//   //       for (int i = N - more_num; i < N; ++i) {
//   //         pos = traj[0].evaluateDeBoorT(traj_duration);
//   //         vel.setZero();
//   //         acc.setZero();
//   //         yaw = traj[3].evaluateDeBoorT(traj_duration)[0];
//   //         yawdot = traj[4].evaluateDeBoorT(traj_duration)[0];

//   //         desired_state(i, 0) = pos(0);
//   //         desired_state(i, 1) = pos(1);
//   //         desired_state(i, 2) = yaw;
//   //       }
//   //   } else if (t_cur >= traj_duration)  {
//   //     pos = traj[0].evaluateDeBoorT(traj_duration);
//   //     vel.setZero();
//   //     acc.setZero();
//   //     yaw = traj[3].evaluateDeBoorT(traj_duration)[0];
//   //     yawdot = traj[4].evaluateDeBoorT(traj_duration)[0];
//   //     for (int i = 0; i < N; ++i) {
//   //         desired_state(i, 0) = pos(0);
//   //         desired_state(i, 1) = pos(1);
//   //         desired_state(i, 2) = yaw;
//   //     }
//   //   } else {
//   //     cout << "[Traj server]: invalid time." << endl;
//   // }

// //   pcl::KdTreeFLANN<pcl::PointXY> kdtree;
// //   pcl::PointCloud<pcl::PointXY>::Ptr path_points(new pcl::PointCloud<pcl::PointXY>);

// //   // 填充路径点
// //   for (const auto& pose : mpc_path_.poses) {
// //     pcl::PointXY pt;
// //     pt.x = pose.pose.position.x;
// //     pt.y = pose.pose.position.y;
// //     path_points->push_back(pt);
// //   }
// //   kdtree.setInputCloud(path_points);
  

// // pcl::PointXY query_pt;
// // query_pt.x = current_state(0);
// // query_pt.y = current_state(1);

// // int nearest_idx;
// // float nearest_dist;
// // std::vector<int> indices(1);
// // std::vector<float> distances(1);

// // if (kdtree.nearestKSearch(query_pt, 1, indices, distances) > 0) {
// //     nearest_idx = indices[0];
// //     nearest_dist = distances[0];
// // }

// int nearest_idx;
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// nearest_idx = 0;

// if (global_path_.poses.size() > N + nearest_idx) {
//   for (int i = 0; i < N; ++i) {
//     desired_state(i, 0) = global_path_.poses[nearest_idx + i].pose.position.x;
//     desired_state(i, 1) = global_path_.poses[nearest_idx + i].pose.position.y;
//     desired_state(i, 2) = 0;
//   }
// } else if (global_path_.poses.size() > nearest_idx && global_path_.poses.size() <= N + nearest_idx) {
//   for (int i = 0; i < global_path_.poses.size() - nearest_idx; ++i) {
//     desired_state(i, 0) = global_path_.poses[nearest_idx + i].pose.position.x;
//     desired_state(i, 1) = global_path_.poses[nearest_idx + i].pose.position.y;
//     desired_state(i, 2) = 0;
//   }
//   for (int i = global_path_.poses.size() - nearest_idx; i < N; ++i) {
//     desired_state(i, 0) = global_path_.poses.back().pose.position.x;
//     desired_state(i, 1) = global_path_.poses.back().pose.position.y;
//     desired_state(i, 2) = 0;
//   }
// }

// // Visualize desired_state using markers
// visualization_msgs::MarkerArray marker_array;
// for (int i = 0; i < desired_state.rows(); ++i) {
//   visualization_msgs::Marker marker;
//   marker.header.frame_id = "odom";
//   marker.header.stamp = ros::Time::now();
//   marker.ns = "desired_state";
//   marker.id = i;
//   marker.type = visualization_msgs::Marker::SPHERE;
//   marker.action = visualization_msgs::Marker::ADD;
//   marker.pose.position.x = desired_state(i, 0);
//   marker.pose.position.y = desired_state(i, 1);
//   marker.pose.position.z = 0.0;
//   marker.scale.x = 0.1;
//   marker.scale.y = 0.1;
//   marker.scale.z = 0.1;
//   marker.color.a = 1.0;
//   marker.color.r = 0.0;
//   marker.color.g = 1.0;
//   marker.color.b = 0.0;
//   marker_array.markers.push_back(marker);
// }

// desired_pub_.publish(marker_array);

//     // auto start_time = std::chrono::high_resolution_clock::now();
//     auto result = mpc_ptr->solve(current_state, desired_state);
//     // auto end_time = std::chrono::high_resolution_clock::now();
//     // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
//     // ROS_INFO("MPC solve time: %ld ms", duration);

//     geometry_msgs::Twist cmd;

//     cmd.linear.x = result[0]*0.4;
//     cmd.linear.y = result[1]*0.4;
//     cmd.linear.z = result[2];
//     // 获取当前速度
//     double current_x = cur_vel.x();
//     double current_y = cur_vel.y();

//     // 获取目标速度
//     double desired_x = cmd.linear.x;
//     double desired_y = cmd.linear.y;

//     // 限制最大阶跃
//     const double max_step = 10.5;

//     // 计算速度变化矢量
//     double delta_x = desired_x - current_x;
//     double delta_y = desired_y - current_y;

//     // 计算速度变化矢量的模（欧几里得距离）
//     double delta_norm = std::sqrt(delta_x * delta_x + delta_y * delta_y);

//     // 如果速度变化矢量的模超过最大阶跃，则进行缩放
//   if (delta_norm > max_step) {
//     double scale_factor = max_step / delta_norm; // 缩放因子
//     delta_x *= scale_factor; // 缩放 x 分量
//     delta_y *= scale_factor; // 缩放 y 分量
//     ROS_WARN("acc too high!!!");
// }

//     // 更新目标速度
//     cmd.linear.x = current_x + delta_x;
//     cmd.linear.y = current_y + delta_y;

//     double dx = current_state(0) - global_path_.poses.back().pose.position.x;
//     double dy = current_state(1) - global_path_.poses.back().pose.position.y;

//     // if(std::sqrt(dx * dx + dy * dy) < 0.7)
//     // cmd.linear.z = 1.0;

//     // else
//     // cmd.linear.z = 0.15;

//     if(std::sqrt(dx * dx + dy * dy) < 0.7)
//     {
//     cmd.linear.x = 0.2 * cmd.linear.x;   
//     cmd.linear.y = 0.2 * cmd.linear.y;   
//     }

//     // cmd.linear.x = (cmd.linear.x * cos(current_state(2)) + cmd.linear.y * sin(current_state(2)));
//     // cmd.linear.y = (- cmd.linear.x * sin(current_state(2)) + cmd.linear.y * cos(current_state(2)));

//     cmd_vel_pub.publish(cmd);
//     //cout << "u:" << result[0] << " " << "r:" << result[1] << endl;

//     navigation.yaw.data=yaw_angle;
//       navigation.x.data=cmd.linear.x;
//       navigation.y.data=cmd.linear.y;
//       navigation.z.data=cmd.linear.z;            

// 	  navigation_pub.publish(navigation);

//     predict_path.header.frame_id = "odom";
//     predict_path.header.stamp = ros::Time::now();
//     geometry_msgs::PoseStamped pose_msg;
//     geometry_msgs::Point pt;
//     for (int i = 2; i < result.size(); i += 2) {
//         pose_msg.pose.position.x = result[i];
//         pose_msg.pose.position.y = result[i + 1];
//         predict_path.poses.push_back(pose_msg);
//     }
//     predict_path_pub.publish(predict_path);
//     predict_path.poses.clear();
// }

// void yaw_callback(const std_msgs::Float32& msg)
// {
//   static double last_yaw;
//   double alpha = 0.8;

// 	yaw_angle = alpha*msg.data + (1-alpha)*last_yaw;
//   last_yaw = yaw_angle;
// }

// void distCallback(std_msgs::Float64 msg)
// {
//   dist = msg.data;
//   // ROS_WARN("DIST: %f",dist);
// }


// void grabCallback(geometry_msgs::Vector3 msg)
// {
//   grab.x = msg.x;
//   grab.y = msg.y;
//   grab.z = msg.z;
//   gradient.x() = grab.x;
//   gradient.y() = grab.y;
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "mpc_tracking_node");
//     ros::NodeHandle nh;
//     cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mpc_cmd_vel", 1);
//     predict_path_pub = nh.advertise<nav_msgs::Path>("/predict_path", 1);
//     motion_path_pub = nh.advertise<nav_msgs::Path>("/motion_path", 1);
//     navigation_pub = nh.advertise<sentry_serial::navigation>("navigation",10);
//     MPC_TRACK_PATH_pub_ = nh.advertise<nav_msgs::Path>("/MPC_TRACK_PATH", 1);
//     desired_pub_ = nh.advertise<visualization_msgs::MarkerArray>("desired_state", 1);
//     ros::Subscriber odom_sub = nh.subscribe("/odom", 1, &odomCallback);
//     ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);
//     ros::Subscriber replan_sub = nh.subscribe("planning/replan", 10, replanCallback);
//     ros::Subscriber path_sub_ = nh.subscribe("/move_base1/NavfnROS/plan", 10, pathCallback);
//     ros::Subscriber fast_planner_sub_ = nh.subscribe("/MY_ICP/fast_planner_start", 10, start_task);
//     ros::Subscriber yaw_sub = nh.subscribe("Obstacle_cloudget/yaw_angle", 1000, yaw_callback); 
//     ros::Subscriber update_sub = nh.subscribe("mpc_path_update", 1000, path_update_callback); 
//     ros::Subscriber dist_sub = nh.subscribe("/dist", 10, distCallback);
//     ros::Subscriber grab_sub = nh.subscribe("/grad", 10, grabCallback);
//     control_cmd_pub = nh.createTimer(ros::Duration(0.1), publish_control_cmd);
//     controller controller(Config(ros::NodeHandle("~")), nh);

//     ros::spin();
//     return 0;

// }

#include <mpc_tracking/control.h>
#include <mpc_tracking/mpc.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_track_node");
    ros::NodeHandle nh;
    controller controller(MPC_Config(ros::NodeHandle("~")), nh);
    ros::spin();
    return 0;
}

