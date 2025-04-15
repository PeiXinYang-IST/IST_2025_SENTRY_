#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline_opt/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>

#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/topo_prm.h>
#include <std_msgs/Empty.h>
#include <plan_env/edt_environment.h>
#include <visualization_msgs/MarkerArray.h>
#include <plan_manage/plan_container.hpp>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <gcopter/PolyTrajectory.h>
#include <gcopter/CoeffRow.h>
#include <gcopter/CoeffMatrix.h>
#include "gcopter/trajectory.hpp"

namespace fast_planner {

// Fast Planner Manager
// Key algorithms of mapping and planning are called

class FastPlannerManager {
  // SECTION stable
public:
  FastPlannerManager();
  ~FastPlannerManager();
  bool jps_updated = false;
  bool mpc_path_updated = false;
  /* main planning interface */
  bool kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                         Eigen::Vector3d end_pt, Eigen::Vector3d end_vel);
  bool planGlobalTraj(const Eigen::Vector3d& start_pos);
  bool topoReplan(bool collide);

  void planYaw(const Eigen::Vector3d& start_yaw);

  void initPlanModules(ros::NodeHandle& nh);
  void setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints);
  bool checkTrajCollision(double& distance);
  ros::Publisher dist_pub_;
  PlanParameters pp_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;
  ros::Publisher grad_pub_;
  ros::Subscriber odom_sub;
  ros::Subscriber mpc_path_sub;
  ros::Subscriber nav_pose_sub;
  ros::Subscriber path_sub_,global_sub,JPS_sub;
  vector<Eigen::Vector3d> move_base_point_set;
  ros::Publisher raw_path_pub_;
  ros::Subscriber trajsub;
  ros::Publisher simplified_path_pub_;
  ros::Publisher idx_pub_;
  nav_msgs::Path JPS_path_,global_path,mpc_path_;
  ros::Publisher mpc_path_update_pub_;
  Trajectory<5> currentTraj_;
  ros::Time trajStartTime_; // 轨迹开始时间
  ros::Timer check_collision_pub;

private:
  /* main planning algorithms & modules */
  SDFMap::Ptr sdf_map_;
  Eigen::Vector3d odom_pos;
  unique_ptr<Astar> geo_path_finder_;
  unique_ptr<KinodynamicAstar> kino_path_finder_;
  unique_ptr<TopologyPRM> topo_prm_;
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;
  geometry_msgs::PoseStamped current_nav_pose;
  geometry_msgs::PoseStamped last_nav_pose;
  double path_idx;
  void updateTrajInfo();

  // topology guided optimization

  void findCollisionRange(vector<Eigen::Vector3d>& colli_start, vector<Eigen::Vector3d>& colli_end,
                          vector<Eigen::Vector3d>& start_pts, vector<Eigen::Vector3d>& end_pts);

  void optimizeTopoBspline(double start_t, double duration, vector<Eigen::Vector3d> guide_path,
                           int traj_id);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double& dt, double& duration);
  Eigen::MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double& dt);

  void selectBestTraj(NonUniformBspline& traj);
  void refineTraj(NonUniformBspline& best_traj, double& time_inc);
  void reparamBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt,
                      double& time_inc);
  void nav_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  // heading planning
  void JPS_pathCallback(const nav_msgs::Path::ConstPtr&msg);
  void calcNextYaw(const double& last_yaw, double& yaw);
  void odomCallback(const nav_msgs::Odometry &msg);
  void pathCallback(const nav_msgs::Path &msg);
  void globalCallback(const std_msgs::Bool::ConstPtr& msg);
  void visualizePaths(const std::vector<Eigen::Vector3d>& global_points,const std::vector<Eigen::Vector3d>& points);
    // !SECTION stable
  // SECTION developing
  void mpcpathCallback(const nav_msgs::Path::ConstPtr& msg);
  void trajCallBack(const gcopter::PolyTrajectory::ConstPtr &msg);
  void GCOPTER_check_collision(const ros::TimerEvent &e);
public:
  typedef unique_ptr<FastPlannerManager> Ptr;

  // !SECTION
};
}  // namespace fast_planner

#endif