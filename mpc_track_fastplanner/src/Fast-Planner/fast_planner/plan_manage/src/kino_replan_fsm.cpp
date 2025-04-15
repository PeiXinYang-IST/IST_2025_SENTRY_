
#include <plan_manage/kino_replan_fsm.h>

int temp = 0;
namespace fast_planner {

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;
  fast_planner_start = false;
  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
  

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }
  get_path_ = false;

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));
  map_to_odom_vector = Eigen::Vector3f::Identity();

  /* callback */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);

  waypoint_sub_ =  nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom", 1, &KinoReplanFSM::odometryCallback, this);
  map_to_odom_sub_ = nh.subscribe("/MY_ICP/map_to_odom", 1, &KinoReplanFSM::map_to_odomcallback, this);
  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  // robot_pose_pub_  = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  global_pub_ = nh.advertise<std_msgs::Bool>("/planning/global_enable", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
  // path_sub_ = nh.subscribe("/move_base1/NavfnROS/plan", 10, &KinoReplanFSM::pathCallback, this);
  fast_planner_sub_ = nh.subscribe("/MY_ICP/fast_planner_start", 10, &KinoReplanFSM::start_task, this);
  trajectory_start_time_ = ros::Time::now();  // 初始化为当前时间
}

void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;

  // if(!get_path_)
  //   return;
  
  // if(planner_manager_->edt_environment_->evaluateCoarseEDT(odom_pos_,-1.0)<0){
  //   cout << "collision" << endl;
  //   return;
  // }

  cout << "Triggered!" << endl;
  trigger_ = true;

  Eigen::Vector3d pt;
  pt << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, msg->poses[0].pose.position.z;
  double dist = planner_manager_->edt_environment_->evaluateCoarseEDT(pt, -1.0);

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 0.0;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");

}

void KinoReplanFSM::map_to_odomcallback(const geometry_msgs::TransformStamped& msg)
{
  map_to_odom_vector.x() = msg.transform.translation.x;
  map_to_odom_vector.y() = msg.transform.translation.y;
  map_to_odom_vector.z() = msg.transform.translation.z;
}

double global_path_count_;
void KinoReplanFSM::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
  global_path_ = *msg;
  global_path_count_++;
  get_path_ = true;
}

void KinoReplanFSM::start_task(const std_msgs::Bool::ConstPtr& msg){
  if(msg->data)
  fast_planner_start = true;
}

//设置目标点
void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
static double global_path_count_check;
// if(get_path_ && global_path_count_check!=global_path_count_){
if(get_path_){
  global_path_count_check=global_path_count_;
  geometry_msgs::PoseStamped robot_pose_;
  global_path_.header.frame_id = "odom";
  global_path_.header.stamp = ros::Time::now();
  odom_pos_(0) = global_path_.poses[0].pose.position.x;
  odom_pos_(1) = global_path_.poses[0].pose.position.y+0.105;
  odom_pos_(2) = global_path_.poses[0].pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;
  // ROS_WARN("odom_vel_: %f, %f, %f", odom_vel_(0), odom_vel_(1), odom_vel_(2));
  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  robot_pose_.pose = global_path_.poses[0].pose;

  
  // robot_pose_.pose.position.x = odom_pos_(0);
  // robot_pose_.pose.position.y = odom_pos_(1);
  // robot_pose_.pose.position.z = odom_pos_(2);
  robot_pose_.header.frame_id = "odom";
  // robot_pose_.pose.orientation = tf2::toMsg(new_quat);
  // robot_pose_pub_.publish(robot_pose_);
  have_odom_ = true;
}
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {

      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      // ROS_WARN("start_vel_: %f, %f, %f", start_vel_(0), start_vel_(1), start_vel_(2));
      start_acc_.setZero();
      
      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;
      start_yaw_(0) = 0.0;
      start_yaw_(1) = 0.0;
      bool success = callKinodynamicReplan();
      static int last_state = 0;
      std_msgs::Bool global_msg;
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      //这里大于10次时发布replan
      std_msgs::Empty replan_msg;
      // replan_pub_.publish(replan_msg);

      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
      t_cur                   = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      // 检查odom与轨迹当前位置的距离
      double dist_to_odom = (pos - odom_pos_).norm();

      // double current_speed = odom_vel_.norm();
      // double dynamic_thresh = std::max(0.5, 0.2 * current_speed); // 基础阈值0.15m，按速度比例增加
      // if (dist_to_odom > dynamic_thresh) {
      //   changeFSMExecState(GEN_NEW_TRAJ, "DIST_CONDITION");
      //   break;
      // }


      if (dist_to_odom > 0.2) {
        // ROS_WARN("dist_to_odom: %f", dist_to_odom);
        // ROS_WARN("Odom distance exceeds threshold, triggering replan.");
        changeFSMExecState(REPLAN_TRAJ, "DIST_CONDITION");
        break;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2) {
              // if ((end_pt_ - pos).norm() < 0.1) {
        have_target_ = false;
        temp = 1;
        changeFSMExecState(WAIT_TARGET, "FSM");
        
        return;
      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        // cout << "near end" << endl;
        return;

      } else if ((info->start_pos_ - pos).norm() < replan_thresh_) {
        // cout << "near start" << endl;
        return;

      } else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
ros::Time replan_start_time = ros::Time::now();
double replan_timeout = 1.0; // 超时时间1秒

// 在状态处理中检查超时
if ((ros::Time::now() - replan_start_time).toSec() > replan_timeout) {
  changeFSMExecState(GEN_NEW_TRAJ, "REPLAN_TIMEOUT");
}

// 在 REPLAN_TRAJ 状态或全局变量中定义
static ros::Time last_replan_time = ros::Time(0);
double replan_cooldown = 0.25; // 冷却时间 0.5秒

if ((ros::Time::now() - last_replan_time).toSec() < replan_cooldown) {
  return; // 未过冷却时间，跳过重规划
}
last_replan_time = ros::Time::now(); // 更新最后触发时间

      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();

      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      // start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      // start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      // start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      start_yaw_(0) = 0;
      start_yaw_(1) = 0;
      start_yaw_(2) = 0;
      
      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        trajectory_start_time_ = ros::Time::now();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.1) {
      /* try to find a max distance goal around */
      bool            new_goal = false;
      const double    dr = 0.05, dtheta = 15, dz = 0.001;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;
      
      for (double r = dr; r <= 20 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.1) {
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");

        std_msgs::Empty emt;
        replan_pub_.publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

  if (plan_success) {

    planner_manager_->planYaw(start_yaw_);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();
    // bspline.header.frame_id = "odom";
    // bspline.header.stamp = ros::Time::now();
    bspline_pub_.publish(bspline);
    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    
    return false;
  }
}

// KinoReplanFSM::
}  // namespace fast_planner
