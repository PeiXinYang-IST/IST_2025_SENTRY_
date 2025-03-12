#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <boost/thread/mutex.hpp>
#include <thread>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "utility.h"
#include "cubic_spline/cubic_spline_ros.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #define ifdebug_

class PIDPathFollower
{
public:
    typedef struct _pid_struct_t
    {
        float kp;
        float ki;
        float kd;
        float i_max;
        float out_max;
        
        float ref;      // target value
        float fdb;      // feedback value
        float err[2];   // error and last error

        float p_out;
        float i_out;
        float d_out;
        float output;
        float K_ffc_dynamic;
        float K_ffc_static;
        float ffc_out;
    } pid_struct_t;

    void pid_init(pid_struct_t& pid,
                  float kp,
                  float ki,
                  float kd,
                  float K_ffc_static,
                  float K_ffc_dynamic,
                  float i_max,
                  float out_max)
    {
        pid.kp      = kp;
        pid.ki      = ki;
        pid.kd      = kd;
        pid.i_max   = i_max;
        pid.out_max = out_max;
        pid.K_ffc_static = K_ffc_static;
        pid.K_ffc_dynamic = K_ffc_dynamic;
    }

    float LIMIT_MIN_MAX(float value,float min,float max)
    {
        if(value>max)
            return max;
        else if (value<min)
            return min;
        else   
            return value;
    }

    float pid_calc(pid_struct_t& pid, float ref, float fdb)
    {
        static float ffc_static_out = 0, ffc_dynamic_out = 0;
        pid.ref = ref;
        pid.fdb = fdb;
        pid.err[1] = pid.err[0];
        pid.err[0] = pid.ref - pid.fdb;
        
        pid.p_out  = pid.kp * pid.err[0];
        pid.i_out += pid.ki * pid.err[0];
        pid.d_out  = pid.kd * (pid.err[0] - pid.err[1]);
        pid.i_out=LIMIT_MIN_MAX(pid.i_out, -pid.i_max, pid.i_max);
        
        // Add feedforward control
        pid.ffc_out = pid.K_ffc_static * ref + pid.K_ffc_dynamic * (ref - pid.fdb);
        
        pid.output = pid.p_out + pid.i_out + pid.d_out + pid.ffc_out;
        pid.output=LIMIT_MIN_MAX(pid.output, -pid.out_max, pid.out_max);
        return pid.output;
    }

    PIDPathFollower() : last_time_(ros::Time::now()),plan_(false),plan_count(0) {
        nh_ = ros::NodeHandle("~");
        nh_.param("max_speed", max_speed_, 1.0);
        nh_.param("pid_p", pid_p_, 1.0);
        nh_.param("pid_i", pid_i_, 0.005);
        nh_.param("pid_d", pid_d_, 0.1);
        nh_.param("i_max_", i_max_, 0.05);
        nh_.param("K_ffc_static", K_ffc_static, 0.0);
        nh_.param("K_ffc_dynamic", K_ffc_dynamic, 0.0);
        nh_.param("goal_dist_tolerance",goal_dist_tolerance_,0.5);
        nh_.param("far_goal_dist_tolerance",far_goal_dist_tolerance_,0.6);
        nh_.param("far_far_goal_dist_tolerance",far_far_goal_dist_tolerance_,1.5);
        nh_.param("set_yaw_speed",set_yaw_speed_,0.8);
        nh_.param("far_dist",far_dist,0.20);
        nh_.param("far_far_dist",far_far_dist,0.10);
        nh_.param("set_max_yaw_speed",set_max_yaw_speed_,1.0);
        nh_.param("alpha",alpha_,0.10);

        pid_init(pid_typedef, pid_p_, pid_i_, pid_d_, K_ffc_static, K_ffc_dynamic, i_max_, max_speed_);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("pid_cmd_vel", 10);
        target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("target_pose",10);
        path_sub_ = nh_.subscribe("/move_base1/NavfnROS/plan", 10, &PIDPathFollower::pathCallback, this);
        pidThread_ = std::thread(&PIDPathFollower::pid_thread, this);
        robot_pose_sub_ = nh_.subscribe("/odom", 10, &PIDPathFollower::odom_callback, this);
        get_icp_change_sub_ = nh_.subscribe("map_to_odom",10, &PIDPathFollower::map_to_odom_callback, this);
        robot_pose_pub_=nh_.advertise<geometry_msgs::PoseStamped>("robot_pose",10);
        local_path_pub_=nh_.advertise<nav_msgs::Path>("pid_local_path",10);
        pid_clear_costmap_pub_=nh_.advertise<std_msgs::Bool>("pid_clear_costmap",10);
        move_base_start_sub_ = nh_.subscribe("MY_ICP/move_base_start", 10, &PIDPathFollower::moveBaseStartCallback, this);
        velocity_marker_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("velocity_marker", 10);
    }

    ~PIDPathFollower() {
    }

    void TebpathCallback(const nav_msgs::Path::ConstPtr& msg) {
        Teb_path_ = *msg;
    }

    void moveBaseStartCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data) {  // 如果消息为 true
            nav_begin_=true;
        }
    }

    void smcmd_vel_callback(const geometry_msgs::Twist& smooth_cmd_vel)
    {
	    smooth_cmd_vel_x=smooth_cmd_vel.linear.x;
	    smooth_cmd_vel_y=smooth_cmd_vel.linear.y;

    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // boost::mutex::scoped_lock lock(path_mutex_);
        global_path_ = *msg;
        prune_index_ = 0;
        plan_count++;
        plan_ = true;
    }

void map_to_odom_callback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
        // 归一化接收到的四元数
        tf2::Quaternion quat;
        tf2::fromMsg(msg->transform.rotation, quat);
        quat.normalize();
        
        // 更新变换信息
        map_to_odom_transform = *msg;
        map_to_odom_transform.transform.rotation = tf2::toMsg(quat);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        boost::mutex::scoped_lock lock(odom_mutex_);
        // 直接将里程计数据中的 pose 部分作为机器人位置
        robot_pose_.pose.position.x = msg->pose.pose.position.x;
        robot_pose_.pose.position.y = msg->pose.pose.position.y;
        robot_pose_.pose.position.z = msg->pose.pose.position.z;
        robot_pose_.pose.orientation = msg->pose.pose.orientation;
        robot_pose_.header.frame_id = "odom";  
        robot_pose_.header.stamp = ros::Time::now();
    // // 获取当前四元数
    // geometry_msgs::Quaternion current_orientation = robot_pose_.pose.orientation;
    
    // // 将当前四元数转换为 tf2::Quaternion 对象
    // tf2::Quaternion quat;
    // tf2::fromMsg(current_orientation, quat);

    // // 将四元数转换为欧拉角 (roll, pitch, yaw)
    // double roll, pitch, yaw;
    // tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // // 将调整后的欧拉角转换回四元数
    // tf2::Quaternion new_quat;
    // new_quat.setRPY(roll, pitch, yaw);

    // // 将新的四元数转换回 geometry_msgs::Quaternion
    // robot_pose_.pose.orientation = tf2::toMsg(new_quat);
    robot_pose_pub_.publish(robot_pose_);
    }

void SmoothPathSegment(nav_msgs::Path& prior_path, nav_msgs::Path& smoothed_path, const nav_msgs::Path& global_path, const geometry_msgs::PoseStamped& robot_pose, double distance_threshold) {
    double accumulated_distance = 0.0;
    int start_index = 0;

    // 找到机器人最近的路径点
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < global_path.poses.size(); ++i) {
        double distance = GetEuclideanDistance(global_path.poses[i], robot_pose);
        if (distance < min_distance) {
            min_distance = distance;
            start_index = i;
        }
    }
    
    //----------------------------------------卡顿也可能是这个原因造成的，后续修改--------------------------------------//
    // 截取距离机器人distance_threshold内的路径
    prior_path.poses.clear(); // 清空prior_path
    smoothed_path.poses.clear(); // 清空smoothed_path
    for (int i = start_index; i < global_path.poses.size(); ++i) {
        if (i == start_index) {
            prior_path.poses.push_back(global_path.poses[i]);
        } else {
            accumulated_distance += GetEuclideanDistance(global_path.poses[i-1], global_path.poses[i]);
            if (accumulated_distance > distance_threshold) {
                break;
            }
            prior_path.poses.push_back(global_path.poses[i]);
        }
    }

    // 设置prior_path和smoothed_path的header
    prior_path.header.frame_id = "odom";
    prior_path.header.stamp = ros::Time::now();
    smoothed_path.header.frame_id = "odom";
    smoothed_path.header.stamp = ros::Time::now();

    // 平滑处理路径
    GenTraj(prior_path, smoothed_path);

    // 发布prior_path
    local_path_pub_.publish(smoothed_path);
}

    void pid_thread()
    {
        followPath();
    }

/*-------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------*/
void followPath() {
        ros::Rate loop_rate(100); // 设置循环频率为100Hz
        ros::Time time = ros::Time::now();
        static float plan_check,last_plan_count;
if(nav_begin_)
{
        while (ros::ok()) {
        //检测路径存在看门狗（doge  一次60大循环内路线必须更新喂狗 不然就清除代价地图一次
        if(++plan_check>=150 && plan_)
        {
            plan_check=0;
            if(global_path_.poses.empty())
            {
                pid_clear_costmap_msg_.data=true;
                pid_clear_costmap_pub_.publish(pid_clear_costmap_msg_);
            }
            else
            {
                pid_clear_costmap_msg_.data=false;
                pid_clear_costmap_pub_.publish(pid_clear_costmap_msg_);
            }
            ROS_WARN("PID CHECK!!!");  //pid看门狗叫
        }

        // 获取当前时间
        ros::Time time = ros::Time::now();

        // 使用变换后的机器人位置计算 PID 控制指令
        geometry_msgs::Twist pid_cmd_vel;
        if(plan_ && !global_path_.poses.empty())   //可以开始plan
        {        
        if (GetEuclideanDistance(robot_pose_,global_path_.poses.back())<= far_goal_dist_tolerance_){
                pid_cmd_vel.linear.x = 0;
                pid_cmd_vel.linear.y = 0;
                pid_cmd_vel.angular.z = set_yaw_speed_;
                ROS_INFO("Planning Success!");
                // return ;
        }
        SmoothPathSegment(prune_path_,local_path_,global_path_,robot_pose_,goal_dist_tolerance_);
        calculatePID(robot_pose_,local_path_,pid_cmd_vel);            
        }

        else
        {
        pid_cmd_vel.linear.x = 0;
        pid_cmd_vel.linear.y = 0;
        pid_cmd_vel.angular.z = set_yaw_speed_;
        }

        cmd_vel_pub_.publish(pid_cmd_vel);
        // 发布目标位置和变换后的机器人位置
        // target_pose_pub_.publish(target_pose); // 假设 target_pose 是全局变量或以其他方式获得的
        robot_pose_pub_.publish(robot_pose_);
        
        // 等待下一个循环
        loop_rate.sleep();
    }
}
}

/*-------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------*/
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber teb_path_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher target_pose_pub_;
    ros::Publisher robot_pose_pub_;
    ros::Publisher pid_clear_costmap_pub_;
    ros::Subscriber get_icp_change_sub_;
    ros::Subscriber sub_teb;
    ros::Publisher local_path_pub_;
    ros::Subscriber move_base_start_sub_;
    ros::Publisher vel_pub_;
    ros::Publisher velocity_marker_pub_;
    ros::Publisher navigation_pub;
    nav_msgs::Path global_path_;
    nav_msgs::Path Teb_path_;
    nav_msgs::Path prune_path_, local_path_;
    boost::mutex path_mutex_;
    boost::mutex odom_mutex_;
    double set_max_yaw_speed_;
    bool plan_;
    bool nav_begin_;
    std_msgs::Bool pid_clear_costmap_msg_;
    double alpha_;
    geometry_msgs::TransformStamped map_to_odom_transform;
    double far_dist,far_far_dist;
    float plan_count;
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped far_target_pose;
    geometry_msgs::PoseStamped far_far_target_pose;

    geometry_msgs::PoseStamped robot_pose_;
    double max_speed_,set_yaw_speed_;
    double pid_p_, pid_i_, pid_d_,yaw_;
    double smooth_cmd_vel_x,smooth_cmd_vel_y;
    double i_max_, K_ffc_static, K_ffc_dynamic,goal_dist_tolerance_,far_goal_dist_tolerance_,far_far_goal_dist_tolerance_;
    ros::Time last_time_;
    pid_struct_t pid_typedef;
    tf::TransformListener listener;
    std::thread pidThread_;
    int prune_index_;


void calculatePID(const geometry_msgs::PoseStamped& robot_pose,
                        const nav_msgs::Path& traj,
                         geometry_msgs::Twist& cmd_vel) {
        static float last_linear_vx,last_linear_vy,last_angular_vw;
        geometry_msgs::PoseStamped yaw_pose;
        static bool get_target;
// double diff_yaw = GetYawFromOrientation(traj.poses[0].pose.orientation)- GetYawFromOrientation(robot_pose.pose.orientation);
            
            double diff_y = traj.poses[1].pose.position.y-robot_pose.pose.position.y;
            double diff_x = traj.poses[1].pose.position.x-robot_pose.pose.position.x;  //弥补静态映射的偏差
            
            double diff_yaw = atan2((traj.poses[1].pose.position.y-robot_pose.pose.position.y ),( traj.poses[1].pose.position.x-robot_pose.pose.position.x));

            double diff_distance = GetEuclideanDistance(robot_pose,traj.poses[1]);
            double yaw_error,last_yaw_error,target_yaw;

        // yaw_ = tf::getYaw(robot_pose.pose.orientation);

        // set it from -PI t
        // if(yaw_ > M_PI){
        //     yaw_ -= 2*M_PI;
        // } else if(yaw_ < -M_PI){
        //     yaw_ += 2*M_PI;
        // }
        
        // set it from -PI t
        if(diff_yaw > M_PI){
            diff_yaw -= 2*M_PI;
        } else if(diff_yaw < -M_PI){
            diff_yaw += 2*M_PI;
        }

        // double vx_global = max_speed_*cos(diff_yaw);//*diff_distance*p_value_;  //这里直接采用最大速度原因是，为了追求追踪性能，FAST!!!
        // //采用跟踪位置不如直接对速度方向进行处理
        // double vy_global = max_speed_*sin(diff_yaw);//*diff_distance*p_value_;

               double vx_global =pid_calc(pid_typedef,diff_x,0);
               double vy_global =pid_calc(pid_typedef,diff_y,0);

        // double vx_global = cos(diff_yaw)*pid_p_*diff_distance*pid_p_;
        // double vy_global = sin(diff_yaw)*pid_p_*diff_distance*pid_p_;

        ros::Time now = ros::Time::now();
        double dt = (now - last_time_).toSec();
        last_time_ = now;
        
        yaw_pose = prune_path_.poses.back();
        target_pose_pub_.publish(yaw_pose);

        // 计算Yaw误差
        // target_yaw = atan2(yaw_pose.pose.position.y - robot_pose.pose.position.y, yaw_pose.pose.position.x - robot_pose.pose.position.x);
        // yaw_error = -(target_yaw-yaw_-2+0.8415);
    
        // // set it from -PI t
        // if(yaw_error > M_PI){
        //     yaw_error -= 2*M_PI;
        // } else if(yaw_error < -M_PI){
        //     yaw_error += 2*M_PI;
        // }

        // //设立伪车头
        // if(abs(yaw_error)>2.0)
        // yaw_error = -(target_yaw-yaw_+(M_PI-0.8415));  

        // // set it from -PI t
        // if(yaw_error > M_PI){
        //     yaw_error -= 2*M_PI;
        // } else if(yaw_error < -M_PI){1

        // yaw_error = (1-0.2)*yaw_error + 0.2*last_yaw_error;
        // last_yaw_error = yaw_error;
 
        // 更新Yaw误差
        // cmd_vel.angular.z = pid_calc(pid_typedef,yaw_error,0);
        // cmd_vel.angular.z = LIMIT_MIN_MAX(cmd_vel.angular.z,-set_max_yaw_speed_,set_max_yaw_speed_); // 直接使用yaw_error更新
        
        // set it from -PI t
        if(yaw_ > M_PI){
            yaw_ -= 2*M_PI;
        } else if(yaw_ < -M_PI){
            yaw_ += 2*M_PI;
        }
 
        cmd_vel.linear.x = vx_global;
        cmd_vel.linear.y = vy_global;
        // 采用上位机计算世界坐标系分解速度
        // cmd_vel.linear.x = (vx_global * cos(yaw_) + vy_global * sin(yaw_));
        // cmd_vel.linear.y = (- vx_global * sin(yaw_) + vy_global * cos(yaw_));
        
        cmd_vel.linear.x = (1-alpha_)*cmd_vel.linear.x+alpha_*last_linear_vx;
        cmd_vel.linear.y = (1-alpha_)*cmd_vel.linear.y+alpha_*last_linear_vy;

        //根据yaw轴偏差 进行减速
        // cmd_vel.linear.x = cmd_vel.linear.x/abs(cmd_vel.linear.x) * fmax(abs(cmd_vel.linear.x)-0.5*abs(yaw_error),0);
        // cmd_vel.linear.y = cmd_vel.linear.y/abs(cmd_vel.linear.y) * fmax(abs(cmd_vel.linear.y)-0.5*abs(yaw_error),0);

        // 可视化速度方向
        geometry_msgs::PoseStamped velocity_marker;
        velocity_marker.header.frame_id = "odom";
        velocity_marker.header.stamp = ros::Time::now();
        velocity_marker.pose.position = robot_pose.pose.position;
        tf2::Quaternion vel_quat;
        vel_quat.setRPY(0, 0, atan2(cmd_vel.linear.y, cmd_vel.linear.x));
        vel_quat.normalize();
        velocity_marker.pose.orientation = tf2::toMsg(vel_quat);
        vel_pub_.publish(velocity_marker);

        if (GetEuclideanDistance(robot_pose,global_path_.poses.back())<= far_far_goal_dist_tolerance_){
                cmd_vel.linear.x = vx_global*GetEuclideanDistance(robot_pose,global_path_.poses.back())/2;
                cmd_vel.linear.y = vy_global*GetEuclideanDistance(robot_pose,global_path_.poses.back())/2;
                cmd_vel.angular.z = set_yaw_speed_ * (1.0 - GetEuclideanDistance(robot_pose,global_path_.poses.back()));
        }

            last_linear_vx = cmd_vel.linear.x;
            last_linear_vy = cmd_vel.linear.y;
            last_angular_vw = cmd_vel.angular.z;

#ifdef ifdebug_
            ROS_INFO("yaw_: %f",yaw_);
            ROS_INFO("target_yaw: %f",target_yaw);
            ROS_INFO("yaw_error: %f",yaw_error);
            ROS_INFO("diff_yaw: %f", diff_yaw);
            ROS_INFO("diff_x: %f", diff_x);
            ROS_INFO("diff_y: %f", diff_y);
            ROS_INFO("vx_global: %f", vx_global);
            ROS_INFO("vy_global: %f", vy_global);
#endif
        } 
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pid_path_follower");
    PIDPathFollower follower;
    ros::spin();
    return 0;
}
