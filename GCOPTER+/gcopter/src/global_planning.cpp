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
#include <vector>
#include <gcopter/PolyTrajectory.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "gcopter/stc_gen.hpp"
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_utils/data_ros_utils.h>
#include <omp.h>
#include "gcopter/lbfgs.hpp"
#include "gcopter/minco.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl_conversions/pcl_conversions.h>
#include <plan_env/edt_environment.h>
#include "tf2/utils.h"
#include <std_msgs/UInt8.h>

struct Config
{
    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;
    double traj_sec;
    double corridor_distance;
    std::string Odomtopic;
    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("MaxVelMag", maxVelMag);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("MinThrust", minThrust);
        nh_priv.getParam("MaxThrust", maxThrust);
        nh_priv.getParam("VehicleMass", vehicleMass);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
        nh_priv.getParam("OdomTopic", Odomtopic);
        nh_priv.getParam("traj_sec", traj_sec);
        nh_priv.getParam("corridor_distance", corridor_distance);
    }
};

class GlobalPlanner
{
private:
    Config config;
    Eigen::Vector3d pos = Eigen::Vector3d::Zero(); // 初始化为(0,0,0)
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber odom_sub_;
    ros::Subscriber targetSub;
    ros::Subscriber path_sub_;
    ros::Publisher trajPub; 
    ros::Subscriber replan_sub;
    ros::Subscriber map_to_odom_sub;
    ros::Subscriber global_path_sub;
    ros::Subscriber move_obstacle_sub;
    ros::Publisher traj_pos_pub_;
    ros::Publisher stc_pub_;
    ros::Publisher pc_2d_pub_;
    ros::Publisher path_pub_;
    ros::Subscriber pursuit_distance_sub_;
    ros::Subscriber sub_;
    ros::Timer check_collision_pub;

    std::vector<Eigen::Vector3d> dijstra_pos;
    std::vector<Eigen::Vector3d> esdf_path_pos;
    std::vector<Eigen::Vector3d> route;
    double path_index = 0;
    double esdf_path_index = 0;
    double roll, pitch, yaw;
    bool mapInitialized;
    voxel_map::VoxelMap staticMap;
    voxel_map::VoxelMap dynamicMap;
    Visualizer visualizer;
    std::vector<Eigen::Vector3d> startGoal;
    std::vector<Eigen::Vector3d> move_base_point_set;
    Trajectory<5> traj;
    double trajStamp;
    minco::MINCO_S3NU opt_;
    ros::Time trajStartTime_;
    SDFMap::Ptr sdf_map_;
    bool receive_traj = false;
    double pursuit_distance;
    template<typename T>
    using VecE = std::vector<T,Eigen::aligned_allocator<T>>;
    fast_planner::EDTEnvironment::Ptr edt_environment_;

    
public:
  typedef unique_ptr<GlobalPlanner> Ptr;

    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          visualizer(nh)
    {
        
        // Initialize the map and dynamic map
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);
        staticMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);
        dynamicMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);
        mapSub = nh.subscribe(config.mapTopic, 1, &GlobalPlanner::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());
 
        targetSub = nh.subscribe(config.targetTopic, 1, &GlobalPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
        odom_sub_ = nh.subscribe("/odom", 1, &GlobalPlanner::odomCallback, this);
        trajPub = nh.advertise<gcopter::PolyTrajectory>("trajectory", 1); 
        replan_sub = nh.subscribe("/mpc_path_update",1, &GlobalPlanner::replanCallback, this);
        map_to_odom_sub = nh.subscribe("MY_ICP/map_to_odom", 1, &GlobalPlanner::mapToOdomCallback, this);
        traj_pos_pub_ = nh.advertise<visualization_msgs::MarkerArray>("traj_pos", 100);
        global_path_sub = nh.subscribe("/move_base1/NavfnROS/plan", 1, &GlobalPlanner::globalPathCallback, this);
        move_obstacle_sub = nh.subscribe("/Obstacle_cloudget/obstacle_cloud", 1, &GlobalPlanner::move_obstacle_callback, this);
        stc_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("stc_poly", 1);
        path_pub_ = nh.advertise<nav_msgs::Path>("dijstra_path", 10);
        pc_2d_pub_ = nh.advertise<sensor_msgs::PointCloud2>("pc_2d", 1);

        check_collision_pub = nh.createTimer(
            ros::Duration(0.2), 
            &GlobalPlanner::GCOPTER_check_collision, 
            this
        );

        sdf_map_.reset(new SDFMap);
        sdf_map_->initMap(nh);
        edt_environment_.reset(new fast_planner::EDTEnvironment);
        edt_environment_->setMap(sdf_map_);

        ROS_WARN("init edt");
    }
    
uint8_t pos_state;

bool PURSUE=false;
void gameStateCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    uint8_t pos_state = msg->data;
    if(pos_state!=7)
    PURSUE = true;

}

void GCOPTER_check_collision(const ros::TimerEvent &e)
{
  if(!receive_traj) return;
    // 遍历 traj 的点并查询 edt
    double query_interval = 0.1; // 查询间隔时间

    double total_duration = traj.getTotalDuration();

    for (double t = 0.1; t < total_duration; t += query_interval) {
      Eigen::Vector3d point = traj.getPos(t); // 获取轨迹点
      point.z() = 0.05;
      double dist = edt_environment_->evaluateCoarseEDT(point, -1.0); // 查询 edt 值
      //ROS_WARN("dist: %f", dist);
      if(dist < 0.0)
      {
        ROS_WARN("collision");
        plan();
      }
    }    
}

double last_distance;

//追击
void pursuit_distance_callback(const std_msgs::Float32::ConstPtr &msg)
{
    pursuit_distance = msg->data;
    //以当前pos为圆心 distance 
    last_distance = pursuit_distance-2;
    static bool pursuit_done=false;
    if(last_distance>0.5 && !pursuit_done)
    {
    last_distance = pursuit_distance-2;
    plan();
    pursuit_done=true;
    }
    else{
    last_distance=0;
    pursuit_done=false;
    }
}

void move_obstacle_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (msg->data.size() > 0)
    {
            dynamicMap.clear();
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }

                dynamicMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            // 进行膨胀操作
            dynamicMap.dilate(std::ceil(config.dilateRadius / dynamicMap.getScale()));
    }
}

bool get_global_path = false;
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

void publishTrajectory(const Trajectory<5>& traj) {
    if (traj.getPieceNum() == 0) {
        ROS_WARN("Empty trajectory, skipping publish.");
        return;
    }

    gcopter::PolyTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";

    // 填充持续时间
    Eigen::VectorXd durs = traj.getDurations();
    if (durs.size() != traj.getPieceNum()) {
        ROS_ERROR("Duration size mismatch!");
        return;
    }
    msg.durations.assign(durs.data(), durs.data() + durs.size());

    // 填充多项式系数
    int pieceNum = traj.getPieceNum();
    msg.coeffs_x.rows.resize(pieceNum); // 预分配空间
    msg.coeffs_y.rows.resize(pieceNum);
    msg.coeffs_z.rows.resize(pieceNum);

    for (int i = 0; i < pieceNum; ++i) {
        // 获取系数矩阵（3x6）
        Eigen::Matrix<double, 3, 6> coeffMat = traj[i].getCoeffMat();

        // 填充X方向系数
        for (int j = 0; j < 6; ++j) {
            msg.coeffs_x.rows[i].data[j] = coeffMat(0, j);
        }

        // 填充Y方向系数
        for (int j = 0; j < 6; ++j) {
            msg.coeffs_y.rows[i].data[j] = coeffMat(1, j);
        }

        // 填充Z方向系数
        for (int j = 0; j < 6; ++j) {
            msg.coeffs_z.rows[i].data[j] = coeffMat(2, j);
        }
    }
    trajPub.publish(msg);
}

Eigen::Affine3d transform = Eigen::Affine3d::Identity();

inline void mapToOdomCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
    if (mapInitialized)
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
}

inline void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!get_global_path)
    {
        return;
    }
    if (mapInitialized)
    {
        // 获取odom坐标系下的原始位姿
        Eigen::Vector3d odom_pos(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        
        // 转换到map坐标系
        // ROS_INFO_STREAM("Transform matrix: \n" << transform.matrix());
        // Eigen::Vector3d map_pos = transform * odom_pos;
        
        odom_pos.x()=dijstra_pos[0].x();
        odom_pos.y()=dijstra_pos[0].y();

        pos << odom_pos.x(), odom_pos.y(), 0.05; // 保持2D
        
        geometry_msgs::Quaternion current_orientation = msg->pose.pose.orientation;

        // 将当前四元数转换为 tf2::Quaternion 对象
        tf2::Quaternion quat;
        tf2::fromMsg(current_orientation, quat);

        // 将四元数转换为欧拉角 (roll, pitch, yaw)
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // 速度需要同时转换
        Eigen::Vector3d odom_vel(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z);
        // Eigen::Vector3d map_vel = transform.linear() * odom_vel;
        
        vel << odom_vel.x(), odom_vel.y(), 0.0;
        if(traj.getPieceNum() == 0)
        {
            plan();
            return;
        }
        // 检查轨迹是否过期
        const double delta = ros::Time::now().toSec() - trajStamp;
        Eigen::Vector3d traj_cur_pos = traj.getPos(delta);

        double dist_to_odom = (traj_cur_pos - odom_pos).norm();
        // ROS_INFO_STREAM("Distance to odom: " << dist_to_odom);
        if(dist_to_odom > 1)
        {
            //触发重规划
            ROS_WARN("Replan triggered due to large distance to odom!");
            plan();
        }
    }
}

    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // staticMap.clear(); // 新增清空地图方法
        if (!mapInitialized)
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                staticMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            staticMap.dilate(std::ceil(config.dilateRadius / staticMap.getScale()));
        
            mapInitialized = true;
        }

    }

    void Visua(VecE<Polyhedron<2>> &ploys_vis){
        decomp_ros_msgs::PolyhedronArray poly_msg = polyhedron_array_to_ros(ploys_vis, 1);
        stc_pub_.publish(poly_msg);
    }

    template <int Dim>
    decomp_ros_msgs::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs, int delta = 1){
        decomp_ros_msgs::PolyhedronArray msg;
        msg.header.frame_id = "odom";
        msg.header.stamp = ros::Time::now();
        int i = 1;
        for (const auto &v : vs){
            if(i == delta){
                msg.polyhedrons.push_back(DecompROS::polyhedron_to_ros(v)); 
                i = 0;   
            }
            i++;
        }
        return msg;
    }

    inline void plan()
    {
        if (startGoal.size() == 2)
        {
            const Eigen::Vector3d goal(startGoal[1].x(),
                                        startGoal[1].y(),
                                        0.05);

        if(staticMap.query(goal) != 0 || dynamicMap.query(goal) != 0){
        ROS_WARN("Goal point is occupied! Adjusting goal...");
        const double maxRadius = 1.5;  
        const double step = 0.3;       
        bool feasible = false;
        Eigen::Vector3d finalGoal = goal; // 存储最终目标点

        for (double radius = step; 
             radius <= maxRadius && !feasible; // 发现可行点时立即终止
             radius += step)
        {
            for (double angle = 0; 
                 angle < 2 * M_PI && !feasible; 
                 angle += M_PI / 8)
            {
                Eigen::Vector3d nearbyGoal = goal + 
                    Eigen::Vector3d(radius * cos(angle), 
                                  radius * sin(angle), 
                                  0.0);
                
                if (staticMap.query(nearbyGoal) == 0 && 
                    dynamicMap.query(nearbyGoal) == 0)
                {
                    ROS_WARN("Planning： Found FIRST feasible goal at radius: %.3fm, angle: %.1f rad", 
                            radius, angle);
                    finalGoal = nearbyGoal;
                    feasible = true;
                    break; // 跳出角度循环
                }
            }
        }

        if (feasible) {
            startGoal[1].x() = finalGoal.x();
            startGoal[1].y() = finalGoal.y();
            ROS_INFO("Adjusted goal to: (%.2f, %.2f)", 
                    finalGoal.x(), finalGoal.y());
        } else {
            ROS_ERROR("No feasible goal found within %.1fm radius!", maxRadius);
            return;
        }
    }
            
            // sfc_gen::planPath<voxel_map::staticMap>(pos,
            //                                        startGoal[1],
            //                                        staticMap.getOrigin(),
            //                                        staticMap.getCorner(),
            //                                        &staticMap, config.timeoutRRT,
            //                                        route);
    ROS_WARN("Planning...");

            sfc_gen::planPath_Djistra<voxel_map::VoxelMap>(pos,
                                                   startGoal[1],
                                                   staticMap.getOrigin(),
                                                   staticMap.getCorner(),
                                                   &staticMap, &dynamicMap, pos,config.timeoutRRT,
                                                   route);
                                                   
    ROS_WARN("Planning SUCCESS!");

        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "odom";

        for (const auto& point : route) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = point.z();
            path_msg.poses.push_back(pose);
        }

        path_pub_.publish(path_msg);

    esdf_path_pos.clear();

for(size_t i = 0; i< route.size(); i++){
      Eigen::Vector3d point = route[i];

      double dist = edt_environment_->evaluateCoarseEDT(point, -1.0);

      if(dist < 0.3)
      {
        Eigen::Vector3d grad;

          edt_environment_->evaluateEDTWithGrad(point, -1.0, dist, grad);
        
          if (grad.norm() > 1e-4) {
          grad.normalize();
          if(dist<0.1)
          point += grad * 0.25; // 推离0.2m
          else
          point += grad * 0.15; // 推离0.1m
          esdf_path_pos.push_back(point);
        }
      }
      else
        esdf_path_pos.push_back(point);
}

        route.clear();
        if(esdf_path_pos.size() > 0)
        {
            for (size_t i = 0; i < esdf_path_pos.size(); i++)
            {
                route.push_back(esdf_path_pos[i]);
            }
        }
        // Publish the route as a nav_msgs::Path

     //cost nearly 0.001s
            std::vector<Eigen::MatrixX4d> hPolys;
        //     std::vector<Eigen::Vector3d> static_pc;
        //     std::vector<Eigen::Vector3d> dynamic_pc;
        //     staticMap.getSurf(static_pc);
        //     dynamicMap.getSurf(dynamic_pc);

        //     std::vector<Eigen::Vector3d> pc;            
        //     pc.reserve(static_pc.size() + dynamic_pc.size());
        //     pc.insert(pc.end(), static_pc.begin(), static_pc.end());
        //     pc.insert(pc.end(), dynamic_pc.begin(), dynamic_pc.end());

            //cost nearly 0.2s
            if(route.size() == 0)
            {
                ROS_WARN("No route available.");
                return;
            }

//三维安全走廊生成
////////////////////////////////////////////////////////////////////////////////////////
            // sfc_gen::convexCover(route,
            //                      pc,
            //                      staticMap.getOrigin(),
            //                      staticMap.getCorner(),
            //                      config.traj_sec,
            //                      config.corridor_distance,
            //                      hPolys);
            // for (size_t i = 0; i < hPolys.size(); ++i) {
            //     ROS_INFO_STREAM("hPoly " << i << ":\n" << hPolys[i]);
            // }
            // sfc_gen::shortCut(hPolys);

///////////////////////////////////////////////////////////////////////////////////////

            std::vector<Eigen::Vector2d> path;

            path.reserve(route.size());
            for (const auto& point : route)
            {
                path.emplace_back(point.x(), point.y());
            }

            auto start_time = std::chrono::high_resolution_clock::now();

            auto corridors = stc_gen::STCGen::generateCorridors(path,edt_environment_);

            // auto corridors = stc_gen::STCGen::generateEDTCorridors(path, edt_environment_, 0.15, 1.0, 0.01);

            for (const auto& corridor : corridors) {
                const Eigen::MatrixX4d& hpoly3d = corridor.hpoly; 
    
            // 调试输出
            // ROS_INFO_STREAM("3D hpoly:\n" << hpoly3d);
            hPolys.push_back(hpoly3d);
            }

            sfc_gen::shortCut(hPolys);

            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_time = end_time - start_time;
            ROS_INFO_STREAM("Convex cover and shortcut computation took: " << elapsed_time.count() << " seconds.");

            if (route.size() > 1)
            {
                visualizer.visualizePolytope(hPolys);

                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << route.front(), vel, Eigen::Vector3d::Zero();
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                // initialize some constraint parameters
                Eigen::VectorXd magnitudeBounds(5);
                Eigen::VectorXd penaltyWeights(5);
                Eigen::VectorXd physicalParams(6);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                magnitudeBounds(3) = config.minThrust;
                magnitudeBounds(4) = config.maxThrust;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                penaltyWeights(4) = (config.chiVec)[4];
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                const int quadratureRes = config.integralIntervs;

                traj.clear();

                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    return;
                }
                ROS_WARN("Optimizing...");

                auto optimization_start = std::chrono::high_resolution_clock::now();
                if (std::isinf(gcopter.optimize(traj, config.relCostTol))) //solve
                {
                    return;
                }
                auto optimization_end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> optimization_time = optimization_end - optimization_start;
                ROS_INFO_STREAM("Optimization took: " << optimization_time.count() << " seconds.");

                if (traj.getPieceNum() > 0)
                {
                    receive_traj = true;
                    trajStamp = ros::Time::now().toSec();

                    // double dt=0.1;
                    // // Visualize desired state
                    // visualization_msgs::MarkerArray traj_marker_array;
                    // for (double i = 0.1; i < traj.getTotalDuration(); i += dt) {
                    //     visualization_msgs::Marker marker;
                    //     marker.header.frame_id = "odom";
                    //     marker.header.stamp = ros::Time::now();
                    //     marker.ns = "traj_pos";
                    //     marker.id = i*10;
                    //     marker.type = visualization_msgs::Marker::SPHERE;
                    //     marker.action = visualization_msgs::Marker::ADD;
                    //     marker.pose.position.x = traj.getPos(i)(0);
                    //     marker.pose.position.y = traj.getPos(i)(1);
                    //     marker.pose.position.z = 0.1;
                    //     marker.pose.orientation.x = 0.0;
                    //     marker.pose.orientation.y = 0.0;
                    //     marker.pose.orientation.z = 0.0;
                    //     marker.pose.orientation.w = 1.0;
                    //     marker.scale.x = 0.1;
                    //     marker.scale.y = 0.1;
                    //     marker.scale.z = 0.3;
                    //     marker.color.a = 1.0;
                    //     marker.color.r = 1.0;
                    //     marker.color.g = 1.0;
                    //     marker.color.b = 0.0;
                    //     traj_marker_array.markers.push_back(marker);
                    // }
                    // traj_pos_pub_.publish(traj_marker_array);

                    visualizer.visualize(traj, route);
                    publishTrajectory(traj); // 发布轨迹
                }
            }
        }
    }

inline void replanCallback(const std_msgs::Empty::ConstPtr &msg)
{
    if (mapInitialized)
    {
        //这里可以向前给0.15s 因为重规划+nmpc求解到下位机实现大概耗时0.15s
        ROS_WARN("Replan triggered!");
        // startGoal[1] = traj.getPos(ros::Time::now().toSec() - trajStamp + 0.15);
        plan();  // 立即触发规划
    }
}

inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (mapInitialized)
    {
        startGoal.clear();  // 清空历史目标
        
        // 添加当前位置作为起点
        startGoal.push_back(pos);
        
        // 添加新目标作为终点
        const double zGoal = 0.05;  // 保持与原始高度一致
        Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
        double dist = edt_environment_->evaluateCoarseEDT(goal, -1.0);
        ROS_WARN("goal esdf: %f", dist);

        if (staticMap.query(goal) == 0)
        {
            visualizer.visualizeStartGoal(goal, 0.5, 1);  // 第二个参数改为1表示终点
            startGoal.push_back(goal);

            plan();  // 立即触发规划
        }

        else
        {
            ROS_WARN("Target point is not feasible! Trying to find a nearby point...");
            // 在目标点周围逐渐增加半径搜索
            const double maxRadius = 1.5;  // 最大搜索半径
            const double step = 0.3;       // 每次增加的半径步长
            bool feasible = false;

            for (double radius = step; radius <= maxRadius; radius += step)
            {
                for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 8)
                {
                    Eigen::Vector3d nearbyGoal = goal + Eigen::Vector3d(radius * cos(angle), radius * sin(angle), 0.0);
                    if (staticMap.query(nearbyGoal) == 0)
                    {
                        ROS_WARN("Found feasible goal at radius: %f, angle: %f", radius, angle);
                        visualizer.visualizeStartGoal(nearbyGoal, 0.5, 1);  // 第二个参数改为1表示终点
                        startGoal.push_back(nearbyGoal);
                        plan();  // 立即触发规划
                        feasible = true;
                        break;
                    }
                }
                if (feasible)
                {
                    ROS_WARN("Infeasible Position Selected!");
                    break;
                }
            }
        }
    }
}

    inline void process()
    {
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        if (traj.getPieceNum() > 0)
        {
            const double delta = ros::Time::now().toSec() - trajStamp;
            if (delta > 0.0 && delta < traj.getTotalDuration())
            {
                double thr;
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;

                flatmap.forward(traj.getVel(delta),
                                traj.getAcc(delta),
                                traj.getJer(delta),
                                0.0, 0.0,
                                thr, quat, omg);
                double speed = traj.getVel(delta).norm();
                double bodyratemag = omg.norm();
                double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));
                std_msgs::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                speedMsg.data = speed;
                thrMsg.data = thr;
                tiltMsg.data = tiltangle;
                bdrMsg.data = bodyratemag;
                visualizer.speedPub.publish(speedMsg);
                visualizer.thrPub.publish(thrMsg);
                visualizer.tiltPub.publish(tiltMsg);
                visualizer.bdrPub.publish(bdrMsg);
                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh_;
    ROS_WARN("Global Planning Node started!");

    GlobalPlanner global_planner(Config(ros::NodeHandle("~")), nh_);

    ros::Rate lr(1000);
    while (ros::ok())
    {
        global_planner.process();
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
