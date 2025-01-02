#include <MY_ICP.h>

// #define ifdebug_ 

MY_ICP::MY_ICP() : nh_("~"), private_node_("~"),
                  prior_map_(new pcl::PointCloud<pcl::PointXYZ>),
                  rotated_lidar_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
                  incoming_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
                  lidar_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
                  map_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
                  transformed_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
                  merged_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
                  icp_start(false),icp_over(false),finish_cut_cloud_(false),
                  transformed(false), saved_PCD(false), need_relocalization(true),
                  find_times(false),restart(false) {
    Initparams();
    
    // 初始化 publishers and subscribers
    pub_prior_map = nh_.advertise<sensor_msgs::PointCloud2>("icp_prior_map", 1);
    pub_rotated_lidar_cloud = nh_.advertise<sensor_msgs::PointCloud2>("icp_rotated_lidar_map", 1);
    pub_incoming_cloud = nh_.advertise<sensor_msgs::PointCloud2>("incoming_cloud", 1);
    pub_transformed_cloud = nh_.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1);
    // 订阅主题
    pointcloud_sub = nh_.subscribe("/processed_cloud", 1, &MY_ICP::pointCloudCallback, this);    
    map_to_odom_pub = nh_.advertise<geometry_msgs::TransformStamped>("map_to_odom", 10);
    removal_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("removed_cloud", 1);
    move_base_start_pub_ = nh_.advertise<std_msgs::Bool>("move_base_start", 5);
    restart_all_pub = nh_.advertise<std_msgs::Bool>("restart", 5);
    local_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("local_cloud",1);
    icp_pub_ = nh_.advertise<std_msgs::Float32>("icp", 5);
    map_sub_ = nh_.subscribe("/map", 1, &MY_ICP::mappointCloudCallback, this);    
    odom_sub_ = nh_.subscribe("/odom", 1, &MY_ICP::odomCallback, this);
    // 启动发布线程
    publishThread_ = std::thread(&MY_ICP::publish_map_msg_thread, this);
}

//这里pcl库中定义的点云就是boost::shared_ptr类型,可以自动释放内存，无需delete对象
MY_ICP::~MY_ICP() {

}

double calculateDistance(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) {
    // 计算两点之间的差值向量
    Eigen::Vector3f diff = p2 - p1;

    // 返回差值向量的模（即欧几里得距离）
    return diff.norm();
}

// 里程计数据的回调函数
void MY_ICP::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    static Eigen::Vector3f last_radar_position = Eigen::Vector3f::Identity();
    radar_position = Eigen::Vector3f(odom_msg->pose.pose.position.x,
                                     odom_msg->pose.pose.position.y,
                                     odom_msg->pose.pose.position.z);
    clear_distance_x = radar_position[0];
    clear_distance_y = radar_position[1];
    clear_distance_z = radar_position[2];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local(new pcl::PointCloud<pcl::PointXYZ>);
    static bool cut_first_ = false;
    
    //两次更新局部点云之间里程计给定1.5m偏差
    if(calculateDistance(radar_position,last_radar_position)>1.5 || !cut_first_)
    {
        last_radar_position = radar_position;
        cloud_local = prior_map_deal(prior_map_);
        gicp.setInputTarget(cloud_local);   
        ROS_INFO("GET LOCAL CLOUD");
        finish_cut_cloud_ = true; 
        cut_first_ = true;
    }
}

void MY_ICP::mappointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
        pcl::fromROSMsg(*input, *map_cloud_);   
}
// NEXTE_Sentry_Nav
// IST_2025_sentry

//这里进行加载先验地图
//加载参数服务器
void MY_ICP::Initparams()
{
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/rm/catkin_livox_ros_driver2/src/IST_2025_sentry/sentry_slam/FAST_LIO_LOCALIZATION/PCD/demo.pcd", *prior_map_);
    initial_pose = Eigen::Matrix4f::Identity(); //这里要对initial_pose进行初始化，不然就会寄（doge
    private_node_.param<float>("find_min_angle",find_min_angle,9); //寻优最小角度设置 180/find_min_angle
    private_node_.param<float>("min_get_score",min_get_score,0.01); //最小获取寻优过程中增加的分数最小容忍度
    private_node_.param<float>("big_jump_yaw_score",big_jump_yaw_score,0.02); //在寻优过程中进行的大幅度跳跃分数阈值
    private_node_.param<float>("small_jump_yaw_score",small_jump_yaw_score,0.01); //在寻优过程中进行的小幅度跳跃分数阈值
    private_node_.param<float>("find_best_yaw_icp_Iterations",find_best_yaw_icp_Iterations,20); //寻优过程中迭代次数
    private_node_.param<float>("icp_Iterations",icp_Iterations,40); //正常icp最大迭代次数
    private_node_.param<float>("re_icp_score",re_icp_score,0.3); //重新进行重定位条件
    private_node_.param<float>("save_lidar_times",save_lidar_times,20); //保存实时雷达点云次数
    private_node_.param<float>("remove_cloud_length",remove_cloud_length,0.1); //移除雷达点云距离阈值
    private_node_.param<float>("clear_icp_global_costmap",clear_icp_global_costmap,5); //清除全局代价地图等待转换次数
    private_node_.param<float>("icp_correct",icp_correct,0.10); //icp纠正的分数匹配阈值
    private_node_.param<float>("local_pointcloud_x",local_pointcloud_x_,0.10); //局部点云地图x
    private_node_.param<float>("local_pointcloud_y",local_pointcloud_y_,0.10); //局部点云地图y
    private_node_.param<float>("local_pointcloud_z",local_pointcloud_z_,0.10); //局部点云地图y
    // gicp.setInputTarget(prior_map_);
    // ndt.setInputTarget(prior_map_);
}

//对全局先验地图进行局部点云分割，根据估计位置分割出局部点云进行icp配准
pcl::PointCloud<pcl::PointXYZ>::Ptr MY_ICP::prior_map_deal(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_local(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::toROSMsg(*input_cloud, global_pointcloud_msg_);
    global_pointcloud_msg_.header.frame_id = "odom";
    global_pointcloud_msg_.header.stamp = ros::Time::now();
    pcl::fromROSMsg(global_pointcloud_msg_, *global_cloud_);   

    for(int i=0;i<input_cloud->size();i++)
    {
        if(input_cloud->points[i].x < fabs(local_pointcloud_x_+radar_position.x()) && 
        input_cloud->points[i].y < fabs(local_pointcloud_y_+radar_position.y()) &&
        input_cloud->points[i].z < local_pointcloud_z_)
        {
            // ROS_INFO("input_cloud : X:%f Y:%f Z:%f",input_cloud->points[i].x,input_cloud->points[i].y,input_cloud->points[i].z);
            cloud_filtered_local->points.push_back(input_cloud->points[i]);
        }
    }

    pcl::toROSMsg(*cloud_filtered_local, local_pointcloud_msg);
    local_pointcloud_msg.header.frame_id = "odom";
    local_pointcloud_msg.header.stamp = ros::Time::now();
    local_pointcloud_pub_.publish(local_pointcloud_msg);
    return cloud_filtered_local;
}

//阻塞主线程寻优
void MY_ICP::findBestYawAngle_thread() {
    try {
        std::thread findbestyawThread(&MY_ICP::findBestYawAngle, this);
        findbestyawThread.join(); //阻塞主线程运行寻优
    } catch (const std::exception& e) {
        std::cerr << "Failed to create thread: " << e.what() << std::endl;
    }
}

void MY_ICP::publish_map_msg()
{
    static Eigen::Matrix4f now_transform = Eigen::Matrix4f::Identity();
    // Publish the point clouds
    pcl::toROSMsg(*rotated_lidar_cloud_, rotated_lidar_map_msg);
    rotated_lidar_map_msg.header.stamp = ros::Time::now();
    // move_base_start_pub_.publish(move_base_start_msg); //move_base启动导航节点
    restart_all_pub.publish(restart_msg);
    icp_pub_.publish(icp_msg);

    //icp看门狗
    if(++icp_transform_update>=50 && icp_start && need_relocalization)  //其实就是 50/25 = 2s 如果这2s之间icp未更新则说明lio飘飞 重启所有相关节点
    {
        ROS_WARN("ICP CHECK!!!");
        icp_transform_update=0;
        if(now_transform == transform && now_transform!=Eigen::Matrix4f::Identity())
        {
            restart_msg.data = true;
            restart_all_pub.publish(restart_msg);
        }
        else
            now_transform = transform;
    }
    publishTransform(icp_transform);
}

//启动一个线程来发布点云消息
void MY_ICP::publish_map_msg_thread()
{
    using namespace std::chrono_literals;
    while (ros::ok()) {
        publish_map_msg();
        // 休眠0.04秒  //手动补25fps
        std::this_thread::sleep_for(0.04s);
    }
}

//获取实时雷达点云 缓存一定次数的集合作为icp的input
void MY_ICP::get_lidar_cloud()
{
    static int lidar_collect_times;
    if(lidar_collect_times<=save_lidar_times)
    {
    // 将接收到的点云添加到全局合并后的点云中
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    *merged_cloud_ += *incoming_cloud_;
    lidar_collect_times++;
    std::cout << "lidar_collect_times:" << lidar_collect_times << std::endl;
    }
    if(!saved_PCD && lidar_collect_times>save_lidar_times) //保存并且进行体素滤波和离群点滤波处理
    {
    // 创建StatisticalOutlierRemoval滤波器对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(merged_cloud_);
    sor.setMeanK(10); // 设置每个点的邻近点数
    sor.setStddevMulThresh(1.0); // 设置标准偏差乘数阈值
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_sor(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_filtered_sor);

    // 创建VoxelGrid滤波器对象
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud_filtered_sor);
    voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f); // 单位：m
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.filter(*cloud_filtered_voxel);

    // 保存点云数据到PCD文件
    if (pcl::io::savePCDFile<pcl::PointXYZ>("/home/rm/catkin_livox_ros_driver2/src/IST_2025_sentry/sentry_slam/FAST_LIO_LOCALIZATION/PCD/my_lidar.pcd", *cloud_filtered_voxel) != -1) {
           std::cout << "save PCD!" << std::endl;
    }
        saved_PCD=true;
    }
}

//点云接收回调
void MY_ICP::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    using namespace std::chrono_literals;
    if(!transformed)
    pub_map_to_camera_init();

    pcl::fromROSMsg(*input, *incoming_cloud_);   

    PointCloudObstacleRemoval(prior_map_,incoming_cloud_,remove_cloud_length);
 
    if (incoming_cloud_->empty()) {
        std::cerr << "Received empty point cloud!" << std::endl;
        return;
    }
    if(!saved_PCD)
    get_lidar_cloud();

    if(saved_PCD && finish_cut_cloud_)
    {
    if(!icp_start)//创建单独寻优线程 仅在初始化时使用寻优
    {
        if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/rm/catkin_livox_ros_driver2/src/IST_2025_sentry/sentry_slam/FAST_LIO_LOCALIZATION/PCD/my_lidar.pcd", *lidar_cloud_)!=-1)
        std::cout << "get lidar PCD!" << std::endl;   
        findBestYawAngle_thread();
    }

    //这种写法在开启一段时间后自动崩溃，且重启的逻辑较为复杂，而且多线程在这里比较危险，不建议使用
    // try {
    //     icp_start = false;
    //     std::thread icp_thread(&MY_ICP::perform_icp_thread, this);
    //     icp_thread.detach(); // 让线程独立运行
    // } catch (const std::exception& e) {
    //     std::cerr << "Failed to create thread: " << e.what() << std::endl;
    // }
    // }

    if(icp_start && need_relocalization) {
        performRelocalization(initial_pose);
    }
}
}

void MY_ICP::perform_icp_thread()
{
    using namespace std::chrono_literals;
    while (ros::ok()) {
        performRelocalization(initial_pose);
        // std::this_thread::sleep_for(2s); //每2s进行一次icp配准进行重定位
    }
}

void MY_ICP::performRelocalization(const Eigen::Matrix4f& initial_pose){
    const std::lock_guard<std::mutex> lock(icp_thread_mutex); //锁住点云变量，不可外部访问
    static int icp_converged_times,icp_converged_HZ;
    double tranDist,angleDist;
    static float last_score;
    static bool icp_publish,icp_clear_costmap;
    pcl::toROSMsg(*prior_map_, prior_map_msg);
    rotated_lidar_map_msg.header.frame_id = "odom";
    pub_rotated_lidar_cloud.publish(rotated_lidar_map_msg); 
    PointCloud<PointXYZ> aligned;

    gicp.setInputSource(incoming_cloud_);
    gicp.setEuclideanFitnessEpsilon(1e-5);	// 设置收敛条件是均方误差和小于阈值，停止迭代;
    gicp.setMaximumIterations(icp_Iterations);			// 最大迭代次数

    // ndt.setInputSource(incoming_cloud_);
    // ndt.setEuclideanFitnessEpsilon(1e-5);	// 设置收敛条件是均方误差和小于阈值，停止迭代;
    // ndt.setMaximumIterations(icp_Iterations);			// 最大迭代次数

    ros::Time start_time=ros::Time::now();

    Eigen::Matrix4f current_transform = 
    transform.isApprox(Eigen::Matrix4f::Identity()) ? initial_pose : transform;

    gicp.align(aligned,current_transform); // 使用初始估计

    ros::Time end_time=ros::Time::now();
    double duration = (end_time - start_time).toSec();  // 计算持续时间，单位为秒
    ROS_WARN("ICP COST:%f",duration);
    if (gicp.hasConverged()) {
        // std::cout << "ICP converged with score: " << gicp.getFitnessScore() << std::endl;
        transform = gicp.getFinalTransformation(); // 获取变换矩阵   

        if(gicp.getFitnessScore()>re_icp_score && icp_converged_times>=15)
        {
            restart_msg.data = true;
            restart_all_pub.publish(restart_msg);
        }

        Eigen::Affine3f transform_3f = Eigen::Affine3f::Identity();
        transform_3f = gicp.getFinalTransformation(); // 获取变换矩阵
        
        if(gicp.getFitnessScore()>icp_correct || icp_converged_times<10)
        icp_transform = transform;

        transformed=true;
        icp_msg.data = gicp.getFitnessScore();
        //在进行几次变换之后而且分数较好时,设置开始move_base节点即清除全局代价地图一次 这里只发一次，在icp定位完成后代价地图的清除交给导航节点
        if(icp_converged_times >= clear_icp_global_costmap && last_score < 0.20 && !icp_clear_costmap)
        {
            icp_clear_costmap=true;
            move_base_start_msg.data=true;
            move_base_start_pub_.publish(move_base_start_msg);
            move_base_start_msg.data=false;
            move_base_start_pub_.publish(move_base_start_msg);

            icp_transform_update=0;
        }
        
        remove_cloud_length = fmin(abs(40*(0.01-last_score)),0.4);   //一般配准好的时候大概是 0.005~0.001 左右 50*0.008=0.4 根据icp配准情况进行梯度设置搜索距离
        remove_cloud_length = fmax(remove_cloud_length,0.1);
        // pcl::transformPointCloud(*incoming_cloud_, *transformed_cloud_, transform); // 应用变换

#ifdef ifdebug_ 
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transform_3f, x, y, z, roll, pitch, yaw);
        double tranDist = sqrt(x*x + y*y);
        double angleDist = abs(yaw);
        static double last_tranDist;
        last_tranDist=tranDist;
        std::cout << "tranDist:" << tranDist << " angleDist:" << angleDist << std::endl;
#endif
        last_score = gicp.getFitnessScore();
        icp_converged_times++;
    } else {
        restart_msg.data=true;
        restart_all_pub.publish(restart_msg);
        std::cerr << "ICP did not converge." << std::endl;
    }
    ros::Duration(0.1).sleep();  //10hz 的icp配准左右
}

void MY_ICP::publishTransform(const Eigen::Matrix4f& transform) {
        // 创建一个TransformStamped消息
    geometry_msgs::TransformStamped transformStamped;

    // 设置时间戳
    transformStamped.header.stamp = ros::Time::now();
    // 设置父坐标帧ID
    transformStamped.header.frame_id = "odom";
    // 设置子坐标帧ID
    transformStamped.child_frame_id = "camera_init";

    // 将Eigen::Matrix4f转换为geometry_msgs::Vector3和geometry_msgs::Quaternion
    Eigen::Affine3f eigen_affine(transform);
    transformStamped.transform.translation.x = eigen_affine.translation()(0);
    transformStamped.transform.translation.y = eigen_affine.translation()(1);
    transformStamped.transform.translation.z = eigen_affine.translation()(2);

    Eigen::Quaternionf quat(eigen_affine.rotation());
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    // 发布变换
    map_to_odom_pub.publish(transformStamped);

#ifdef ifdebug_ 
    std::cout << "Published transform" << std::endl;
    // 打印变换信息
    std::cout << "Transform from " << transformStamped.header.frame_id.c_str() 
    << " to " << transformStamped.child_frame_id.c_str() << ":" << std::endl;

    std::cout << "Translation: x = " << transformStamped.transform.translation.x 
    << ", y = " << transformStamped.transform.translation.y << ", z = " << transformStamped.transform.translation.z << std::endl;
    
    std::cout << "Rotation: x = " << transformStamped.transform.rotation.x 
    << ", y = " << transformStamped.transform.rotation.y << ", z = " << transformStamped.transform.rotation.z << ", w = " << transformStamped.transform.rotation.w << std::endl;
#endif

}

void MY_ICP::findBestYawAngle(){
    const std::lock_guard<std::mutex> lock(findbestyaw_thread_mutex);
    static float last_score;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_yaw;
    float best_score = std::numeric_limits<float>::max();
    float best_yaw = 0.0;
    std::cout << "begin find" << std::endl;

    for (float yaw = -M_PI; yaw <= M_PI; yaw += M_PI / find_min_angle) {
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        Eigen::Matrix3f rotation_matrix = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        transformation.block<3,3>(0,0) = rotation_matrix;
        pcl::transformPointCloud(*lidar_cloud_, *rotated_lidar_cloud_, transformation);
        PointCloud<PointXYZ> aligned;
        gicp_yaw.setInputSource(rotated_lidar_cloud_);
        gicp_yaw.setInputTarget(prior_map_);
        gicp_yaw.setEuclideanFitnessEpsilon(1e-5);	// 设置收敛条件是均方误差和小于阈值，停止迭代;
        gicp_yaw.setMaximumIterations(find_best_yaw_icp_Iterations);
        gicp_yaw.align(aligned);
        std::cout << "yaw:" << yaw << " with score: " << gicp_yaw.getFitnessScore() << std::endl;
        if (gicp_yaw.hasConverged()) {
            float score = gicp_yaw.getFitnessScore();
            if (score < best_score) {
                best_score = score;
                best_yaw = yaw;
            }

            if(score-last_score>min_get_score)
            {
                yaw += M_PI / find_min_angle;
            }
            last_score=score;
        }
        if(gicp_yaw.getFitnessScore()>big_jump_yaw_score)//这里如果效果不好直接开始跳跃角度提升速度
            yaw += M_PI / find_min_angle * 2;
        if(gicp_yaw.getFitnessScore()>small_jump_yaw_score)//小幅度跳跃
            yaw += M_PI / find_min_angle;
         // 将旋转后的先验地图点云转换为ROS消息
        pcl::toROSMsg(*rotated_lidar_cloud_, rotated_lidar_map_msg);
        rotated_lidar_map_msg.header.frame_id = "odom";
        rotated_lidar_map_msg.header.stamp = ros::Time::now();
        // 发布旋转后的先验地图点云
        pub_rotated_lidar_cloud.publish(rotated_lidar_map_msg);
    }


    std::cout << "Best yaw angle: " << best_yaw << " with score: " << best_score << std::endl;

    best_yaw_angle_ = best_yaw;

    // 设置初始位姿为 (0, 0, 0)  角度为最佳角度
    initial_pose.block<3, 3>(0, 0) = Eigen::AngleAxisf(best_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    initial_pose.block<3, 1>(0, 3) = Eigen::Vector3f(0, 0, 0); // 设置位置为 (0, 0, 0)
    icp_start=true;
}

/**
 * 对点云中障碍点进行剔除
 * cloud_map_msg为参考点云，cloud_msg为需要剔除障碍点的点云
 */
void MY_ICP::PointCloudObstacleRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_map_msg, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_msg, 
                              double Distance_Threshold) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed_z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp_z(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::toROSMsg(*map_cloud_, prior_map_msg);
    prior_map_msg.header.frame_id = "odom";
    prior_map_msg.header.stamp = ros::Time::now();
    pub_prior_map.publish(prior_map_msg);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    std::vector<int> indices_to_remove;

    // 滤除z轴恰当位置的点云
    pass_z.setInputCloud(cloud_msg);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(clear_distance_z+1.5,std::numeric_limits<float>::max()); // 距离雷达z坐标过高的点云滤除 人的身高左右
    pass_z.filter(*cloud_z); // 更新

for (size_t i = 0; i < cloud_z->points.size(); ++i) {
    // 查找原始点云中与滤波后的点相匹配的点，并标记它们的索引
    pcl::PointXYZ point = cloud_z->points[i];
    for (size_t j = 0; j < cloud_msg->points.size(); ++j) {
        if (cloud_msg->points[j].x == point.x &&
            cloud_msg->points[j].y == point.y &&
            cloud_msg->points[j].z == point.z) {
            indices_to_remove.push_back(j); // 记录该点的索引
        }
    }
}

// 从原始点云中删除这些点
for (size_t i = 0; i < cloud_msg->points.size(); ++i) {
    if (std::find(indices_to_remove.begin(), indices_to_remove.end(), i) == indices_to_remove.end()) {
        cloud_removed_z->points.push_back(cloud_msg->points[i]);
    }
}

    pcl::transformPointCloud(*cloud_removed_z, *cloud_icp_z, transform); // 应用变换   

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    
    kdtree.setInputCloud(cloud_map_msg);

    int K = 1;  // 1-nearest neighbor search
    for (size_t i = 0; i < cloud_icp_z->points.size(); ++i) {
        pcl::PointXYZ searchPoint = cloud_icp_z->points[i];
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            if (pointNKNSquaredDistance[0] > Distance_Threshold) {  // If distance is greater than threshold, remove the point
                cloud_icp_z->erase(cloud_icp_z->begin() + i);   //这里进行了剔除
                --i;  // Decrement i because the size of the cloud has changed
                cloud_removed->push_back(searchPoint);  // Optionally store the removed point
            }
        }
    }

    pcl::toROSMsg(*cloud_msg, incoming_cloud_msg);
    pub_incoming_cloud.publish(incoming_cloud_msg); //发布实时雷达点云

    cloud_removed->width = cloud_removed->points.size();
    cloud_removed->height = 1;
    cloud_removed->is_dense = false;  // contains nans

    pcl::toROSMsg(*cloud_removed, cloud_removed_msg);
    cloud_removed_msg.header.frame_id = "camera_init";
    cloud_removed_msg.header.stamp = ros::Time::now();
    // Assuming you have a publisher defined as `removal_pointcloud_publisher_`, publish the cloud_removed
    // pcl::toROSMsg(*cloud_removed, cloud_removed_msg);
    removal_pointcloud_publisher_.publish(cloud_removed_msg);
}

//防止报错的，在未进行icp之前发布空变换
void MY_ICP::pub_map_to_camera_init() {
    // 创建一个TransformStamped消息
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "camera_init";

    // 设置变换的平移和旋转部分
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1; // 单位四元数，表示没有旋转

    // 发布变换
    map_to_odom_pub.publish(transformStamped);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_node");
    MY_ICP My_ICP;
    ros::spin();
    return 0;
}

