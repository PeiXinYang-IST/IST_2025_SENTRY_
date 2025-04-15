#include <Obstacle_cloudget.h>

Obstacle_cloud_get::Obstacle_cloud_get():
nh_("~"),getmap_(false),get_transform(false),
incoming_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
processed_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
cloudTarget(new pcl::PointCloud<pcl::PointXYZ>),
map_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
transformed_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
cloud_filtered_radius(new pcl::PointCloud<pcl::PointXYZ>),
cloud_removed(new pcl::PointCloud<pcl::PointXYZ>),
real_obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>),
kdmeans_cloud(new pcl::PointCloud<pcl::PointXYZ>),
real_obstacle_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>),
world_Obstacle_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
pursuit_mode_enabled(false),
tf_listener_(tf_buffer_)
{
    incoming_cloud_sub_ = nh_.subscribe("/xyz_processed_cloud", 1, &Obstacle_cloud_get::pointCloudCallback, this);    
    removal_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("real_obstacle_cloud", 1);
    prior_map_sub_ = nh_.subscribe("/map", 1, &Obstacle_cloud_get::mappointCloudCallback, this);   
    incoming_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("incoming_cloud", 1);
    pub_prior_map_ = nh_.advertise<sensor_msgs::PointCloud2>("prior_map", 1);
    Obstacle_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 1);
    yaw_pub_ = nh_.advertise<std_msgs::Float32>("yaw_angle", 1);
    pursuit_position_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pursuit_position", 1);
    pursuit_pub_ = nh_.advertise<geometry_msgs::Point>("pursuit", 1);
    odom_sub_ = nh_.subscribe("/odom", 1, &Obstacle_cloud_get::odomCallback, this);
    map_to_odom_sub_ = nh_.subscribe("/MY_ICP/map_to_odom", 1 ,&Obstacle_cloud_get::transformCallback, this);
    world_obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("world_obstacle_cloud", 1);
    grid_map_sub_ = nh_.subscribe("/prior_map",  1, &Obstacle_cloud_get::gridmapCallback, this);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    kdmeans_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("kdmeans_cloud", 1);
    navigation_mode_sub = nh_.subscribe<std_msgs::Bool>("navigation_mode", 10, &Obstacle_cloud_get::navigationModeCallback, this);
    nearest_centroid = pcl::PointXYZ(0, 0, 0); // 显式初始化
    Init_params();
    icp_transform_ = Eigen::Matrix4f::Identity();
}


void Obstacle_cloud_get::navigationModeCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data == PURSUIT) {
        ROS_INFO("PURSUIT mode enabled");
        pursuit_mode_enabled = true;
    }
    else
    {
        ROS_INFO("PURSUIT mode disabled");
        pursuit_mode_enabled = false;
    }
}


void Obstacle_cloud_get::gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) // 获取地图信息
{
        // 获取地图信息
        float resolution = msg->info.resolution;
        float origin_x = msg->info.origin.position.x;
        float origin_y = msg->info.origin.position.y;
        int width = msg->info.width;
        int height = msg->info.height;

        const std::vector<int8_t>& data = msg->data;

        // 清空点云数据
        world_Obstacle_cloud_->points.clear();
        world_Obstacle_cloud_->width = 0;
        world_Obstacle_cloud_->height = 8;  
 
        // 遍历地图数据，找到值为100的点（障碍物）
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int idx = y * width + x;
                if (data[idx] == 100) {  // 如果是障碍物
                    // 计算点的世界坐标
                    pcl::PointXYZ point;
                    point.x = origin_x + x * resolution;
                    point.y = origin_y + y * resolution;
                    point.z = 0.0;  // 假设障碍物的高度为0
                    world_Obstacle_cloud_->points.push_back(point); 
                    point.z = 0.05;
                    world_Obstacle_cloud_->points.push_back(point);
                    point.z = 0.1;
                    world_Obstacle_cloud_->points.push_back(point);
                    point.z = 0.15;
                    world_Obstacle_cloud_->points.push_back(point);
                    point.z = 0.2;
                    world_Obstacle_cloud_->points.push_back(point);
                    point.z = 0.25;
                    world_Obstacle_cloud_->points.push_back(point);
                    point.z = 0.3;
                    world_Obstacle_cloud_->points.push_back(point);
                    point.z = 0.35;
                    world_Obstacle_cloud_->points.push_back(point);

                    world_Obstacle_cloud_->width += 1;
                }
            }
        }

        // 膨胀处理
        // pcl::PointCloud<pcl::PointXYZ>::Ptr inflated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // for (const auto& point : world_Obstacle_cloud_->points) {
        //     for (float dx = -0.15; dx <= 0.15; dx += 0.05) { // Adjust the range and step as needed
        //     for (float dy = -0.15; dy <= 0.15; dy += 0.05) { // Adjust the range and step as needed
        //         pcl::PointXYZ inflated_point = point;
        //         inflated_point.x += dx;
        //         inflated_point.y += dy;
        //         inflated_cloud->points.push_back(inflated_point);
        //     }
        //     }
        // }
        // *world_Obstacle_cloud_ = *inflated_cloud;
        // 如果找到了障碍物，则发布点云
        if (world_Obstacle_cloud_->width > 0) {
            // 转换为 PointCloud2 格式
            sensor_msgs::PointCloud2 output_cloud;
            pcl::toROSMsg(*world_Obstacle_cloud_, output_cloud);
            output_cloud.header.stamp = ros::Time::now();
            output_cloud.header.frame_id = "odom";  // 坐标系
        } else {
            ROS_WARN("No obstacles found in the map!");
        }
}

void Obstacle_cloud_get::Init_params()
{
    nh_.param<float>("kdtree_search_radius",kdtree_search_radius_,0.02);
    nh_.param<float>("filter_search_radius",filter_search_radius_,0.15);
    nh_.param<float>("save_obstacle_cloud_time",save_obstacle_cloud_time_,1.5);
    nh_.param<float>("clear_x",clear_x_,0.35);
    nh_.param<float>("clear_y",clear_y_,0.35);
    nh_.param<float>("dynamic_inflate_distance",dynamic_inflate_distance_,0.1);
    nh_.param<float>("static_inflate_distance",static_inflate_distance_,0.35);
    nh_.param<bool>("icp_success",icp_success_,0);
    transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()).toRotationMatrix();
}


//获取icp配准之后的矩阵
void Obstacle_cloud_get::transformCallback(const geometry_msgs::TransformStampedConstPtr& input)
{
    // ROS_WARN("Get Transform!!!");

    static Eigen::Vector3f last_trans = Eigen::Vector3f::Identity();
    // 从ROS消息中提取旋转和位置
    
    Eigen::Quaternionf quat(
        input->transform.rotation.w,
        input->transform.rotation.x,
        input->transform.rotation.y,
        input->transform.rotation.z
    );

    Eigen::Vector3f trans(
        input->transform.translation.x,
        input->transform.translation.y,
        input->transform.translation.z
    );

    // 将四元数赋值给旋转部分
    icp_transform_.block<3,3>(0,0) = quat.toRotationMatrix();

    // 将位置赋值给平移部分
    icp_transform_.block<3,1>(0,3) = trans;
if(trans.x() != 0 && trans.y() != 0)
    get_transform = true;
}

void Obstacle_cloud_get::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::fromROSMsg(*input, *incoming_cloud_); 

    // for (auto& point : incoming_cloud_->points) {
    //     point.z = 0.0;
    // }

    // 膨胀处理
    // pcl::PointCloud<pcl::PointXYZ>::Ptr inflated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // for (const auto& point : cloud_filtered_radius->points) {
    //     for (float dx = -static_inflate_distance_; dx <= static_inflate_distance_; dx += 0.05) { // Adjust the range and step as needed
    //         for (float dy = -static_inflate_distance_; dy <= static_inflate_distance_; dy += 0.05) { // Adjust the range and step as needed
    //             pcl::PointXYZ inflated_point = point;
    //             inflated_point.x += dx;
    //             inflated_point.y += dy;
    //             inflated_cloud->points.push_back(inflated_point);
    //         }
    //     }
    // }
    // *processed_cloud_ = *inflated_cloud;

    if(getmap_ && get_transform)
    {
        PointCloudObstacleRemoval(map_cloud_,incoming_cloud_,kdtree_search_radius_);
        // ROS_WARN("BEGIN GET OBSTACLE!!!");
        //incoming_pub_.publish(input);
    }
}

void Obstacle_cloud_get::mappointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    static bool set_kdtree_map=false;
    pcl::fromROSMsg(*input, *map_cloud_);   

    // pcl::toROSMsg(*map_cloud_, incoming_cloud_msg);
    // incoming_cloud_msg.header.frame_id = "odom";
    // incoming_cloud_msg.header.stamp = ros::Time::now();
    // incoming_pub_.publish(incoming_cloud_msg); //发布实时雷达点云
    if(!set_kdtree_map && get_transform)
    {
    // pcl::transformPointCloud(*map_cloud_, *map_cloud_, icp_transform_);
    kdtree.setInputCloud(map_cloud_);
    set_kdtree_map=true;    
    getmap_ = true;
    ROS_WARN("get map");
    }
}

// 雷达位置
double real_yaw;
// 里程计数据的回调函数
void Obstacle_cloud_get::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    radar_position = Eigen::Vector3f(odom_msg->pose.pose.position.x,
                                     odom_msg->pose.pose.position.y,
                                     odom_msg->pose.pose.position.z);
    clear_distance_x = radar_position[0];
    clear_distance_y = radar_position[1];
    clear_distance_z = radar_position[2];

    // 获取yaw轴
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    real_yaw = yaw;

    // ROS_WARN("CLEAR_DISTANCE_X: %.2f, CLEAR_DISTANCE_Y: %.2f, CLEAR_DISTANCE_Z: %.2f, YAW: %.2f", clear_distance_x, clear_distance_y, clear_distance_z, yaw_angle);
}

/**
 * 对点云中障碍点进行剔除
 * cloud_map_msg为参考点云，cloud_msg为需要剔除障碍点的点云
 */
void Obstacle_cloud_get::PointCloudObstacleRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_map_msg, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_msg, 
                              double Distance_Threshold) {
    static bool get_yaw=false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::toROSMsg(*cloud_map_msg, prior_map_msg);
    prior_map_msg.header.frame_id = "odom";
    prior_map_msg.header.stamp = ros::Time::now();
    pub_prior_map_.publish(prior_map_msg);

    pcl::transformPointCloud(*cloud_msg, *cloud_processed, icp_transform_); // 应用变换
    // ROS_WARN("BEGIN GET OBSTACLE!!!");

    // pcl::toROSMsg(*cloud_icp_z, incoming_cloud_msg);
    // incoming_cloud_msg.header.frame_id = "map";
    // incoming_cloud_msg.header.stamp = ros::Time::now();
    // incoming_pub_.publish(incoming_cloud_msg); //发布实时雷达点云

	start_time=clock();

    //这里保存缓存点云，防止因为在视野盲区而无法计入sdf_map  具体缓存时间根据旋转速度而定
    static ros::Time last_time,current_time;
    current_time = ros::Time::now();
    if (current_time - last_time > ros::Duration(save_obstacle_cloud_time_)) {
        last_time = current_time;
        get_yaw=true;
        cloud_removed->clear();
    }
    real_obstacle_cloud->clear();
    real_obstacle_filtered_cloud->clear();

    int K = 1;  // 1-nearest neighbor search
    for (size_t i = 0; i < cloud_processed->points.size(); ++i) {
        pcl::PointXYZ searchPoint = cloud_processed->points[i];
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            if (pointNKNSquaredDistance[0] > Distance_Threshold) {  // If distance is greater than threshold, remove the point
                cloud_processed->erase(cloud_processed->begin() + i);   //这里进行了剔除
                --i;  // Decrement i because the size of the cloud has changed
                cloud_removed->push_back(searchPoint);  // Optionally store the removed point
                real_obstacle_cloud->push_back(searchPoint); 
            }
        }
    }


    cloud_filtered_radius->clear();
    //使用半径搜索滤波 去除噪点 
    if(!cloud_removed->empty())
    {
    // 设置输入点云
    outrem.setInputCloud(cloud_removed);
    // 设置搜索半径
    outrem.setRadiusSearch(filter_search_radius_);
    // 设置最小邻居数
    outrem.setMinNeighborsInRadius(10);
    // 执行滤波
    outrem.filter(*cloud_filtered_radius);
    }
	end_time=clock();

    cloud_filtered_radius->width = cloud_filtered_radius->points.size();
    cloud_filtered_radius->height = 1;
    cloud_filtered_radius->is_dense = false;  // contains nans

    // //对实时点云处理
    // for(int i=0;i<real_obstacle_cloud->size();i++)
    // {
    //     if(real_obstacle_cloud->points[i].x < fabs(clear_distance_x+30) && 
    //     real_obstacle_cloud->points[i].y < fabs(clear_distance_y+30) &&
    //     real_obstacle_cloud->points[i].z < clear_distance_z+2.5 && 
    //     real_obstacle_cloud->points[i].z > clear_distance_z-0.8)
    //     {
    //         // ROS_INFO("input_cloud : X:%f Y:%f Z:%f",input_cloud->points[i].x,input_cloud->points[i].y,input_cloud->points[i].z);
    //         real_obstacle_filtered_cloud->points.push_back(real_obstacle_cloud->points[i]);
    //     }
    // }

    // pcl::toROSMsg(*real_obstacle_cloud, real_obstacle_cloud_msg);
    // real_obstacle_cloud_msg.header.frame_id = "odom";
    // real_obstacle_cloud_msg.header.stamp = ros::Time::now();
    //removal_pointcloud_publisher_.publish(real_obstacle_cloud_msg);

    // pcl::toROSMsg(*cloud_filtered_radius, obstacle_cloud_msg);
    // obstacle_cloud_msg.header.frame_id = "odom";
    // obstacle_cloud_msg.header.stamp = ros::Time::now();
    // Obstacle_cloud_pub_.publish(obstacle_cloud_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto& point : cloud_filtered_radius->points) {
        if (point.z > 0.2 && point.z < 1.7) {
            for (float z = 0.0; z <= 0.35; z += 0.05) {
                pcl::PointXYZ filled_point = point;
                filled_point.z = z;
                filled_cloud->points.push_back(filled_point);
            }
        }
    }

    *cloud_filtered_radius = *filled_cloud;

    // 膨胀处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr inflated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud_filtered_radius->points) {
        for (float dx = -dynamic_inflate_distance_; dx <= dynamic_inflate_distance_; dx += 0.05) { // Adjust the range and step as needed
            for (float dy = -dynamic_inflate_distance_; dy <= dynamic_inflate_distance_; dy += 0.05) { // Adjust the range and step as needed
                pcl::PointXYZ inflated_point = point;
                inflated_point.x += dx;
                inflated_point.y += dy;
                inflated_cloud->points.push_back(inflated_point);
            }
        }
    }
    
    // *cloud_filtered_radius = *inflated_cloud;
    
    //transformed_cloud_->clear();
    // pcl::transformPointCloud(*world_Obstacle_cloud_, *transformed_cloud_, transform); // 应用变换
    // if(icp_success_)
    // {
    *transformed_cloud_ = *world_Obstacle_cloud_+*cloud_filtered_radius;
    *real_obstacle_cloud = *world_Obstacle_cloud_ + *inflated_cloud;
    // *transformed_cloud_ = *world_Obstacle_cloud_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_local(new pcl::PointCloud<pcl::PointXYZ>);
    radar_position.x() = radar_position.x() + icp_transform_(0, 3);
    radar_position.y() = radar_position.y() + icp_transform_(1, 3);
    for (int i = 0; i < transformed_cloud_->size(); i++) {
        float distance_x = transformed_cloud_->points[i].x - radar_position.x();
        float distance_y = transformed_cloud_->points[i].y - (radar_position.y()) ;
        if (std::abs(distance_x) <= clear_x_ && std::abs(distance_y) <= clear_y_) {
            // Skip points within 0.3m x 0.3m area around the robot
            continue;
        }
        cloud_filtered_local->points.push_back(transformed_cloud_->points[i]);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr real_cloud_filtered_local(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < real_obstacle_cloud->size(); i++) {
        float distance_x = real_obstacle_cloud->points[i].x - radar_position.x();
        float distance_y = real_obstacle_cloud->points[i].y - (radar_position.y()) ;
        if (std::abs(distance_x) <= clear_x_ && std::abs(distance_y) <= clear_y_) {
            // Skip points within 0.3m x 0.3m area around the robot
            continue;
        }
        real_cloud_filtered_local->points.push_back(real_obstacle_cloud->points[i]);
    }

    // }
    // else
    transformed_cloud_->clear();
    real_obstacle_cloud->clear();

    *transformed_cloud_ = *cloud_filtered_local;
    *real_obstacle_cloud = *real_cloud_filtered_local;

    // 创建VoxelGrid滤波器对象
    // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    // voxel_grid.setInputCloud(transformed_cloud_);
    // voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f); // 单位：m
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    // voxel_grid.filter(*cloud_filtered_voxel);

    pcl::toROSMsg(*transformed_cloud_, world_obstacle_msg);
    world_obstacle_msg.header.frame_id = "odom";
    world_obstacle_msg.header.stamp = ros::Time::now();
    world_obstacle_pub_.publish(world_obstacle_msg);

    pcl::toROSMsg(*real_obstacle_cloud, real_obstacle_cloud_msg);
    real_obstacle_cloud_msg.header.frame_id = "odom";
    real_obstacle_cloud_msg.header.stamp = ros::Time::now();
    removal_pointcloud_publisher_.publish(real_obstacle_cloud_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr move_cloud_filtered_local(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < inflated_cloud->size(); i++) {
        float distance_x = inflated_cloud->points[i].x - radar_position.x();
        float distance_y = inflated_cloud->points[i].y - (radar_position.y()) ;
        if (std::abs(distance_x) <= clear_x_ && std::abs(distance_y) <= clear_y_) {
            // Skip points within 0.3m x 0.3m area around the robot
            continue;
        }
        move_cloud_filtered_local->points.push_back(inflated_cloud->points[i]);
    }


    pcl::toROSMsg(*move_cloud_filtered_local, obstacle_cloud_msg);
    obstacle_cloud_msg.header.frame_id = "odom";
    obstacle_cloud_msg.header.stamp = ros::Time::now();
    Obstacle_cloud_pub_.publish(obstacle_cloud_msg);
    // if(!real_obstacle_cloud->empty())
    // obstacle_cloud_get_pose();
}

//使用点云聚类将障碍物点云聚类为具体位置
void Obstacle_cloud_get::obstacle_cloud_get_pose()
{
    if (real_obstacle_cloud->empty()) {
        ROS_WARN("No points in real_obstacle_cloud, skipping clustering.");
        return;
    }
    
    // 创建Kd树对象用于点云的搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(real_obstacle_cloud);

    // 设置聚类算法的参数
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1); // 10cm
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(real_obstacle_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);
    kdmeans_cloud->clear();

std::vector<pcl::PointXYZ> all_centroids; // 用于存储所有聚类的质心

for (const auto& indices : cluster_indices) {
    if (indices.indices.empty()) {
        ROS_WARN("Empty cluster, skipping centroid calculation.");
        continue;
    }

    // 计算当前聚类的质心
    pcl::PointXYZ centroid;
    centroid.x = 0.0;
    centroid.y = 0.0;
    centroid.z = 0.5; // 固定高度

    for (const auto& idx : indices.indices) {
        centroid.x += (*real_obstacle_cloud)[idx].x;
        centroid.y += (*real_obstacle_cloud)[idx].y;
    }

    centroid.x /= indices.indices.size();
    centroid.y /= indices.indices.size();
    
    all_centroids.push_back(centroid); // 将质心加入全局列表
}

if (all_centroids.empty()) {
    // ROS_WARN("No valid centroids found. Skipping pursuit position update.");
    return; // 无质心时提前退出
}

// 寻找距离雷达最近的质心
double min_distance = std::numeric_limits<double>::max();
pcl::PointXYZ nearest_centroid(0, 0, 0); // 显式初始化

for (const auto& centroid : all_centroids) {
    double distance = std::sqrt(
        std::pow(centroid.x - clear_distance_x, 2) +
        std::pow(centroid.y - clear_distance_y, 2)
    );

    if (distance < min_distance) {
        min_distance = distance;
        nearest_centroid = centroid;
    }
}

// 确保找到有效质心
if (min_distance == std::numeric_limits<double>::max()) {
    ROS_WARN("Failed to find nearest centroid.");
    return;
}

    pcl::PointXYZ odom_point;
    odom_point.x=clear_distance_x;
    odom_point.y=clear_distance_z;
    moving_average_filter(nearest_centroid);
    if(filtered_centroid_.x !=0 && filtered_centroid_.y !=0)
    calculateYawPitch();
    visual_centroid(filtered_centroid_);
}


void Obstacle_cloud_get::moving_average_filter(const pcl::PointXYZ& new_centroid) {
        centroid_history_.push_back(new_centroid);
        if (centroid_history_.size() > filter_window_size_) {
            centroid_history_.pop_front();
        }

        filtered_centroid_.x = 0.0;
        filtered_centroid_.y = 0.0;
        for (const auto& centroid : centroid_history_) {
            filtered_centroid_.x += centroid.x;
            filtered_centroid_.y += centroid.y;
        }
        filtered_centroid_.x /= centroid_history_.size();
        filtered_centroid_.y /= centroid_history_.size();
        filtered_centroid_.z = 0.5;

    pursuit_position.pose.position.x = nearest_centroid.x;
    pursuit_position.pose.position.y = nearest_centroid.y;
    pursuit_position.pose.position.z = 0.0;
    pursuit_position.pose.orientation.w = 1.0;
    pursuit_position.header.stamp = ros::Time::now();
    pursuit_position.header.frame_id = "odom";
    pursuit_position_pub_.publish(pursuit_position);
}

void Obstacle_cloud_get::calculateYawPitch() {
    float distance_x = filtered_centroid_.x - clear_distance_x;
    float distance_y = filtered_centroid_.y - clear_distance_y;
    // 检查距离是否过近
    const float distance_sq = distance_x * distance_x + distance_y * distance_y;
    if (distance_sq < 1e-6f) { // 1mm的平方阈值
        ROS_WARN("Target too close, skipping yaw calculation.");
        return;
    }
    // 计算当前角度
    float current_yaw = atan2(distance_y, distance_x);

    // 转换到度数并调整到[-180, 180]
    float yaw_deg = current_yaw * (180.0f / M_PI);
    yaw_deg += 75;
    if (yaw_deg > 180.0f) {
        yaw_deg -= 360.0f;
    } else if (yaw_deg < -180.0f) {
        yaw_deg += 360.0f;
    }

    yaw_msg.data = yaw_deg;
    yaw_pub_.publish(yaw_msg);
    // ROS_WARN("yaw_angle: %.2f degrees", yaw_deg);
}

void Obstacle_cloud_get::visual_centroid(const pcl::PointXYZ point)
{
     // 创建一个Marker消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";  // 设置参考坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "centroid";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;  // 设置标记类型为球体
    marker.action = visualization_msgs::Marker::ADD;  // 添加标记

    // 设置标记的位置
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;

    // 设置标记的方向（这里不需要设置，因为球体没有方向）
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // 设置标记的大小
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;

    // 设置标记的颜色
    marker.color.a = 1.0;  // 透明度
    marker.color.r = 1.0;  // 红色
    marker.color.g = 0.0;  // 绿色
    marker.color.b = 0.0;  // 蓝色

    // 设置标记的生命周期（0表示永久存在）
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
}


//这里pcl库中定义的点云就是boost::shared_ptr类型,可以自动释放内存，无需delete对象
Obstacle_cloud_get::~Obstacle_cloud_get() {
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Obstacle_cloudget");
    ROS_INFO("begin");
    Obstacle_cloud_get obstacle_cloud_get;
    ros::spin();
    return 0;
}