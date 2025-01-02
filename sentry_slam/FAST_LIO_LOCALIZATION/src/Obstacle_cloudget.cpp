#include <Obstacle_cloudget.h>

Obstacle_cloud_get::Obstacle_cloud_get():
nh_("~"),getmap_(false),get_transform(false),
incoming_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
cloudTarget(new pcl::PointCloud<pcl::PointXYZ>),
map_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
transformed_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
world_Obstacle_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
    incoming_cloud_sub_ = nh_.subscribe("/processed_cloud", 1, &Obstacle_cloud_get::pointCloudCallback, this);    
    removal_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("removed_cloud", 1);
    prior_map_sub_ = nh_.subscribe("/map", 1, &Obstacle_cloud_get::mappointCloudCallback, this);   
    incoming_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("incoming_cloud", 1);
    pub_prior_map_ = nh_.advertise<sensor_msgs::PointCloud2>("prior_map", 1);
    Obstacle_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacle_cloud", 1);
    odom_sub_ = nh_.subscribe("Odometry", 1, &Obstacle_cloud_get::odomCallback, this);
    map_to_odom_sub_ = nh_.subscribe("/MY_ICP/map_to_odom", 1 ,&Obstacle_cloud_get::transformCallback, this);
    world_obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("world_obstacle_cloud", 1);
    grid_map_sub_ = nh_.subscribe("/prior_map",  1, &Obstacle_cloud_get::gridmapCallback, this);
    Init_params();

    icp_transform_ = Eigen::Matrix4f::Identity();
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
        world_Obstacle_cloud_->height = 9;  

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

                    // 将点添加到点云中
                    world_Obstacle_cloud_->points.push_back(point);
                    point.z = 0.1;  
                    world_Obstacle_cloud_->points.push_back(point);
                    point.z = 0.2;  
                    world_Obstacle_cloud_->points.push_back(point);
                    point.z = 0.3;  
                    world_Obstacle_cloud_->points.push_back(point);
                    point.z = 0.4;  
                    world_Obstacle_cloud_->points.push_back(point);      
                    point.z = 0.5;  
                    world_Obstacle_cloud_->points.push_back(point);                  
                    point.z = -0.3;  
                    world_Obstacle_cloud_->points.push_back(point);
                    point.z = -0.2;  
                    world_Obstacle_cloud_->points.push_back(point);      
                    point.z = -0.1;  
                    world_Obstacle_cloud_->points.push_back(point); 

                    world_Obstacle_cloud_->width += 1;
                }
            }
        }

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
    nh_.param<float>("kdtree_search_radius",kdtree_search_radius_,0.03);
    nh_.param<float>("filter_search_radius",filter_search_radius_,0.15);
    transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()).toRotationMatrix();
}

//获取icp配准之后的矩阵
void Obstacle_cloud_get::transformCallback(const geometry_msgs::TransformStampedConstPtr& input)
{
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

    get_transform = true;
}

void Obstacle_cloud_get::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::fromROSMsg(*input, *incoming_cloud_);   
    if(getmap_ && get_transform)
    {
        PointCloudObstacleRemoval(map_cloud_,incoming_cloud_,kdtree_search_radius_);
        incoming_pub_.publish(input);
    }
}

void Obstacle_cloud_get::mappointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    static bool set_kdtree_map=false;
    pcl::fromROSMsg(*input, *map_cloud_);   
    if(!set_kdtree_map)
    {
    kdtree.setInputCloud(map_cloud_);
    set_kdtree_map=true;
    }
    getmap_ = true;

}

// 里程计数据的回调函数
void Obstacle_cloud_get::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    radar_position = Eigen::Vector3f(odom_msg->pose.pose.position.x,
                                     odom_msg->pose.pose.position.y,
                                     odom_msg->pose.pose.position.z);
    clear_distance_x = radar_position[0];
    clear_distance_y = radar_position[1];
    clear_distance_z = radar_position[2];
}

/**
 * 对点云中障碍点进行剔除
 * cloud_map_msg为参考点云，cloud_msg为需要剔除障碍点的点云
 */
void Obstacle_cloud_get::PointCloudObstacleRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_map_msg, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_msg, 
                              double Distance_Threshold) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed(new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::toROSMsg(*cloud_map_msg, prior_map_msg);
    // prior_map_msg.header.frame_id = "map";
    // prior_map_msg.header.stamp = ros::Time::now();
    // pub_prior_map_.publish(prior_map_msg);


    pcl::transformPointCloud(*cloud_msg, *cloud_processed, icp_transform_); // 应用变换
    // ROS_WARN("BEGIN GET OBSTACLE!!!");

    // pcl::toROSMsg(*cloud_icp_z, incoming_cloud_msg);
    // incoming_cloud_msg.header.frame_id = "map";
    // incoming_cloud_msg.header.stamp = ros::Time::now();
    // incoming_pub_.publish(incoming_cloud_msg); //发布实时雷达点云

	start_time=clock();

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
            }
        }
    }
    
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_radius(new pcl::PointCloud<pcl::PointXYZ>);
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
    
    pcl::toROSMsg(*cloud_removed, cloud_removed_msg);
    cloud_removed_msg.header.frame_id = "odom";
    cloud_removed_msg.header.stamp = ros::Time::now();
    removal_pointcloud_publisher_.publish(cloud_removed_msg);

    pcl::toROSMsg(*cloud_filtered_radius, obstacle_cloud_msg);
    obstacle_cloud_msg.header.frame_id = "odom";
    obstacle_cloud_msg.header.stamp = ros::Time::now();
    Obstacle_cloud_pub_.publish(obstacle_cloud_msg);

    transformed_cloud_->clear();
    // pcl::transformPointCloud(*world_Obstacle_cloud_, *transformed_cloud_, transform); // 应用变换
    *transformed_cloud_ = *world_Obstacle_cloud_+*cloud_filtered_radius;

    pcl::toROSMsg(*transformed_cloud_, world_obstacle_msg);
    world_obstacle_msg.header.frame_id = "odom";
    world_obstacle_msg.header.stamp = ros::Time::now();
    world_obstacle_pub_.publish(world_obstacle_msg);
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