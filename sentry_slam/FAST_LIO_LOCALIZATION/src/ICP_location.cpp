//icp
#include <ctime>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ndt.h>

using namespace std;
clock_t start_time,end_time;
double t_icp_man=0;
Eigen::Matrix4f T_ts_man = Eigen::Matrix4f::Identity();
// 可视化两个点云的配准
void twoPointCloudRegistrationViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud3)
{
	// 输出结果到可视化窗口
	// 创建可视化窗口
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D PointCloud Viewer"));

	// 设置视口1，显示原始点云
	int v1;
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  // 左侧窗口
	viewer->setBackgroundColor(0.0, 0.0, 0.0, v1);  // 黑色背景
	viewer->addText("cloud1 PointCloud", 10, 10, "vp1_text", v1);  // 标题
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_color(cloud1, 0, 255, 0);  // 绿色
	viewer->addPointCloud(cloud1, cloud1_color, "original_cloud1", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud2_color(cloud2, 255, 0, 0);  // 红色
	viewer->addPointCloud(cloud2, cloud2_color, "original_cloud2", v1);

	// 设置视口2，显示
	int v2;
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);  // 右侧窗口
	viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);   // 黑色背景
	viewer->addText("cloud2 PointCloud", 10, 10, "vp2_text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud2_color2(cloud2, 0, 0, 255);  // 蓝色
	viewer->addPointCloud(cloud2, cloud2_color2, "original_cloud21", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud3_color(cloud3, 255, 0, 0);  // 红色
	viewer->addPointCloud(cloud3, cloud3_color, "original_cloud3", v2);

	// 设置点的大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud1", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud2", v2);

	// 添加坐标系
   /* viewer->addCoordinateSystem(0.1);
	viewer->initCameraParameters();*/

	// 可视化循环
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
}

// Point cloud registration by manual ICP-SVD
void ICP_MANUAL(pcl::PointCloud<pcl::PointXYZ>::Ptr &cl_s,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &cl_t,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &cl_final,
                Eigen::Matrix4f &T_ts,
                double &time_cost)
{
    clock_t startT, endT;
    startT = clock();

    // target cloud is fixed, so kd-tree create for target
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
    kd_tree.setInputCloud(cl_t);

    int nICP_Step = 20; //you can change this param for different situations
    float max_correspond_distance_ = 0.001; //you can change this param for different situations
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity(); //init T_st
    for (int i = 0; i < nICP_Step; ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_S_Trans(new pcl::PointCloud<pcl::PointXYZ>);
        // let us transform [cl_s] to [cloud_S_Trans] by [T_st](transform)
        // T_st changed for each interation, make [cloud_S_Trans] more and more close to [cl_s]
        pcl::transformPointCloud(*cl_s, *cloud_S_Trans, transform);
        std::vector<int> match_indexs;
        for (auto i = 0; i < cloud_S_Trans->size(); ++i)
        {
            std::vector<float> k_distances(2);
            std::vector<int> k_indexs(2);
            kd_tree.nearestKSearch(cloud_S_Trans->at(i), 1, k_indexs, k_distances);
            if (k_distances[0] < max_correspond_distance_)
            {
                match_indexs.emplace_back(k_indexs[0]);
            }
        }

        // matched clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_match_from_S_trans(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_S_Trans, match_indexs, *cloud_match_from_S_trans);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_match_from_T(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cl_t, match_indexs, *cloud_match_from_T);

        // 3d center of two clouds
        Eigen::Vector4f mu_S_temp, mu_T_temp;
        pcl::compute3DCentroid(*cloud_match_from_S_trans, mu_S_temp);
        pcl::compute3DCentroid(*cloud_match_from_T, mu_T_temp);
        Eigen::Vector3f mu_S(mu_S_temp[0], mu_S_temp[1], mu_S_temp[2]);
        Eigen::Vector3f mu_T(mu_T_temp[0], mu_T_temp[1], mu_T_temp[2]);

        // H += (S_i) * (T_i^T)
        Eigen::Matrix3f H_icp = Eigen::Matrix3f::Identity();
        for (auto i = 0; i < match_indexs.size(); ++i)
        {
            Eigen::Vector3f s_ci(cloud_match_from_S_trans->at(i).x - mu_S[0],
                                 cloud_match_from_S_trans->at(i).y - mu_S[1],
                                 cloud_match_from_S_trans->at(i).z - mu_S[2]);
            Eigen::Vector3f t_ci(cloud_match_from_T->at(i).x - mu_T[0],
                                 cloud_match_from_T->at(i).y - mu_T[1],
                                 cloud_match_from_T->at(i).z - mu_T[2]);
            Eigen::Matrix3f H_temp = s_ci * t_ci.transpose();
            H_icp += H_temp;
        }

        // H = SVD
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(H_icp, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix3f R_ts = svd.matrixU() * (svd.matrixV().transpose());
        Eigen::Vector3f t_ts = mu_T - R_ts * mu_S;// T = R_ts * S + t_ts
        Eigen::Quaternionf q_ts(R_ts);
        transform.block<3, 3>(0,
                              0) *= q_ts.normalized().toRotationMatrix().inverse();//we use left form, so we need .inverse()
        transform.block<3, 1>(0, 3) += t_ts;
        // We got a new [transform] here, that is, we are more closer to the [source] than last [transform].
        // This is why we call it Iterative method.
    }
    T_ts = transform;//transform = T_ts
    // Target = T_t * Source
    pcl::transformPointCloud(*cl_s, *cl_final, T_ts);

    endT = clock();
    time_cost = (double) (endT - startT) / CLOCKS_PER_SEC;
}


 void loadPCDCloud(const std::string& file_path,pcl::PointCloud<pcl::PointXYZ>::Ptr cloudname)
{
    pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloudname);
    if(cloudname==nullptr)
    ROS_ERROR("PCD LOAD FAILED!");
}

int main()
{
	// 读取点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSre(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTat(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/rm/catkin_livox_ros_driver2/src/IST_2025_sentry/sentry_slam/FAST_LIO_LOCALIZATION/PCD/demo456.pcd", *cloudSre) == -1 ||
		pcl::io::loadPCDFile<pcl::PointXYZ>("/home/rm/catkin_livox_ros_driver2/src/IST_2025_sentry/sentry_slam/FAST_LIO_LOCALIZATION/PCD/demo123.pcd", *cloudTat) == -1)
	{
		PCL_ERROR("Couldn't read the PCD files!\n");
		return -1;
	}

	Eigen::Matrix4f T_ts_my = Eigen::Matrix4f::Identity();
	// ICP配准
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	icp.setInputSource(cloudSre);			// 源点云
	icp.setInputTarget(cloudTat);			// 目标点云
	icp.setTransformationEpsilon(1e-5);	// 为终止条件设置最小转换差异
	icp.setMaxCorrespondenceDistance(0.25);	// 设置对应点对之间的最大距离
	icp.setEuclideanFitnessEpsilon(1e-5);	// 设置收敛条件是均方误差和小于阈值， 停止迭代；
	icp.setMaximumIterations(30);			// 最大迭代次数
	icp.setUseReciprocalCorrespondences(false);	// 设置是否使用点对之间的交互对应关系

	gicp.setInputSource(cloudSre);
	gicp.setInputTarget(cloudTat);
	gicp.setMaximumIterations(30);			// 最大迭代次数

	ndt.setInputSource(cloudSre);
	ndt.setInputTarget(cloudTat);
	ndt.setEuclideanFitnessEpsilon(1e-10);	// 设置收敛条件是均方误差和小于阈值， 停止迭代；
	ndt.setMaximumIterations(100);			// 最大迭代次数
	ndt.setTransformationEpsilon(1e-5);	// 为终止条件设置最小转换差异

	//KDtree存储目标点云
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>());
	tree1->setInputCloud(cloudSre);
	gicp.setSearchMethodSource(tree1);
	icp.setSearchMethodSource(tree1);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(cloudTat);
	gicp.setSearchMethodTarget(tree);
	icp.setSearchMethodTarget(tree);

	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr gicp_result(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr my_icp_result(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_result(new pcl::PointCloud<pcl::PointXYZ>);

	ICP_MANUAL(cloudSre, cloudTat, my_icp_result, T_ts_man,t_icp_man);
	cout << "time cost of ICP-SVD = " << t_icp_man << "s" << endl;
	start_time=clock();
	gicp.align(*gicp_result);
    if(gicp.hasConverged())
	{
	end_time=clock();
    ROS_INFO("GICP cost:%f",(double)(end_time-start_time));
    ROS_INFO("ICP converged with score: %f", gicp.getFitnessScore());
	}

	start_time=clock();
	icp.align(*icp_result);
	if(icp.hasConverged())
	{
	end_time=clock();
	ROS_INFO("ICP cost:%f",(double)(end_time-start_time));
	}

	start_time=clock();
	ndt.align(*ndt_result);
	if(ndt.hasConverged())
	{
	end_time=clock();
	ROS_INFO("NDT cost:%f",(double)(end_time-start_time));
	}
	
	twoPointCloudRegistrationViewer(cloudSre, cloudTat, icp_result);

	return 0;
}
