#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/ndt.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <pose_prediction/PoseDetectionRes.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

// 前向声明
pcl::PointCloud<pcl::PointXYZ>::Ptr createCubeCloud(float size);

class PosePredictionNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher pose_pub_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;  // 目标点云(立方体)
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features_;  // 目标点云特征

    // 配准参数
    std::string target_cloud_path_;
    double cube_size_;
    double voxel_size_;
    double normal_radius_;
    double feature_radius_;
    double sac_ia_max_distance_;
    int sac_ia_max_iterations_;
    double icp_max_distance_;
    int icp_max_iterations_;
    double icp_transformation_epsilon_;

public:
    PosePredictionNode() : nh_("~") {
        // 初始化参数
        nh_.param<std::string>("target_cloud_path", target_cloud_path_, "/home/smartdrone/position/saved_cloud.pcd");
        nh_.param("cube_size", cube_size_, 0.05);
        nh_.param("voxel_size", voxel_size_, 0.005);
        nh_.param("normal_radius", normal_radius_, 0.02);
        nh_.param("feature_radius", feature_radius_, 0.05);
        nh_.param("sac_ia_max_distance", sac_ia_max_distance_, 0.01);
        nh_.param("sac_ia_max_iterations", sac_ia_max_iterations_, 1000);
        nh_.param("icp_max_distance", icp_max_distance_, 0.005);
        nh_.param("icp_max_iterations", icp_max_iterations_, 50);
        nh_.param("icp_transformation_epsilon", icp_transformation_epsilon_, 1e-8);

        // 从PCD文件加载目标点云
        target_cloud_ = loadTargetCloudFromPCD(target_cloud_path_);

        // 预计算目标点云特征
        computeTargetFeatures();

        // 初始化订阅和发布
        cloud_sub_ = nh_.subscribe("/pose_prediction/output_cloud", 1, &PosePredictionNode::cloudCallback, this);
        pose_pub_ = nh_.advertise<pose_prediction::PoseDetectionRes>("/pose_prediction/pose_result", 1);

        ROS_INFO("位姿预测节点已启动，等待点云数据...");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr loadTargetCloudFromPCD(const std::string& file_path) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        ROS_INFO("正在加载目标点云文件: %s", file_path.c_str());

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
            ROS_ERROR("无法加载PCD文件: %s", file_path.c_str());
            ROS_ERROR("回退到使用程序生成的立方体点云");
            // 回退到立方体点云
            return createCubeCloud(cube_size_);
        }

        ROS_INFO("成功加载目标点云，点数: %lu", cloud->size());

        // 检查点云是否为空
        if (cloud->empty()) {
            ROS_WARN("加载的点云为空，回退到使用程序生成的立方体点云");
            return createCubeCloud(cube_size_);
        }

        // 移除无效点
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        if (cloud->empty()) {
            ROS_WARN("移除无效点后点云为空，回退到使用程序生成的立方体点云");
            return createCubeCloud(cube_size_);
        }

        ROS_INFO("清理后的目标点云点数: %lu", cloud->size());
        return cloud;
    }

    void computeTargetFeatures() {
        // 下采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_target(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(target_cloud_);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter.filter(*downsampled_target);

        if (downsampled_target->empty()) {
            ROS_ERROR("下采样后的目标点云为空!");
            return;
        }

        // 计算法向量
        pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setInputCloud(downsampled_target);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setRadiusSearch(normal_radius_);
        normal_estimator.compute(*target_normals);

        // 计算FPFH特征
        target_features_.reset(new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimator;
        fpfh_estimator.setInputCloud(downsampled_target);
        fpfh_estimator.setInputNormals(target_normals);
        fpfh_estimator.setSearchMethod(tree);
        fpfh_estimator.setRadiusSearch(feature_radius_);
        fpfh_estimator.compute(*target_features_);

        // 更新目标点云为下采样后的点云
        target_cloud_ = downsampled_target;

        ROS_INFO("目标点云特征计算完成，点数: %lu", target_cloud_->size());
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // 转换点云格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *source_cloud);

        if (source_cloud->empty()) {
            ROS_WARN("接收到的点云为空，跳过处理");
            publishFailureResult(msg->header);
            return;
        }

        // 预处理源点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_source = preprocessPointCloud(source_cloud);

        if (processed_source->empty()) {
            ROS_WARN("预处理后的源点云为空，跳过处理");
            publishFailureResult(msg->header);
            return;
        }

        // 执行配准
        Eigen::Matrix4f transformation;
        bool success = performRegistration(processed_source, transformation);

        // 发布结果
        publishResult(msg->header, success, transformation);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr processed(new pcl::PointCloud<pcl::PointXYZ>);

        // 移除无效点
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *processed, indices);

        if (processed->empty()) {
            return processed;
        }

        // 下采样
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(processed);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel_filter.filter(*processed);

        return processed;
    }

    bool performRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, Eigen::Matrix4f& transformation) {
        try {
            // 使用NDT算法进行点云配准
            pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
            ndt.setInputSource(source_cloud);
            ndt.setInputTarget(target_cloud_);
            ndt.setMaximumIterations(icp_max_iterations_); // 可复用icp参数
            ndt.setTransformationEpsilon(icp_transformation_epsilon_);
            ndt.setStepSize(0.1); // 步长可调
            ndt.setResolution(1.0); // 分辨率可调

            pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_result(new pcl::PointCloud<pcl::PointXYZ>);
            ndt.align(*ndt_result);

            if (!ndt.hasConverged()) {
                ROS_WARN("NDT配准失败");
                return false;
            }

            transformation = ndt.getFinalTransformation();
            double ndt_score = ndt.getFitnessScore();
            ROS_INFO("NDT配准完成，适应性得分: %f", ndt_score);
            return true;
        }
        catch (const std::exception& e) {
            ROS_ERROR("NDT配准过程中发生异常: %s", e.what());
            return false;
        }
    }

    void publishResult(const std_msgs::Header& header, bool success, const Eigen::Matrix4f& transformation) {
        pose_prediction::PoseDetectionRes result;
        result.header = header;
        result.success.data = success;

        if (success) {
            // 从变换矩阵提取位置和姿态
            Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);
            Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);

            // 转换为四元数
            Eigen::Quaternionf quaternion(rotation);

            result.pose.position.x = translation.x();
            result.pose.position.y = translation.y();
            result.pose.position.z = translation.z();
            result.pose.orientation.x = quaternion.x();
            result.pose.orientation.y = quaternion.y();
            result.pose.orientation.z = quaternion.z();
            result.pose.orientation.w = quaternion.w();

            ROS_INFO("配准成功! 位置: (%.3f, %.3f, %.3f), 姿态: (%.3f, %.3f, %.3f, %.3f)",
                translation.x(), translation.y(), translation.z(),
                quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
        }
        else {
            // 设置默认值
            result.pose.position.x = 0.0;
            result.pose.position.y = 0.0;
            result.pose.position.z = 0.0;
            result.pose.orientation.x = 0.0;
            result.pose.orientation.y = 0.0;
            result.pose.orientation.z = 0.0;
            result.pose.orientation.w = 1.0;

            ROS_WARN("配准失败!");
        }

        pose_pub_.publish(result);
    }

    void publishFailureResult(const std_msgs::Header& header) {
        pose_prediction::PoseDetectionRes result;
        result.header = header;
        result.success.data = false;
        result.pose.position.x = 0.0;
        result.pose.position.y = 0.0;
        result.pose.position.z = 0.0;
        result.pose.orientation.x = 0.0;
        result.pose.orientation.y = 0.0;
        result.pose.orientation.z = 0.0;
        result.pose.orientation.w = 1.0;

        pose_pub_.publish(result);
    }
};

pcl::PointCloud<pcl::PointXYZ>::Ptr createCubeCloud(float size) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cube(new pcl::PointCloud<pcl::PointXYZ>);
    // 生成立方体点云
    for (float y = -size / 2; y <= size / 2; y += size / 5)
        for (float z = -size / 2; z <= size / 2; z += size / 5)
            cube->push_back(pcl::PointXYZ(size / 2, y, z));
    for (float x = -size / 2; x <= size / 2; x += size / 5)
        for (float z = -size / 2; z <= size / 2; z += size / 5)
            cube->push_back(pcl::PointXYZ(x, size / 2, z));
    for (float x = -size / 2; x <= size / 2; x += size / 5)
        for (float y = -size / 2; y <= size / 2; y += size / 5)
            cube->push_back(pcl::PointXYZ(x, y, size / 2));
    return cube;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_prediction_node");

    try {
        PosePredictionNode node;
        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR("节点运行异常: %s", e.what());
        return 1;
    }

    return 0;
}