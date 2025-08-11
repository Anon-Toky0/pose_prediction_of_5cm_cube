#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/core/core.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>
#include <vector>
#include <pose_prediction/PoseDetectionRes.h>

std::vector<Eigen::Vector4f> plane_equations;
std::vector<Eigen::Vector3f> plane_normals;

image_geometry::PinholeCameraModel cam_model;
void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& cam_info_msg) {
    cam_model.fromCameraInfo(*cam_info_msg);
}

sensor_msgs::Image image;
cv::Mat mask;
void image_callback(const sensor_msgs::Image::ConstPtr& img_msg) {
    image = *img_msg;
    cv_bridge::CvImagePtr cv_img;
    try {
        cv_img = cv_bridge::toCvCopy(*img_msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mask = cv_img->image;
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    plane_equations.clear();
    plane_normals.clear();
    static ros::Publisher cloud_pub = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("/pose_prediction/output_cloud", 1);

    if (mask.empty()) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    int image_width = image.width;
    int image_height = image.height;

    for (const pcl::PointXYZ& point : cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        if (std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z) > 0.9) {
            continue;
        }

        cv::Point2d uv = cam_model.project3dToPixel(cv::Point3d(point.x, point.y, point.z));
        int u = static_cast<int>(uv.x);
        int v = static_cast<int>(uv.y);

        if (mask.at<uchar>(v, u) == 0 || u < 0 || u >= image_width || v < 0 || v >= image_height) {
            continue;
        }

        filtered_cloud->points.push_back(point);
    }

    filtered_cloud->header = cloud->header;
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;

    static ros::Publisher plane_pub0 = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("/segmented_plane0", 1);
    static ros::Publisher plane_pub1 = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("/segmented_plane1", 1);
    static ros::Publisher plane_pub2 = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("/segmented_plane2", 1);
    static ros::Publisher center_pub = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("/center_point", 1);

    if (filtered_cloud->empty()) {
        ROS_WARN("Received empty point cloud");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 剩余点云（用于循环分割，初始为原始点云）
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>(*filtered_cloud));

    // 分割参数设置
    int max_planes = 3;          // 最大分割平面数量
    int min_plane_points = 5000;  // 平面最小点数量（过滤噪声）
    int plane_count = 0;         // 当前分割平面计数

    pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    center_cloud->header = cloud->header;
    center_cloud->width = 0;
    center_cloud->height = 1;
    center_cloud->is_dense = false;

    // 循环分割多个平面
    while (plane_count < max_planes && remaining_cloud->size() > min_plane_points) {
        // 1. 平面分割
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);   // 平面模型
        seg.setMethodType(pcl::SAC_RANSAC);      // RANSAC算法
        seg.setDistanceThreshold(0.001);          // 距离阈值（平面容忍度）
        seg.setInputCloud(remaining_cloud);      // 输入当前剩余点云
        seg.segment(*inliers, *coefficients);    // 执行分割

        // 检查是否分割到有效平面
        if (inliers->indices.size() < min_plane_points) {
            ROS_WARN("No more valid planes (insufficient points)");
            break;
        }

        Eigen::Vector4f plane_eq(
            coefficients->values[0],
            coefficients->values[1],
            coefficients->values[2],
            coefficients->values[3]
        );
        plane_equations.push_back(plane_eq);

        // 2. 提取分割出的平面点云
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(remaining_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);  // 提取inliers（平面点）
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*plane_cloud);

        long double plane_normal[3] = { 0, 0, 0 };

        for (const auto& point : plane_cloud->points) {
            plane_normal[0] += point.x;
            plane_normal[1] += point.y;
            plane_normal[2] += point.z;
        }

        plane_normals.push_back(Eigen::Vector3f(
            plane_normal[0] / plane_cloud->points.size(),
            plane_normal[1] / plane_cloud->points.size(),
            plane_normal[2] / plane_cloud->points.size()
        ));

        // 将中心点放入点云
        pcl::PointXYZ center_point;
        center_point.x = plane_normal[0] / plane_cloud->points.size();
        center_point.y = plane_normal[1] / plane_cloud->points.size();
        center_point.z = plane_normal[2] / plane_cloud->points.size();
        center_cloud->points.push_back(center_point);
        center_cloud->width += 1;

        *final_cloud += *plane_cloud;

        // 4. 从剩余点云中移除当前平面点（用于下一次分割）
        extract.setNegative(true);  // 保留非inliers（剩余点）
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*temp_cloud);
        remaining_cloud.swap(temp_cloud);  // 更新剩余点云

        plane_count++;
    }

    sensor_msgs::PointCloud2 center_msg;
    pcl::toROSMsg(*center_cloud, center_msg);
    center_msg.header = cloud_msg->header; // 保留原始点云的header
    center_pub.publish(center_msg);

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*final_cloud, output_msg);
    output_msg.header = cloud_msg->header; // 保留原始点云的header
    cloud_pub.publish(output_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_segment_node");
    ros::NodeHandle nh;
    ros::Subscriber cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, camera_info_callback);

    ros::Subscriber img_sub = nh.subscribe("/pose_prediction/segments_masks", 1, image_callback);
    ros::Subscriber cloud_sub = nh.subscribe("/camera/depth/color/points", 1, cloud_callback);
    ros::Publisher pose_pub = nh.advertise<pose_prediction::PoseDetectionRes>("/pose_prediction/pose_result", 1);
    while (ros::ok()) {
        ros::spinOnce();
        pose_prediction::PoseDetectionRes result;
        result.header.frame_id = "camera_color_optical_frame";
        result.header.stamp = ros::Time::now();
        if (plane_equations.size() == 3) {
            long double center[3] = { 0, 0, 0 };

            for (int i = 0; i < 3; i++) {
                if (plane_equations[i][2] > 0) {
                    center[0] += (plane_equations[i][0] * 0.025 + plane_normals[i][0]) / 3.0;
                    center[1] += (plane_equations[i][1] * 0.025 + plane_normals[i][1]) / 3.0;
                    center[2] += (plane_equations[i][2] * 0.025 + plane_normals[i][2]) / 3.0;
                } else {
                    center[0] += (-plane_equations[i][0] * 0.025 + plane_normals[i][0]) / 3.0;
                    center[1] += (-plane_equations[i][1] * 0.025 + plane_normals[i][1]) / 3.0;
                    center[2] += (-plane_equations[i][2] * 0.025 + plane_normals[i][2]) / 3.0;
                }
            }
            result.pose.position.x = center[0];
            result.pose.position.y = center[1];
            result.pose.position.z = center[2];
            result.success.data = true;

            Eigen::Vector3d normal_vector1(plane_equations[0][0], plane_equations[0][1], plane_equations[0][2]);
            normal_vector1.normalize();
            Eigen::Vector3d normal_vector2(plane_equations[1][0], plane_equations[1][1], plane_equations[1][2]);
            normal_vector2.normalize();
            Eigen::Vector3d normal_vector3(plane_equations[2][0], plane_equations[2][1], plane_equations[2][2]);
            normal_vector3.normalize();
            Eigen::Quaterniond quaternion;
            if (normal_vector3.dot(normal_vector1.cross(normal_vector2)) > 0) {
                normal_vector2 = (normal_vector2 - normal_vector1.dot(normal_vector2) * normal_vector1).normalized();
                normal_vector3 = normal_vector1.cross(normal_vector2).normalized();
                Eigen::Matrix3d rotation_matrix;
                rotation_matrix.col(0) = normal_vector1;
                rotation_matrix.col(1) = normal_vector2;
                rotation_matrix.col(2) = normal_vector3;
                quaternion = Eigen::Quaterniond(rotation_matrix);
                result.pose.orientation.x = quaternion.x();
                result.pose.orientation.y = quaternion.y();
                result.pose.orientation.z = quaternion.z();
                result.pose.orientation.w = quaternion.w();
            } else {
                normal_vector2 = (normal_vector2 - normal_vector1.dot(normal_vector2) * normal_vector1).normalized();
                normal_vector3 = normal_vector2.cross(normal_vector1).normalized();
                Eigen::Matrix3d rotation_matrix;
                rotation_matrix.col(0) = normal_vector1;
                rotation_matrix.col(1) = normal_vector2;
                rotation_matrix.col(2) = normal_vector3;
                quaternion = Eigen::Quaterniond(rotation_matrix);
                result.pose.orientation.x = quaternion.x();
                result.pose.orientation.y = quaternion.y();
                result.pose.orientation.z = quaternion.z();
                result.pose.orientation.w = quaternion.w();
            }

        } else if (plane_equations.size() == 2) {
            long double center[3] = { 0, 0, 0 };
            for (int i = 0; i < 2; i++) {
                if (plane_equations[i][2] > 0) {
                    center[0] += (plane_equations[i][0] * 0.025 + plane_normals[i][0]) / 2.0;
                    center[1] += (plane_equations[i][1] * 0.025 + plane_normals[i][1]) / 2.0;
                    center[2] += (plane_equations[i][2] * 0.025 + plane_normals[i][2]) / 2.0;
                } else {
                    center[0] += (-plane_equations[i][0] * 0.025 + plane_normals[i][0]) / 2.0;
                    center[1] += (-plane_equations[i][1] * 0.025 + plane_normals[i][1]) / 2.0;
                    center[2] += (-plane_equations[i][2] * 0.025 + plane_normals[i][2]) / 2.0;
                }
            }
            result.pose.position.x = center[0];
            result.pose.position.y = center[1];
            result.pose.position.z = center[2];
            result.success.data = true;

            Eigen::Vector3d normal_vector1(plane_equations[0][0], plane_equations[0][1], plane_equations[0][2]);
            normal_vector1.normalize();
            Eigen::Vector3d normal_vector2(plane_equations[1][0], plane_equations[1][1], plane_equations[1][2]);
            normal_vector2.normalize();
            Eigen::Quaterniond quaternion;
            normal_vector2 = (normal_vector2 - normal_vector1.dot(normal_vector2) * normal_vector1).normalized();
            Eigen::Vector3d normal_vector3 = normal_vector1.cross(normal_vector2).normalized();
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix.col(0) = normal_vector1;
            rotation_matrix.col(1) = normal_vector2;
            rotation_matrix.col(2) = normal_vector3;
            quaternion = Eigen::Quaterniond(rotation_matrix);
            result.pose.orientation.x = quaternion.x();
            result.pose.orientation.y = quaternion.y();
            result.pose.orientation.z = quaternion.z();
            result.pose.orientation.w = quaternion.w();

        } else {
            result.success.data = false;
        }

        pose_pub.publish(result);
    }
    return 0;
}