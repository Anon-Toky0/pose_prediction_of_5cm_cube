#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
bool has_new_box = false;

image_geometry::PinholeCameraModel cam_model;
void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& cam_info_msg) {
    cam_model.fromCameraInfo(*cam_info_msg);
    
    // 验证相机标定参数
    if (cam_info_msg->K[0] <= 0 || cam_info_msg->K[4] <= 0) {
        ROS_WARN("Invalid camera intrinsic parameters detected!");
    }
    
    // 输出相机参数用于调试
    ROS_INFO_ONCE("Camera info: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f, size=%dx%d", 
                  cam_info_msg->K[0], cam_info_msg->K[4], 
                  cam_info_msg->K[2], cam_info_msg->K[5],
                  cam_info_msg->width, cam_info_msg->height);
}

darknet_ros_msgs::BoundingBoxes bounding_boxes;
void box_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes_msg) {
    bounding_boxes = *boxes_msg;
    has_new_box = true;
}

void sync_callback(const sensor_msgs::Image::ConstPtr& img_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    static ros::Publisher cloud_pub = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("/pose_prediction/output_cloud", 1);

    if (!has_new_box) {
        ROS_WARN("No new bounding box received, skipping processing.");
        return;
    }

    if (bounding_boxes.bounding_boxes.empty()) {
        ROS_WARN("No bounding boxes available for processing");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    int image_width = img_msg->width;
    int image_height = img_msg->height;

    for (const pcl::PointXYZ& point : cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue; // Skip invalid points
        }
        // 仅保留距离原点1m以内的点
        double dist = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (dist > 1.0) {
            continue;
        }

        cv::Point2d uv = cam_model.project3dToPixel(cv::Point3d(point.x, point.y, point.z));
        int u = static_cast<int>(uv.x);
        int v = static_cast<int>(uv.y);

        // 添加投影调试信息（每1000个点输出一次）
        static int debug_count = 0;
        if (debug_count++ % 1000 == 0) {
            ROS_DEBUG("Point (%.3f,%.3f,%.3f) -> Pixel (%d,%d)", point.x, point.y, point.z, u, v);
        }

        // 检查投影点是否在图像范围内（添加边界缓冲）
        const int border_margin = 2;  // 像素边界缓冲
        if (u < border_margin || u >= (image_width - border_margin) || 
            v < border_margin || v >= (image_height - border_margin)) {
            continue;
        }

        // 检查该点是否在任何检测框内
        bool in_any_box = false;
        for (const auto& box : bounding_boxes.bounding_boxes) {
            if (u >= box.xmin && u <= box.xmax && v >= box.ymin && v <= box.ymax) {
                in_any_box = true;
                break;
            }
        }

        // 如果在检测框内，则保留该点
        if (in_any_box) {
            filtered_cloud->points.push_back(point);
        }
    }

    filtered_cloud->header = cloud->header;
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    output_msg.header = cloud_msg->header; // 保留原始点云的header
    cloud_pub.publish(output_msg);

    has_new_box = false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_solve_node");
    ros::NodeHandle nh;
    ros::Subscriber cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, camera_info_callback);
    ros::Subscriber box_sub = nh.subscribe("/pose_prediction/bounding_boxes", 1, box_callback);

    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/camera/depth/color/points", 1);


    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_sub, cloud_sub);
    sync.registerCallback(boost::bind(&sync_callback, _1, _2));

    ros::spin();
    return 0;
}