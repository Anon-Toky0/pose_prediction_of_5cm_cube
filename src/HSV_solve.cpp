#include "opencv4/opencv2/core/core.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

image_geometry::PinholeCameraModel cam_model;
void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& cam_info_msg) {
    cam_model.fromCameraInfo(*cam_info_msg);
}

void sync_callback(const sensor_msgs::Image::ConstPtr& img_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    static ros::Publisher cloud_pub = ros::NodeHandle().advertise<sensor_msgs::PointCloud2>("/pose_prediction/output_cloud", 1);
    static ros::Publisher debug_image_pub = ros::NodeHandle().advertise<sensor_msgs::Image>("/debug/color/image", 1);

    // 将ROS图像消息转换为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat hsv_img;
    cv::cvtColor(cv_ptr->image, hsv_img, cv::COLOR_BGR2HSV);

    // 红色HSV范围（低色调和高色调两段）
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv_img, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1); // 低色调红
    cv::inRange(hsv_img, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2); // 高色调红
    cv::bitwise_or(mask1, mask2, mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    // 创建输出图像，红色区域保留原像素，其余设为黑色
    cv::Mat result = cv::Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());
    cv_ptr->image.copyTo(result, mask);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;


    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxArea = 0;
    int mainContourIdx = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            mainContourIdx = i;
        }
    }


    // 创建纯黑背景的分离结果
    cv::Mat mainObject = cv::Mat::zeros(mask.size(), CV_8UC1);
    cv::Mat scatteredPoints = cv::Mat::zeros(mask.size(), CV_8UC1);

    // 处理找到的轮廓
    if (mainContourIdx >= 0) {
        // 绘制主体轮廓（内部填充）
        cv::drawContours(mainObject, contours, mainContourIdx, cv::Scalar(255), cv::FILLED);
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
        if (std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z) > 1.0) {
            continue;
        }

        cv::Point2d uv = cam_model.project3dToPixel(cv::Point3d(point.x, point.y, point.z));
        int u = static_cast<int>(uv.x);
        int v = static_cast<int>(uv.y);

        // 检查投影点是否在图像范围内, 如果不在或超出图像范围，则跳过
        if (mask.at<uchar>(v, u) == 0 || u < 0 || u >= image_width || v < 0 || v >= image_height) {
            continue;
        }

        filtered_cloud->points.push_back(point);
    }

    filtered_cloud->header = cloud->header;
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    output_msg.header = cloud_msg->header; // 保留原始点云的header
    cloud_pub.publish(output_msg);

    // 可选：显示结果（调试用）
    cv_bridge::CvImage debug_1 = cv_bridge::CvImage(img_msg->header, sensor_msgs::image_encodings::MONO8, mainObject);
    debug_image_pub.publish(*debug_1.toImageMsg());

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "HSV_solve");
    ros::NodeHandle nh;
    ros::Subscriber cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, camera_info_callback);
    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/camera/depth/color/points", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_sub, cloud_sub);
    sync.registerCallback(boost::bind(&sync_callback, _1, _2));
    ros::spin();

    return 0;
}
