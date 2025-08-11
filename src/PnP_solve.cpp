#include <ros/ros.h>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>

int low_threshold_;
const int ratio_ = 3;
const int kernel_size_ = 3;

image_geometry::PinholeCameraModel cam_model;
void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& cam_info_msg) {
    cam_model.fromCameraInfo(*cam_info_msg);
}

void image_callback(const sensor_msgs::Image::ConstPtr& img_msg) {

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(*img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;

    // 依据bounding box进行裁剪

    cv::Mat src_gray, detected_edges;
    // 转换为灰度图
    cv::cvtColor(image, src_gray, cv::COLOR_BGR2GRAY);

    // 灰度图0-255
    cv::normalize(src_gray, src_gray, 0, 255, cv::NORM_MINMAX);

    // Canny边缘检测
    cv::Canny(detected_edges, detected_edges, low_threshold_, low_threshold_ * ratio_, kernel_size_);
    // 显示裁剪后的图像
    cv::imshow("Cropped Image", detected_edges);
    cv::waitKey(1); // 等待1毫秒以更新窗口
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_prediction_node");
    ros::NodeHandle nh;
    nh.param("/low_threshold", low_threshold_, 50);

    ros::Subscriber cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, camera_info_callback);
    ros::Subscriber cropped_image_sub = nh.subscribe("/pose_prediction/cropped_image", 1, image_callback);
    cv::namedWindow("Cropped Image", cv::WINDOW_AUTOSIZE);
    ros::spin();
    return 0;
}