#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

int low_threshold_;
static const std::string ORIGINAL_WINDOW = "Original Image";
static const std::string CANNY_WINDOW = "Canny Edges";
const int ratio_ = 3;
const int kernel_size_ = 3;

cv::Mat image;
void image_callback(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImagePtr temp;
    try
    {
        temp = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    image = temp->image;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "canny_edge");
    ros::NodeHandle nh;
    cv::namedWindow(ORIGINAL_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(CANNY_WINDOW, cv::WINDOW_AUTOSIZE);
    nh.param("/low_threshold", low_threshold_, 50);
    ros::Subscriber image_sub = nh.subscribe("/camera/color/image_raw", 1, image_callback);
    while (ros::ok())
    {
        ros::spinOnce();
        if (!image.empty())
        {
            cv::Mat src_gray, detected_edges;
            // 转换为灰度图
            cv::cvtColor(image, src_gray, cv::COLOR_BGR2GRAY);

            // 高斯模糊去除噪声
            cv::blur(src_gray, detected_edges, cv::Size(3, 3));

            // Canny边缘检测
            cv::Canny(detected_edges, detected_edges, low_threshold_, low_threshold_ * ratio_, kernel_size_);
            cv::imshow(CANNY_WINDOW, detected_edges);
            cv::imshow(ORIGINAL_WINDOW, image);
            cv::waitKey(5);
        }
    }
}
