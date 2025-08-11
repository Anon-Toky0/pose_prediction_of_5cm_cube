#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pose_prediction/PoseDetectionRes.h>
#include <std_msgs/Bool.h>

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    static ros::Publisher res_pub = ros::NodeHandle().advertise<pose_prediction::PoseDetectionRes>("/pose_prediction/pose_result", 1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    long double sumx = 0;
    long double sumy = 0;
    long double sumz = 0;
    int num = 0;
    for (const pcl::PointXYZ& point : cloud->points) {
        sumx += point.x;
        sumy += point.y;
        sumz += point.z;
        num++;
    }

    double avex = sumx / num;
    double avey = sumy / num;
    double avez = sumz / num;

    double length = std::sqrt(avex * avex + avey * avey + avez * avez);
    double length_plus = length + 0.01;

    avex = avex * length_plus / length;
    avey = avey * length_plus / length;
    avez = avez * length_plus / length;

    pose_prediction::PoseDetectionRes result;
    result.success.data = true;
    result.pose.position.x = avex;
    result.pose.position.y = avey;
    result.pose.position.z = avez;

    result.pose.orientation.w = 1.0;
    result.pose.orientation.x = 0.0;
    result.pose.orientation.y = 0.0;
    result.pose.orientation.z = 0.0;

    res_pub.publish(result);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "temp_prediction");
    ros::NodeHandle node;
    ros::Subscriber cloud_subscriber = node.subscribe("/pose_prediction/output_cloud", 1, cloud_callback);

    ros::spin();
    return 0;
}
