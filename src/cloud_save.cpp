#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "ros/ros.h"

#include <pcl/io/pcd_io.h>
#include <std_srvs/Trigger.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud(new pcl::PointCloud<pcl::PointXYZ>);
bool cloud_received = false;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    *latest_cloud = cloud;
    cloud_received = true;
}


bool save_cloud(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    static int save_count = 0;
    if (!cloud_received) {
        res.success = false;
        res.message = "No point cloud received yet.";
        return true;
    }
    std::string filename = "saved_cloud.pcd";
    if (pcl::io::savePCDFileBinary(filename, *latest_cloud) == 0) {
        res.success = true;
        res.message = "Saved to " + filename;
    } else {
        res.success = false;
        res.message = "Failed to save cloud.";
    }
    return true;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_saver");
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/pose_prediction/output_cloud", 1, cloud_callback);
    ros::ServiceServer service = nh.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>("/save_cloud", save_cloud);
    ROS_INFO("Cloud saver service ready. Call /save_cloud to save latest point cloud.");
    ros::spin();
    return 0;
}