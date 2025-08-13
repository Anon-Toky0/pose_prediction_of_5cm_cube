#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <pose_prediction/PoseDetectionRes.h>
#include <std_msgs/Bool.h>

class TFPublisherNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    std::string base_frame_;
    std::string target_frame_;
    bool publish_tf_on_success_only_;
    bool last_detection_success_;
    geometry_msgs::TransformStamped last_transform_;
    
    // 发布频率控制
    ros::Timer tf_timer_;
    double publish_rate_;

public:
    TFPublisherNode() : nh_("~"), last_detection_success_(false) {
        // 初始化参数
        nh_.param<std::string>("base_frame", base_frame_, "camera_color_optical_frame");
        nh_.param<std::string>("target_frame", target_frame_, "camera_result");
        nh_.param("publish_tf_on_success_only", publish_tf_on_success_only_, true);
        nh_.param("publish_rate", publish_rate_, 10.0);  // 10Hz
        
        // 初始化订阅
        pose_sub_ = nh_.subscribe("/pose_prediction/pose_result", 1, &TFPublisherNode::poseCallback, this);
        
        // 初始化定时器用于持续发布TF
        if (publish_rate_ > 0) {
            tf_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), 
                                       &TFPublisherNode::timerCallback, this);
        }
        
        // 初始化默认变换
        last_transform_.header.frame_id = base_frame_;
        last_transform_.child_frame_id = target_frame_;
        last_transform_.transform.translation.x = 0.0;
        last_transform_.transform.translation.y = 0.0;
        last_transform_.transform.translation.z = 0.0;
        last_transform_.transform.rotation.x = 0.0;
        last_transform_.transform.rotation.y = 0.0;
        last_transform_.transform.rotation.z = 0.0;
        last_transform_.transform.rotation.w = 1.0;
        
        ROS_INFO("TF node start");
    }
    
    void poseCallback(const pose_prediction::PoseDetectionRes::ConstPtr& msg) {
        // 更新检测状态
        last_detection_success_ = msg->success.data;
        
        // 创建TF变换消息
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header = msg->header;
        transform_stamped.header.frame_id = base_frame_;
        transform_stamped.child_frame_id = target_frame_;
        
        if (last_detection_success_) {
            // 使用检测到的位姿
            transform_stamped.transform.translation.x = msg->pose.position.x;
            transform_stamped.transform.translation.y = msg->pose.position.y;
            transform_stamped.transform.translation.z = msg->pose.position.z;
            transform_stamped.transform.rotation = msg->pose.orientation;
            
            ROS_INFO("received successful result:");
            ROS_INFO("  position: (%.3f, %.3f, %.3f)", 
                     msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            ROS_INFO("  pose: (%.3f, %.3f, %.3f, %.3f)", 
                     msg->pose.orientation.x, msg->pose.orientation.y, 
                     msg->pose.orientation.z, msg->pose.orientation.w);
            
            // 输出欧拉角用于调试
            tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, 
                             msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            ROS_INFO("  Eolur (RPY): (%.3f, %.3f, %.3f) rad", roll, pitch, yaw);
            ROS_INFO("  Eolur (RPY): (%.1f, %.1f, %.1f) du", 
                     roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
        } else {
            // 检测失败，使用单位变换
            transform_stamped.transform.translation.x = 0.0;
            transform_stamped.transform.translation.y = 0.0;
            transform_stamped.transform.translation.z = 0.0;
            transform_stamped.transform.rotation.x = 0.0;
            transform_stamped.transform.rotation.y = 0.0;
            transform_stamped.transform.rotation.z = 0.0;
            transform_stamped.transform.rotation.w = 1.0;
            
            ROS_WARN("detected failed");
        }
        
        // 更新最后的变换
        last_transform_ = transform_stamped;
        
        // 如果不使用定时器，直接发布
        if (publish_rate_ <= 0) {
            publishTransform(transform_stamped);
        }
    }
    
    void timerCallback(const ros::TimerEvent&) {
        // 根据设置决定是否发布TF
        if (publish_tf_on_success_only_ && !last_detection_success_) {
            return;  // 仅在成功时发布，但当前检测失败
        }
        
        // 更新时间戳
        last_transform_.header.stamp = ros::Time::now();
        publishTransform(last_transform_);
    }
    
    void publishTransform(const geometry_msgs::TransformStamped& transform) {
        try {
            tf_broadcaster_.sendTransform(transform);
            
            // 每5秒输出一次调试信息
            static ros::Time last_debug_time = ros::Time::now();
            if ((ros::Time::now() - last_debug_time).toSec() > 5.0) {
                last_debug_time = ros::Time::now();
            }
        } catch (const std::exception& e) {
            ROS_ERROR("release TF failed: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher_node");
    
    try {
        TFPublisherNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("TF publisher node failed: %s", e.what());
        return 1;
    }
    
    return 0;
} 