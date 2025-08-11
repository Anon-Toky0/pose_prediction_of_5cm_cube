#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from pose_prediction.msg import PoseDetectionRes
from std_msgs.msg import Bool, Header
from geometry_msgs.msg import Pose, Point, Quaternion
import math

class TFTester:
    def __init__(self):
        rospy.init_node('tf_tester', anonymous=True)
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 发布测试位姿结果
        self.pose_pub = rospy.Publisher('/pose_prediction/pose_result', PoseDetectionRes, queue_size=1)
        
        rospy.loginfo("TF测试节点已启动")
        
    def publish_test_pose(self, success=True, x=0.1, y=0.2, z=0.3, 
                         qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """发布测试位姿数据"""
        msg = PoseDetectionRes()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_aligned_depth_to_color_frame"
        
        msg.success = Bool()
        msg.success.data = success
        
        msg.pose = Pose()
        msg.pose.position = Point(x=x, y=y, z=z)
        msg.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        
        self.pose_pub.publish(msg)
        rospy.loginfo(f"发布测试位姿: 成功={success}, 位置=({x:.3f}, {y:.3f}, {z:.3f})")
        
    def check_tf_transform(self, source_frame="camera_aligned_depth_to_color_frame", 
                          target_frame="camera_result", timeout=5.0):
        """检查TF变换是否存在"""
        try:
            # 等待变换可用
            self.tf_buffer.can_transform(target_frame, source_frame, 
                                       rospy.Time(), rospy.Duration(timeout))
            
            # 获取变换
            transform = self.tf_buffer.lookup_transform(source_frame, target_frame, 
                                                      rospy.Time())
            
            # 输出变换信息
            t = transform.transform.translation
            r = transform.transform.rotation
            
            rospy.loginfo(f"TF变换 {source_frame} -> {target_frame}:")
            rospy.loginfo(f"  位置: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")
            rospy.loginfo(f"  姿态: ({r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f})")
            
            # 转换为欧拉角
            roll, pitch, yaw = self.quaternion_to_euler(r.x, r.y, r.z, r.w)
            rospy.loginfo(f"  欧拉角: Roll={math.degrees(roll):.1f}°, "
                         f"Pitch={math.degrees(pitch):.1f}°, "
                         f"Yaw={math.degrees(yaw):.1f}°")
            
            return True
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"无法获取TF变换: {e}")
            return False
    
    def quaternion_to_euler(self, x, y, z, w):
        """四元数转欧拉角"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def run_test(self):
        """运行测试序列"""
        rospy.loginfo("开始TF测试序列...")
        
        rate = rospy.Rate(1)  # 1Hz
        
        test_cases = [
            # (success, x, y, z, qx, qy, qz, qw)
            (True, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0),  # 基本平移
            (True, 0.0, 0.0, 0.5, 0.0, 0.0, 0.707, 0.707),  # 90度旋转
            (True, -0.1, -0.2, 0.1, 0.5, 0.5, 0.5, 0.5),  # 复杂变换
            (False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),  # 失败情况
        ]
        
        for i, (success, x, y, z, qx, qy, qz, qw) in enumerate(test_cases):
            if rospy.is_shutdown():
                break
                
            rospy.loginfo(f"\n=== 测试用例 {i+1}/{len(test_cases)} ===")
            
            # 发布测试位姿
            self.publish_test_pose(success, x, y, z, qx, qy, qz, qw)
            
            # 等待TF更新
            rospy.sleep(1.0)
            
            # 检查TF变换
            if success:
                self.check_tf_transform()
            else:
                rospy.loginfo("检测失败，应该不会有TF变换")
                
            rate.sleep()
        
        rospy.loginfo("\n测试完成！")

def main():
    try:
        tester = TFTester()
        
        # 等待其他节点启动
        rospy.loginfo("等待TF发布节点启动...")
        rospy.sleep(2.0)
        
        # 运行测试
        tester.run_test()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被中断")
    except Exception as e:
        rospy.logerr(f"测试出错: {e}")

if __name__ == '__main__':
    main() 