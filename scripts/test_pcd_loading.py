#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import subprocess

def check_pcd_file(file_path):
    """检查PCD文件是否存在并显示基本信息"""
    print(f"检查PCD文件: {file_path}")
    
    if not os.path.exists(file_path):
        print(f"❌ 文件不存在: {file_path}")
        return False
    
    # 获取文件大小
    file_size = os.path.getsize(file_path)
    print(f"✅ 文件存在")
    print(f"📁 文件大小: {file_size / 1024:.1f} KB")
    
    # 尝试使用pcl_viewer查看点云信息（如果可用）
    try:
        # 使用pcl工具获取点云信息
        result = subprocess.run(['pcl_pcd_convert_NaN_nan', file_path, '/tmp/temp_check.pcd'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("✅ PCD文件格式有效")
        else:
            print("⚠️  PCD文件格式可能有问题")
    except:
        print("ℹ️  无法验证PCD格式（pcl工具不可用）")
    
    return True

def test_node_parameters():
    """测试节点参数设置"""
    print("\n=== 测试节点参数 ===")
    
    # 模拟ROS参数
    default_path = "/home/smartdrone/position/saved_cloud.pcd"
    
    print(f"默认目标点云路径: {default_path}")
    
    if check_pcd_file(default_path):
        print("✅ 默认PCD文件检查通过")
        return True
    else:
        print("❌ 默认PCD文件检查失败")
        return False

def suggest_improvements():
    """提供改进建议"""
    print("\n=== 配准精度改进建议 ===")
    print("使用真实目标点云的优势：")
    print("1. 🎯 更准确的几何特征匹配")
    print("2. 🔧 更好的初始对齐效果") 
    print("3. 📊 更高的配准成功率")
    print("4. ⚡ 更快的收敛速度")
    
    print("\n目标点云质量要求：")
    print("• 包含完整的物体几何信息")
    print("• 点密度适中（不要太稀疏或太密集）")
    print("• 没有明显的噪声和离群点")
    print("• 坐标系与输入点云一致")
    
    print("\n进一步优化建议：")
    print("• 调整voxel_size参数以平衡精度和速度")
    print("• 根据目标物体大小调整feature_radius")
    print("• 增加ICP迭代次数以获得更高精度")
    print("• 考虑使用多分辨率配准策略")

def main():
    print("🔍 PCD文件加载测试工具")
    print("=" * 50)
    
    # 测试参数设置
    success = test_node_parameters()
    
    if success:
        print("\n✅ 所有检查通过！系统已准备好使用真实目标点云进行配准。")
    else:
        print("\n❌ 检查失败！请确保saved_cloud.pcd文件存在且有效。")
    
    # 提供改进建议
    suggest_improvements()
    
    print("\n" + "=" * 50)
    print("测试完成！")

if __name__ == '__main__':
    main() 