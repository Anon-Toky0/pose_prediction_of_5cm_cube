#! /usr/bin/python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import ultralytics
import cv_bridge
import rospy
import typing
from sensor_msgs.msg import Image
import ultralytics.engine.results
from pose_prediction.srv import image2yoloseg, image2yolosegRequest, image2yolosegResponse

class YoloSegmenter:
    def __init__(self) -> None:
        self.bridge = cv_bridge.CvBridge()
        self.model = ultralytics.YOLO(str(rospy.get_param('/path_of_model')))
        self.image_inferrence_client = rospy.Service("/pose_prediction/image_inferrence", image2yoloseg, self.image_inferrence)
        
    def image_inferrence(self, request: image2yolosegRequest):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(request.input, desired_encoding='bgr8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr('CvBridgeError: {}'.format(e))
            return
        
        results: typing.List[ultralytics.engine.results.Results] = self.model(cv_image)
        
        # 创建合成掩码图像
        combined_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        
        for result in results:
            if result.masks is not None and result.masks.xy is not None:
                masks = result.masks.xy
                
                for mask in masks:
                    # 将每个掩码添加到合成掩码中
                    mask_img = cv2.fillPoly(
                        np.zeros(cv_image.shape[:2], dtype=np.uint8),
                        [mask.astype(int)],
                        255
                    )
                    # 使用OR操作合并掩码
                    combined_mask = cv2.bitwise_or(combined_mask, mask_img)
                    
                    # 在原始图像上绘制掩码（用于可视化）
                    cv2.fillPoly(cv_image, [mask.astype(int)], (0, 255, 0))
        
        # 发布合成掩码
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(combined_mask, encoding='mono8')
            return image2yolosegResponse(mask_msg)
        except cv_bridge.CvBridgeError as e:
            rospy.logerr('CvBridgeError when publishing mask: {}'.format(e))
        
        cv2.imshow('Segmented Image', cv_image)
        cv2.waitKey(1)
    
    
    

def main():
    rospy.init_node('yolo_segmenter', anonymous=False)
    seg = YoloSegmenter()
    rospy.spin()
    

if __name__ == '__main__':
    main()