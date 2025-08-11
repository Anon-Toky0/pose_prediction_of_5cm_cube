#! /usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
import numpy
import cv_bridge
import ultralytics
import typing
import ultralytics.engine.results
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes


class YoloDetector:
    def __init__(self) -> None:
        self.bridge = cv_bridge.CvBridge()
        self.model = ultralytics.YOLO(str(rospy.get_param('/path_of_model')))
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback, queue_size=1)
        self.boxes_pub = rospy.Publisher('/pose_prediction/bounding_boxes', BoundingBoxes, queue_size=1)
        self.copped_image_pub = rospy.Publisher('/pose_prediction/cropped_image', Image, queue_size=1)
        
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr('CvBridgeError: {}'.format(e))
            return
        results: typing.List[ultralytics.engine.results.Results] = self.model(cv_image)
        res = BoundingBoxes()
        res.header = msg.header
        res.bounding_boxes = []
        for result in results:
            # 绘制检测框和类别名
            if result.boxes is not None and result.boxes.xyxy is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                # probs: numpy.ndarray = result.probs.cpu().numpy()
                for box, cls_id in zip(boxes, classes):
                    temp = BoundingBox()
                    x1, y1, x2, y2 = map(int, box)
                    class_name = self.model.names[int(cls_id)]
                    temp.Class = class_name
                    temp.xmin = min(x1, x2) + 5
                    temp.ymin = min(y1, y2) + 5
                    temp.xmax = max(x1, x2) - 5
                    temp.ymax = max(y1, y2) - 5
                    # temp.probability = float(prob)
                    # cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # cv2.putText(cv_image, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    res.bounding_boxes.append(temp)
                    if temp.xmin >= temp.xmax or temp.ymin >= temp.ymax:
                        rospy.logwarn('Invalid bounding box dimensions, skipping cropping.')
                        continue
                    self.copped_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image[temp.ymin:temp.ymax, temp.xmin:temp.xmax], encoding='bgr8'))
        
        self.boxes_pub.publish(res)
                    
        # cv2.imshow('YOLO Detection', cv_image)
        # cv2.waitKey(1)        
        
    def loop(self):
        rospy.spin()

def main():
    rospy.init_node('yolo_detector', anonymous=False)
    detector = YoloDetector()
    detector.loop()
    

if __name__ == '__main__':
    main()