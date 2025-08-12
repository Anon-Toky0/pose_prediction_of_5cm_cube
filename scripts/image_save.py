#! /usr/bin/python3
# -*- coding: utf-8 -*-

import os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSaver:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.count = 0
        self.image_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        
    def image_callback(self, msg: Image):
        try:
            # Convert the ROS Image message to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
            # Define the save path
            save_path = os.path.join(os.getcwd(), 'images/Image_{:04}.png'.format(self.count))
            self.count += 1
        
            # Save the image
            cv2.imwrite(save_path, cv_image)
            rospy.loginfo(f"Image saved at {save_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save image: {e}")
            
    def loop(self):
        rospy.spin()

def main():
    rospy.init_node('image_saver', anonymous=False)
    image_saver = ImageSaver()
    image_saver.loop()    
    
if __name__ == '__main__':
    main()