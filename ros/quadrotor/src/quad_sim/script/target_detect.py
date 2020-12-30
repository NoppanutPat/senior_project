#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from std_msgs.msg import Float64MultiArray

class TargetDetect(object):
    
    def __init__(self):
        
        super(TargetDetect,self).__init__()
        
        rospy.init_node("target_detect",anonymous=True)
        rospy.Subscriber("/camera1/image_raw",Image,self.image_callback)
        self.pos_pub = rospy.Publisher("/quadrotor/point_pos",Float64MultiArray,queue_size=10)
        self.bridge = CvBridge()

    def image_callback(self,data): 

        self.cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")

    def detect(self):

        try:

            image = self.cv_image

        except AttributeError:

            return 1

        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        image = cv2.medianBlur(image, 3)
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2Lab)
        image = cv2.inRange(image, np.array([20, 150, 150]), np.array([190, 255, 255]))

        image = cv2.GaussianBlur(image, (5, 5), 2, 2)

        circles = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)

        # print(circles[0][0][:2])

        try:

            data = Float64MultiArray(data=circles[0][0][:2])

        except Exception:

            data = Float64MultiArray(data=[-1000,-1000])

        self.pos_pub.publish(data)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            cv2.circle(image, center=(circles[0, 0], circles[0, 1]), radius=circles[0, 2], color=(100, 100, 100), thickness=2)

        # cv2.imshow("image",image)
        # cv2.waitKey(1)


if __name__ == "__main__":
    detect = TargetDetect()
    print("Node is starting!")
    while not rospy.is_shutdown():
        detect.detect()