#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

import numpy as np
import cv2

import torch
import torchvision.transforms as T

import sys
sys.path = ["/home/pat/catkin_workspace/install/lib/python3/dist-packages"] + sys.path
from cv_bridge import CvBridge

import imutils



class target_detect(object):
    def __init__(self):

        rospy.init_node("target_detect",anonymous=True)
        self.br = CvBridge()
        rospy.Subscriber("/camera1/image_raw",Image,self.image_callback)
        self.pub = rospy.Publisher("/target_coor",Float64MultiArray,queue_size=10)
        self.ratio = 0.8
    
    def image_callback(self,msg):
        tmp_image = self.br.imgmsg_to_cv2(msg)
        scale_percent = 60 # percent of original size
        width = int(tmp_image.shape[1] * scale_percent / 100)
        height = int(tmp_image.shape[0] * scale_percent / 100)
        dim = (width, height)
        # resize image
        self.image = cv2.resize(tmp_image, dim, interpolation = cv2.INTER_AREA)
        cv2.imshow("image",self.image)
        cv2.waitKey(1)

    def detect_image(self,image):

        ratio = self.ratio

        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        for c in cnts:

            M = cv2.moments(c)
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)

            if len(approx) == 4:

                (x, y, w, h) = cv2.boundingRect(approx)
                coor = Float64MultiArray(data=[x,y])
                pub.publish(coor)
    
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)

if __name__ == "__main__":

    target = target_detect()
    rospy.spin()