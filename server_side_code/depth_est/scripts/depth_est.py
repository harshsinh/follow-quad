#!/usr/bin/env python

import sys
import rospy
import roslib
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image

class image_feature:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_features",Image, queue_size=1)
        self.z_pub = rospy.Publisher("depth_meas",Pose2D, queue_size=3)
        self.bridge = CvBridge()
        self.prev_img = None
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.orb = cv2.ORB_create()
        self.stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        # subscribed Topic
        self.image_sub = rospy.Subscriber("/image",Image,self.callback)
        self.zx = 0
        self.zy = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        if self.prev_img is None:
            self.prev_img = cv_image
            return
        else:
            
            '''ORB feat'''
            kp1, des1 = self.orb.detectAndCompute(cv_image,None)
            kp2, des2 = self.orb.detectAndCompute(self.prev_img,None)
            matches = self.bf.match(des1,des2)
            matches = sorted(matches, key = lambda x:x.distance)
            kp1_mat = []
            kp2_mat = []
            for match in matches:
                pt1 = kp1[match.queryIdx].pt
                pt2 = kp2[match.trainIdx].pt
                if(pt1[0]>60 and pt1[0]<260 and pt1[1]>100 and pt1[1]<400):
                    kp1_mat.append(pt1)
                if(pt2[0]>100 and pt2[0]<400 and pt2[1]>100 and pt2[1]<400):   
                    kp2_mat.append(pt2)
            
            img3 = cv2.drawMatches(cv_image,kp1,self.prev_img,kp2,matches, None, flags=2)
            # img3 = cv2.drawKeypoints(cv_image, kp1, None, color=(0,255,0), flags=0)
            
            '''Stereo'''
            # img1 = cv2.cvtColor(self.prev_img, cv2.COLOR_BGR2GRAY)
            # img2 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # img3 = self.stereo.compute(img1,img2)
            
            self.prev_img = cv_image
            try:
              self.image_pub.publish(self.bridge.cv2_to_imgmsg(img3, "bgr8"))
            except CvBridgeError as e:
                print(e)

            '''ORB message'''
            kp1_maty,kp1_matx = zip(*kp1_mat)
            kp2_maty,kp2_matx = zip(*kp2_mat)
            
            sigma1x = np.std(kp1_matx)
            sigma1y = np.std(kp1_maty)
            sigma2x = np.std(kp2_matx)
            sigma2y = np.std(kp2_maty)
            
            dzx = 100./sigma2x - 100./sigma1x
            dzy = 100./sigma2y - 100./sigma1y
            
            z_msg = Pose2D()
            self.zx += np.mean(dzx)
            self.zy += np.mean(dzy)
            z_msg.x = self.zx
            z_msg.y = self.zy
            z_msg.theta = 0
            self.z_pub.publish(z_msg)

        


def main(args):
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
