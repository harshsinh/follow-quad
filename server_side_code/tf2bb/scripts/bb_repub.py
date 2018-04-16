#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from tld_msgs.msg import BoundingBox
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

f = 0
T = 0
image = np.ones((320, 240))
disparity_time = 0
pub = rospy.Publisher ("/bbox", BoundingBox, queue_size=2)

def disparity_cb(data):

    global f, T, image, disparity_time
    bridge = CvBridge()
    disparity_time = data.header.stamp
    image = bridge.imgmsg_to_cv2(data.image, "32FC1")
    f = data.f
    T = data.T

def boundingbox_cb(data):

    global f, T, image, disparity_time, pub
    time = data.header.stamp

    submatrix = image[data.y-data.height/2:data.y+data.height/2, data.x-data.width/2:data.x+data.width/2]
    distance = np.mean(submatrix[submatrix!=0])
    num_nzeros = np.count_nonzero(submatrix)/float(image.shape[0]*image.shape[1])

    print ("num of nz : " + str(num_nzeros))

    cv2.imshow("submat", submatrix)

    print (data.x, data.y, data.width, data.height);

    rectim = cv2.rectangle(image, (data.x - data.width/2, data.y - data.height/2), (data.x + data.width/2, data.y + data.height/2), 255)
    cv2.imshow("bb overlay", rectim)
    cv2.waitKey(1)
    distance = f*T/(distance + 0.001)

    if(distance < 0):
        return

    # print ("distance, max(image), max(submat), mean(submat), sigma(submat)")
    # print(distance, np.max(image), np.max(submatrix), np.mean(submatrix), np.std(submatrix))

    msg = data
    msg.header.stamp = rospy.get_rostime()
    msg.z = distance
    pub.publish (msg)



def republisher():
    rospy.init_node('republisher_node')
    rospy.Subscriber("/boundingbox", BoundingBox, boundingbox_cb)
    rospy.Subscriber("/stereo/disparity", DisparityImage, disparity_cb)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        rate.sleep()
        rospy.spin()


if __name__ == '__main__':
    republisher()
