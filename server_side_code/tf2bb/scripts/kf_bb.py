#!/usr/bin/env python
import numpy as np
from numpy import ma
from pykalman import KalmanFilter
import rospy
from tld_msgs.msg import BoundingBox

msgrc =  0
meas_x = 0
meas_y = 0
meas_z = 0
meas_w = 0
meas_h = 0
meas_conf = 0

def boundingbox_cb(data):
    global msgrc, meas_conf, meas_h, meas_w, meas_x, meas_y, meas_z
    
    meas_conf = data.confidence
    meas_h = data.height
    meas_w = data.width
    meas_x = data.x
    meas_y = data.y
    meas_z = data.z
    msgrc = 1

    
def republisher():
    global msgrc, meas_conf, meas_h, meas_w, meas_x, meas_y, meas_z
    
    rospy.init_node('kf_bb_node')
    rospy.Subscriber("/bbox", BoundingBox, boundingbox_cb)
    
    pub = rospy.Publisher ("/bbox_final", BoundingBox, queue_size=2)
    rate = rospy.Rate(10)

    trans_mat = [[1, 0, 0, 0.1, 0, 0, 0.005, 0, 0],
                [0, 1, 0, 0, 0.1, 0, 0, 0.005, 0],
                [0, 0, 1, 0, 0, 0.1, 0, 0, 0.005],
                [0, 0, 0, 1, 0, 0, 0.1, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0.1, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0.1],
                [0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1],
                ]

    obs_mat =   [[1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0],
                ]


    kf = KalmanFilter(transition_matrices = trans_mat, observation_matrices = obs_mat)
    filtered_state_means = [np.zeros((9))] * 2
    filtered_state_covariances = [np.eye(9)] * 2
    measurement_ar = []
    seq_count = 0

    while not rospy.is_shutdown():
        seq_count = seq_count+1
        print("msgrc: "+str(msgrc))
        if(msgrc==1):
            measurement = ma.asarray([meas_x, meas_y, meas_z])
            print(measurement.shape)
            msgrc = 0
        else:
            # measurement = ma.asarray([ma.masked, ma.masked, ma.masked])
            measurement = ma.masked
        measurement_ar.append(measurement)
        # print(measurement_ar)
        # kf = kf.em(ma.asarray(measurement_ar), n_iter=5)
            
        filtered_state_means[1], filtered_state_covariances[1]  = kf.filter_update(filtered_state_means[0],filtered_state_covariances[0], measurement)
        # print(out)
        filtered_state_means[0] = filtered_state_means[1]
        filtered_state_covariances[0] = filtered_state_covariances[1]
        
        print(filtered_state_means[1][:3])
        msg = BoundingBox()
        msg.header.seq = seq_count
        msg.header.stamp = rospy.get_rostime()
        msg.x = filtered_state_means[1][0]
        msg.y = filtered_state_means[1][1]
        msg.z = filtered_state_means[1][2]
        msg.width = meas_w
        msg.height = meas_h
        msg.confidence = meas_conf
        pub.publish(msg)
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    republisher()

