#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String 
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from tld_msgs.msg import BoundingBox


bb_pub = rospy.Publisher("/boundingbox", BoundingBox, queue_size=1)

def det_cb(objArray):
  
  if(len(objArray.detections)>0):
  	msg = BoundingBox()
  	obj = objArray.detections[0]
        i = 0
	while(obj.results[0].id!=1 and i<(len(objArray.detections)-1)):
		i = i+1
  		obj = objArray.detections[i]
		
  	rospy.loginfo(rospy.get_caller_id() + "I heard %s", obj.results[0].id)
        if(obj.results[0].id==1):	
  		msg.header = objArray.header
  		msg.x = obj.bbox.center.x
  		msg.y = obj.bbox.center.y
  		msg.height = obj.bbox.size_y
  		msg.width = obj.bbox.size_x
  		msg.confidence = obj.results[0].score
  		bb_pub.publish(msg) 
  


def main(args):
  
  rospy.init_node('tf2bb_node')
  det_sub = rospy.Subscriber("/objects", Detection2DArray, det_cb, queue_size=1)
  
  rospy.spin()

if __name__=='__main__':
  main(sys.argv)
