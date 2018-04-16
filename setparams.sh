#!/bin/bash
rosparam set kpx 1.50   # x in image = roll control
rosparam set kpy 0.00   # y in image = height control + pitch control(?)
rosparam set kpz -0.2  # z in image = distance control = pitch control
rosparam set kpyaw 0.00 # yaw in image

rosparam set kdx 0.0
rosparam set kdy 0.0
rosparam set kdz 0.001
rosparam set kdyaw 0.0

rosparam set kix 0.0
rosparam set kiy 0.0
rosparam set kiz 0.0
rosparam set kiyaw 0.0
