## [follow-quad](https://github.com/harshsinh/follow-quad)
Vision based Human robust tracking and following on Quadrotors.

### Directory Structure:
```
├── driver_common
│   └── ..............................::
├── README.md
├── remote_execute.md ..............................:: Instructions for remote server execution.
├── local_execute.md ..............................:: Instructions for local execution.
├── set_params.sh ..............................:: Shell scripts to set different params easily.
├── hector_quadrotor_tutorial
│   └── ..........:: Files related to hector quadrotor. [TODO: Purge and keep only the required ones]
├── gazebo_ros_pkgs
│   └── ..........:: Files needed for gazebosim. [TODO: Purge]
├── tld_msgs
│   └── ..........:: Custom ROS message files for publishing/subscribing BoundingBoxes.
├── results
│   └── ..........:: Some images/videos from the runs.
├── server_side_code
│   └── ..............................:: ROS based to be executed on remote machine.
└── follow_quad
    └── ..........:: Prime ROS package for quadrotor control.
```

### ROS Graph
The following `ros_graph` out put shows the major nodes and topic working on the simultaion side, i.e on the quadrotor side.
![ros-graph](results/fquad_rosgraph1.png?raw=true "ROS Graph")

### Running the code
Take a look at the files [`remote_execute.md`](https://github.com/harshsinh/follow-quad/blob/master/remote_execute.md) and [`local_execute.md`](https://github.com/harshsinh/follow-quad/blob/master/local_execute.md) for detailed instructions on how to recreate the runs.

### Requirements
On the remote machine the following packages are required to be installed:  
* `ros`
* `tensorflow_object_detector`
* `vision_msgs`
* `pykalman`

The requirements of these are not listed but would obiviously need to be installed.

On the local machine:  
* `gazebo`
* `ros`