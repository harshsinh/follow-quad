## Instructions to set up the remote server for running the code.

* Exporting environment variables on the **remote** machine:  
    * Check machine-ip using `ifconfig`
    * `export ROS_MASTER_URI=http://<romote-machine-ip>:11311`
    * `export ROS_IP=<remote-machine-ip>`

**Note**: You need to export Environment Variable on **all terminals the involved**.

Execute the following (prefereably in this order)
* `roscore`  
* Activate your tensorflow environment (if any).
* `roslaunch tf2bb server_code.launch`
* `ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc`