## Instructions to set up the  **local-machine** or **`onboard_computer`** for running the code.

* Exporting environment variables on your local machine:  
    * Check machine-ip using `ifconfig`
    * `export ROS_MASTER_URI=http://<romote-machine-ip>:11311`
    * `export ROS_IP=<local-machine-ip>`

**Note**: You need to export Environment Variable on **all terminals the involved**. 

Execute the following (prefereably in this order)
* `roslaunch hector_quadrotor_demo custom_flight.launch`
* `rosservice call /enable_motors "enable: true"`
* `cd follow_quad/follow_quad/` --> `python bb_repub.py`
* `rosrun follow_quad thesisDone`
