## Instructions for setting up the remote, for running the code.

* Exporting environment variables on the **remote** machine:  
    * Check machine-ip using `ifconfig`
    * `export ROS_MASTER_URI=http://<romote-machine-ip>:11311`
    * `export ROS_IP=<remote-machine-ip>`

* Exporting environment variables on the **local** machine:  
    * Check machine-ip using `ifconfig`
    * `export ROS_MASTER_URI=http://<romote-machine-ip>:11311`
    * `export ROS_IP=<local-machine-ip>`

* Run the `roscore` on **remote-machine**.
* Run `roslaunch hector_quadrotor_demo custom_flight.launch` on the **local-machine**.
