source devel_isolated/pointcloud_to_laserscan/setup.bash



To compile for landing:



Simulate bridge:
rosrun dji_m100_gazebo dji_m100_pcsim_gazebo_bridge



Calibrate the camera:
rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.108 image:=/usb_cam/image_raw camera:=/usb_cam


Simulation:
roslaunch dji_sdk sdk.launch 
roslaunch dji_m100_gazebo simulate_dji_m100_gazebo.launch
rosrun dji_m100_gazebo dji_m100_pcsim_gazebo_bridge
 
roslaunch dji_m100_landing tunning.launch 


Landing:
roslaunch dji_m100_landing apriltag_track_landing.launch
ROS_NAMESPACE=usb_cam rosrun image_proc image_proc
rostopic pub -r 10 dji_landing/landing_enable std_msgs/Bool 'true'

## Multi-Master Setup

1. Put both computer's IP address and hostname in /etc/hosts
2. Put {export ROS_MASTER_URI=htttp://<IP_Adress>:11311} and {export ROS_HOSTNAME=<IP_Adress>} in .bashrc for both computers
3. Run {sudo sh -c "echo 0 > /proc/sys/net/ipv4/icmp_echo_ignore_broadcasts"} and then {sudo service procps restart} on both computers
4. On computer A, start roscore and then {rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1}
5. On computer B, start roscore and then {rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1 __name:=ugv_master}
6. On computer A, run {rosrun master_sync_fkie master_sync}
7. On computer B, run {rosrun master_sync_fkie master_sync __name:=ugv_sync}



