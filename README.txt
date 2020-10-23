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

ssh teamc@192.168.0.116

Landing:
roslaunch dji_m100_landing apriltag_track_landing.launch
ROS_NAMESPACE=usb_cam rosrun image_proc image_proc
rostopic pub -r 10 dji_landing/landing_enable std_msgs/Bool 'true'


About Gains:
p
d: Greater or equal to 0.25 will fuck up everything
