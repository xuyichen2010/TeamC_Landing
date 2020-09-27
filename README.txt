source devel_isolated/pointcloud_to_laserscan/setup.bash



To compile for landing:

rostopic pub -r 10 dji_landing/landing_enable std_msgs/Bool 'true'

Simulate bridge:
rosrun dji_m100_gazebo dji_m100_pcsim_gazebo_bridge
