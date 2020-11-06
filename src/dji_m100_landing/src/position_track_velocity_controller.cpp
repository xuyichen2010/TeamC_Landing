#include <ros/ros.h>
#include <ros/console.h>

#include <vector>
#include <iterator>
#include <string>

#include <sensor_msgs/Joy.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/PoseArray.h>
#include <cmath>
#include <iostream>

#include <Eigen/Geometry>


#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <ros/console.h>

#include <tag.h>
#include <algorithm>

#include <sensor_msgs/Joy.h>

#include "dji_sdk/dji_sdk.h"
#include <dji_sdk/DroneTaskControl.h>

ros::Subscriber velocity_control_x_sub;
ros::Subscriber velocity_control_y_sub;
ros::Subscriber control_z_sub;
ros::Subscriber velocity_control_yaw_sub;
ros::Subscriber landing_enable_sub;
ros::Subscriber found_tag_sub;
ros::Subscriber local_position_sub;
ros::Subscriber flight_status_sub;
ros::Subscriber z_state_sub;

ros::Publisher ctrlVelYawPub;
ros::Publisher ctrlRPYPub;

ros::ServiceClient drone_task_service;

double velocity_control_effort_x;
double velocity_control_effort_y;
double control_effort_z;
double velocity_control_effort_yaw;
int flight_status;

bool landing_enabled = false;
bool found_tag = false;
bool during_landing = false;
bool continue_landing = false;
double local_z;
double local_x;
double local_y;
double z_state;
double landing_height_threshold = 1;

std::string topic_from_controller;

void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& local_position_msg)
{
  local_x = local_position_msg->point.x;
  local_y = local_position_msg->point.y;
  local_z = local_position_msg->point.z;
}

void velocityControlEffortXCallback(std_msgs::Float64 velocity_control_effort_x_msg)
{
	velocity_control_effort_x = velocity_control_effort_x_msg.data;
}

void velocityControlEffortYCallback(std_msgs::Float64 velocity_control_effort_y_msg)
{
	velocity_control_effort_y = velocity_control_effort_y_msg.data;
}

void ControlEffortZCallback(std_msgs::Float64 control_effort_z_msg)
{
	control_effort_z = control_effort_z_msg.data;
}

void velocityControlEffortYawCallback(std_msgs::Float64 velocity_control_effort_yaw_msg)
{
	velocity_control_effort_yaw = velocity_control_effort_yaw_msg.data;
}

void foundapriltagCallback(const std_msgs::Bool& found_tag_msg){
	found_tag = found_tag_msg.data;
}

void landingEnableCallback(const std_msgs::Bool& landing_enable_msg)
{
	landing_enabled = landing_enable_msg.data;
}

void flightStatusCallback(const std_msgs::UInt8& flight_status_msg)
{
  flight_status = (int)flight_status_msg.data;
}

void zStateCallback(std_msgs::Float64 z_state_msg)
{
	z_state = z_state_msg.data;
}

bool land()
{
	ROS_INFO("land().");
	dji_sdk::DroneTaskControl droneTaskControl;
	droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_LAND;
	drone_task_service.call(droneTaskControl);
	if(!droneTaskControl.response.result)
	{
		ROS_ERROR("landing fail");
		return false;
	}
	ROS_INFO("Return TRue.");
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_track_controller");
	ros::NodeHandle nh;
  ros::NodeHandle node_priv("~");
  node_priv.param<double>("landing_height_threshold", landing_height_threshold, 1);
	nh.param<std::string>("topic_from_controller", topic_from_controller, "/teamc/position_track_x/control_effort");

	velocity_control_x_sub = nh.subscribe("/teamc/position_track_x/control_effort", 100, velocityControlEffortXCallback);
	velocity_control_y_sub = nh.subscribe("/teamc/position_track_y/control_effort", 100, velocityControlEffortYCallback);
	control_z_sub = nh.subscribe("/teamc/position_track_z/control_effort", 100, ControlEffortZCallback);
	velocity_control_yaw_sub = nh.subscribe("/teamc/position_track_yaw/control_effort", 100, velocityControlEffortYawCallback);
  local_position_sub = nh.subscribe("dji_sdk/local_position", 100, localPositionCallback);
	landing_enable_sub = nh.subscribe("dji_landing/landing_enable", 1, landingEnableCallback );
	found_tag_sub = nh.subscribe("/teamc/found_april_tag_pub", 1, foundapriltagCallback );
  flight_status_sub = nh.subscribe("dji_sdk/flight_status", 1, flightStatusCallback);
  z_state_sub = nh.subscribe("/teamc/position_track_z/state", 1, zStateCallback);

	drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");

	ctrlVelYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 100);
	ctrlRPYPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 100);
	ros::Rate loop_rate(30);

	while(ros::ok())
	{
		ros::spinOnce();

		if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR)
		{
			ROS_ERROR_ONCE("The UAV is not in air, landing cannot start!");
			continue;
		}

		// Check if received command for landing
		if(landing_enabled)
		{
			ROS_INFO_ONCE("Landing is enabled.");
			// Found apriltag, start landing
      double curr_height = local_z;
      if (found_tag && z_state < -0.01){
        curr_height = -z_state;
      }
      // ROS_INFO_STREAM("GPS_height: " << local_z << " visual_height: " << -z_state);
		  if(curr_height > landing_height_threshold)
			 {

				ROS_INFO_ONCE("Found Apriltag, start landing.");

				// Set the Joy message and publish to flight_control_setpoint_ENUposition_yaw topic
				ROS_DEBUG_ONCE("Velocity controller: Landing condition met, going down");
        sensor_msgs::Joy controlVel;
        //ROS_INFO_STREAM("control_effort_z: " << control_effort_z);
        controlVel.axes.push_back(velocity_control_effort_x);
        controlVel.axes.push_back(velocity_control_effort_y);
        controlVel.axes.push_back(control_effort_z);
        controlVel.axes.push_back(velocity_control_effort_yaw);
        //controlVel.axes.push_back(0);
        ctrlVelYawPub.publish(controlVel);

				//ROS_INFO_STREAM("effort_x: " << velocity_control_effort_x << " effort_y: " << velocity_control_effort_y);
				ROS_INFO_STREAM(" effort_yaw: " << velocity_control_effort_yaw);

				during_landing = true;
				continue_landing = true;
			}
			else
			{
        ROS_INFO_STREAM("Landing Height: " << z_state << "\n");
				if (land())
				{
					ROS_INFO_ONCE("Continue landing.");
					if(flight_status == DJISDK::M100FlightStatus::M100_STATUS_ON_GROUND)
					{
						ROS_INFO_ONCE("Successful landing!");
						during_landing = false;
						continue_landing = false;
					}
				}
			}

			if(flight_status == DJISDK::M100FlightStatus::M100_STATUS_FINISHED_LANDING){
				ROS_INFO_ONCE("Successful landing!");
			}
			loop_rate.sleep();
		}
	}
}
