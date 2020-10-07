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

#include <algorithm>

#include <sensor_msgs/Joy.h>

#include "dji_sdk/dji_sdk.h"
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/QueryDroneVersion.h>




ros::Subscriber velocity_control_x_sub;
ros::Subscriber velocity_control_y_sub;
ros::Subscriber control_z_sub;
ros::Subscriber velocity_control_yaw_sub;
ros::Subscriber landing_enable_sub;
ros::Subscriber local_position_sub;
ros::Subscriber flight_status_sub;

ros::Publisher ctrlVelYawPub;
ros::Publisher ctrlRPYPub;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t current_gps_health = 0;
sensor_msgs::NavSatFix current_gps_position;
bool destination_reached = false;
geometry_msgs::PointStamped local_position;

// ros::ServiceClient velocity_control_service;
ros::ServiceClient drone_task_service;

double velocity_control_effort_x;
double velocity_control_effort_y;
double control_effort_z;
double velocity_control_effort_yaw;

const double descending_speed = -0;
//const double ascending_speed = 0.5;

bool landing_enabled = false;
bool during_landing = false;
bool continue_landing = false;
double local_z;
double local_x;
double local_y;
double landing_height_threshold = 1;

std::string topic_from_controller;

void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& local_position_msg)
{
  local_x = local_position_msg->point.x;
  local_y = local_position_msg->point.y;
  local_z = local_position_msg->point.z;
  if (local_z >= 3.5 && local_x >= 0.6 && local_y >= 0.6){
        destination_reached = true;
  }
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


void landingEnableCallback(const std_msgs::Bool& landing_enable_msg)
{
	landing_enabled = landing_enable_msg.data;
}

void flightStatusCallback(const std_msgs::UInt8& flight_status_msg)
{
  flight_status = (int)flight_status_msg.data;
}

bool obtain_control(){
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
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



bool takeoff_land(int task) {
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

/*!
* This function demos how to use M100 flight_status
* to monitor the take off process with some error
* handling. Note M100 flight status is different
* from A3/N3 flight status.
*/
bool M100monitoredTakeoff() {
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps_position.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
 ROS_INFO("flight_status:%d \n", flight_status);
  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR)
    {
      ROS_ERROR("Takeoff failed.");
      return false;
    }
    else
    {
      start_time = ros::Time::now();
      ROS_INFO("Successful takeoff!");
      ros::spinOnce();
    }
    return true;
  }

bool set_local_position() {
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference.call(localPosReferenceSetter);

    return (bool)localPosReferenceSetter.response.result;
  }


int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_track_controller");
	ros::NodeHandle nh;

	nh.param<std::string>("topic_from_controller", topic_from_controller, "/teamc/position_track_x/control_effort");

	velocity_control_x_sub = nh.subscribe("/teamc/position_track_x/control_effort", 100, velocityControlEffortXCallback);
	velocity_control_y_sub = nh.subscribe("/teamc/position_track_y/control_effort", 100, velocityControlEffortYCallback);
	control_z_sub = nh.subscribe("/teamc/position_track_z/control_effort", 100, ControlEffortZCallback);
	velocity_control_yaw_sub = nh.subscribe("/teamc/position_track_yaw/control_effort", 100, velocityControlEffortYawCallback);
  local_position_sub = nh.subscribe("dji_sdk/local_position", 100, localPositionCallback);
	landing_enable_sub = nh.subscribe("/dji_landing/landing_enable", 1, landingEnableCallback );
  flight_status_sub = nh.subscribe("/dji_sdk/flight_status", 1, flightStatusCallback);

	drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");

	ctrlVelYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 100);
	ctrlRPYPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 100);
	ros::Rate loop_rate(100);

  //Publishers
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

  // Basic services
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  if (!set_local_position())
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }


  ROS_INFO("M100 taking off!");
  takeoff_result = M100monitoredTakeoff();
  if(takeoff_result) {
    ROS_INFO("Take off successful!");
  }

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
			ROS_INFO_ONCE("Landing is enabled, start landing.");
		  if(local_z > landing_height_threshold)
			 {

				// Set the Joy message and publish to flight_control_setpoint_ENUposition_yaw topic
				ROS_DEBUG_ONCE("Velocity controller: Landing condition met, going down");
				sensor_msgs::Joy controlRPzY;
				//ROS_INFO_STREAM("control_effort_z: " << control_effort_z);
				controlRPzY.axes.push_back(velocity_control_effort_x);
				controlRPzY.axes.push_back(velocity_control_effort_y);
				controlRPzY.axes.push_back(control_effort_z);
				controlRPzY.axes.push_back(velocity_control_effort_yaw);
				ctrlRPYPub.publish(controlRPzY);

				ROS_INFO_STREAM("effort_x: " << velocity_control_effort_x << " effort_y: " << velocity_control_effort_y);
				ROS_INFO_STREAM("effort_z: " << descending_speed << " effort_yaw: " << velocity_control_effort_yaw);

				during_landing = true;
				continue_landing = true;
			}
			else
			{
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

    else if(!destination_reached){
      sensor_msgs::Joy controlPosYaw;
      controlPosYaw.axes.push_back(0.1);
      controlPosYaw.axes.push_back(0.1);
      controlPosYaw.axes.push_back(3.6);
      controlPosYaw.axes.push_back(1.6);
      ctrlPosYawPub.publish(controlPosYaw);
    }
	}
}
