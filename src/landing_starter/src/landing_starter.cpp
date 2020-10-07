/** @file base_controller
*  @version 1.0
*  @date 3/11/2020
*
*  @brief
*  Subscribes to vel_cmd and publish it to the drone via velocity_yawrate
*
*/

#include "landing_starter.h"

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t current_gps_health = 0;
sensor_msgs::NavSatFix current_gps_position;
bool landing_enabled = false;
bool destination_reached = false;
geometry_msgs::PointStamped local_position;

int main(int argc, char** argv) {
  ros::init(argc, argv, "landing_starter_node");
  ros::NodeHandle nh;
  // Subscribe to messages from dji_sdk_node

  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber gpsSub          = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
	ros::Subscriber landing_enable_sub = nh.subscribe("/dji_landing/landing_enable", 1, landingEnableCallback );
	ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 100, &local_position_callback);

  // Basic services
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");

  //Publishers
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

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
  ros::Rate r(30.0);
  while(nh.ok() && !landing_enabled && !destination_reached)
	{
		ros::spinOnce();
    sensor_msgs::Joy controlPosYaw;
    controlPosYaw.axes.push_back(0.1);
    controlPosYaw.axes.push_back(0.1);
    controlPosYaw.axes.push_back(3.6);
    controlPosYaw.axes.push_back(1.6);
    ctrlPosYawPub.publish(controlPosYaw);
    r.sleep();
  }

  return 0;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  local_position = *msg;
  if (local_position.point.z >= 3.5 && local_position.point.x >= 0.1 && local_position.point.z >= 0.1){
        destination_reached = true;
      }
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
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

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  current_gps_position = *msg;
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

  void landingEnableCallback(const std_msgs::Bool& landing_enable_msg)
  {
  	landing_enabled = landing_enable_msg.data;
  }

bool set_local_position() {
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference.call(localPosReferenceSetter);

    return (bool)localPosReferenceSetter.response.result;
  }
