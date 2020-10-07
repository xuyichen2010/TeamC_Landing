
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <ros/console.h>

#include <cmath>
#include <Eigen/Geometry>

#include <vector>
#include <iterator>
#include <algorithm>

#include <sensor_msgs/Joy.h>

#include "dji_sdk/dji_sdk.h"
#include <dji_sdk/DroneTaskControl.h>
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"

ros::Subscriber local_position_sub;
ros::Subscriber landing_enable_sub;
ros::Subscriber attitude_quaternion_sub;
ros::Subscriber flight_status_sub;
ros::Subscriber global_position_sub;

ros::Publisher setpoint_x_pub;
ros::Publisher setpoint_y_pub;
ros::Publisher setpoint_yaw_pub;
ros::Publisher yaw_state_pub;
ros::Publisher x_state_pub;
ros::Publisher y_state_pub;
ros::Publisher z_state_pub;
ros::Publisher z_control_pub;


ros::ServiceClient drone_task_service;

double local_x;
double local_y;
double local_z;
double flight_height;

double heading_q0;
double heading_q1;
double heading_q2;
double heading_q3;

Eigen::Quaternion<double> drone_heading;
double yaw_state = 0;

double setpoint_x = 0;
double setpoint_y = 0;
double setpoint_z = 0;
double setpoint_yaw = 0;

double  target_x = 0;
double  target_y = 0;
bool    target_captured = false;

double landing_height_threshold = 1;
double landing_center_threshold = 0.5;

int flight_status;

double yaw_error;


bool found_36h11 = false;
bool landing_enabled = false;

std_msgs::Float64 setpoint_x_msg;
std_msgs::Float64 setpoint_y_msg;
std_msgs::Float64 control_z_msg;
std_msgs::Float64 setpoint_yaw_msg;

std_msgs::Float64 x_state_msg;
std_msgs::Float64 y_state_msg;
std_msgs::Float64 z_state_msg;
std_msgs::Float64 yaw_state_msg;


tf::Quaternion tmp_;

#ifndef TF_MATRIX3x3_H
typedef btScalar tfScalar;
namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

tfScalar yaw, pitch, roll;

void imuMsgCallback(const sensor_msgs::Imu& imu_msg)
{
  tf::quaternionMsgToTF(imu_msg.orientation, tmp_);
  tf::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);
}


void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& local_position_msg)
{
  local_x = local_position_msg->point.x;
  local_y = local_position_msg->point.y;
  local_z = local_position_msg->point.z;
}

void attitudeQuaternionCallback(const geometry_msgs::QuaternionStamped::ConstPtr& attitude_quaternion_msg)
{
  heading_q0 = attitude_quaternion_msg->quaternion.w;
  heading_q1 = attitude_quaternion_msg->quaternion.x;
  heading_q2 = attitude_quaternion_msg->quaternion.y;
  heading_q3 = attitude_quaternion_msg->quaternion.z;

  drone_heading = Eigen::Quaternion<double>(heading_q0, heading_q1, heading_q2, heading_q3);
  yaw_state = atan2(2*(heading_q0 * heading_q3 + heading_q1 * heading_q2), 1 - 2 * (heading_q2 * heading_q2 + heading_q3 * heading_q3)) / M_PI * 180;
}

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& global_position_msg)
{
  flight_height = global_position_msg->altitude;
}

void landingEnableCallback(const std_msgs::Bool& landing_enable_msg)
{
  landing_enabled = landing_enable_msg.data;
}

void flightStatusCallback(const std_msgs::UInt8& flight_status_msg)
{
  flight_status = (int)flight_status_msg.data;
}


void print_parameters()
{
  ROS_INFO("landing_height_threshold: %f", landing_height_threshold);
  ROS_INFO("landing_center_threshold: %f", landing_center_threshold);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "track_apriltag_landing_node");
  ros::NodeHandle nh;
  ros::NodeHandle node_priv("~");

  node_priv.param<double>("landing_height_threshold", landing_height_threshold, 1);
  node_priv.param<double>("landing_center_threshold", landing_center_threshold, 0.5);

  print_parameters();

  setpoint_x_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_x/setpoint", 100);
  setpoint_y_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_y/setpoint", 100);
  setpoint_yaw_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_yaw/setpoint", 100);
  yaw_state_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_yaw/state", 10);
  x_state_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_x/state", 100);
  y_state_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_y/state", 100);
  z_state_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_z/state", 100);
  z_control_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_z/control_effort", 100);

  local_position_sub = nh.subscribe("dji_sdk/local_position", 10, localPositionCallback);
  attitude_quaternion_sub = nh.subscribe("/dji_sdk/attitude", 1, attitudeQuaternionCallback );
  flight_status_sub = nh.subscribe("/dji_sdk/flight_status", 1, flightStatusCallback);
  global_position_sub = nh.subscribe("/dji_sdk/gps_position", 10, globalPositionCallback);
  landing_enable_sub = nh.subscribe("/dji_landing/landing_enable", 1, landingEnableCallback );
  ros::Subscriber imu_subscriber = nh.subscribe("dji_sdk/imu", 100, imuMsgCallback);

  ros::Rate loop_rate(100);

  ros::spinOnce();

  while (ros::ok())
  {
    ros::spinOnce();

    //ROS_INFO("flight_status:%d \n", flight_status);
    //ROS_INFO("suppose:%d \n", DJISDK::M100FlightStatus::M100_STATUS_IN_AIR);

    // Check if the UAV is currently in air, if not in air, then UAV cannot land
    if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR)
    {
      ROS_ERROR_ONCE("The UAV is not in air, landing cannot start!");
      continue;
    }

    // Check if received command for landing
    if(landing_enabled){
      ROS_INFO_ONCE("Landing is enabled.");
      double curr_error = sqrt(pow(local_x, 2) + pow(local_y, 2));

      if(curr_error > 0.15)
      {
        control_z_msg.data = local_z;
      }
      else{
        control_z_msg.data = std::max(0.0, local_z-1.0);
      }
      //ROS_INFO_STREAM("curr_error: " << curr_error << " control_z_msg.data: " << control_z_msg.data);

      setpoint_x = 0;
      setpoint_y = 0;
      setpoint_yaw = 90;

      setpoint_yaw_msg.data = setpoint_yaw;
      setpoint_x_msg.data = setpoint_x;
      setpoint_y_msg.data = setpoint_y;

      z_control_pub.publish(control_z_msg);
      setpoint_x_pub.publish(setpoint_x_msg);
      setpoint_y_pub.publish(setpoint_y_msg);
      setpoint_yaw_pub.publish(setpoint_yaw_msg);

      x_state_msg.data = local_x;
      y_state_msg.data = local_y;
      z_state_msg.data = local_z;
      yaw_state_msg.data = yaw_state;

      x_state_pub.publish(x_state_msg);
      y_state_pub.publish(y_state_msg);
      z_state_pub.publish(z_state_msg);
      yaw_state_pub.publish(yaw_state_msg);

      // ROS_INFO_STREAM("setpoint_x: " << setpoint_x_msg.data << " setpoint_y: " << setpoint_y_msg.data);
      // ROS_INFO_STREAM("local_x: " << local_x << " local_y: " << local_y);
      // ROS_INFO_STREAM("delta_x: " << delta_x << " delta_y: " << delta_y);
      // ROS_INFO_STREAM("yaw_state: " << yaw_state_msg.data << " yaw_error: " << yaw_error)

      loop_rate.sleep();
    }
  }
}
