
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

#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

#include <tag.h>

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


ros::Subscriber apriltags_36h11_sub;
ros::Subscriber local_position_sub;
ros::Subscriber landing_enable_sub;
ros::Subscriber attitude_quaternion_sub;
ros::Subscriber flight_status_sub;
ros::Subscriber global_position_sub;

ros::Publisher setpoint_x_pub;
ros::Publisher setpoint_y_pub;
ros::Publisher setpoint_yaw_pub;
ros::Publisher found_april_tag_pub;
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

Tag *tag_36h11_0;
Tag *tag_36h11_1;
Tag *tag_36h11;

double yaw_error = 0.0;
double offset_x;
double offset_y;
double error_threshold;
double error_threshold_small_tag;
double transition_height;

std::string tag_36h11_detection_topic;

bool found_36h11 = false;
bool found_36h11_0 = false;
bool found_36h11_1 = false;
bool landing_enabled = false;

std_msgs::Float64 setpoint_x_msg;
std_msgs::Float64 setpoint_y_msg;
std_msgs::Float64 control_z_msg;
std_msgs::Float64 setpoint_yaw_msg;
std_msgs::Bool found_tag_msg;

std_msgs::Float64 x_state_msg;
std_msgs::Float64 y_state_msg;
std_msgs::Float64 z_state_msg;
std_msgs::Float64 yaw_state_msg;

geometry_msgs::Vector3Stamped current_velocity;
sensor_msgs::Imu current_imu;


tf::Quaternion tmp_;

#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

tfScalar yaw, pitch, roll;

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  current_velocity = *msg;
}

void imuMsgCallback(const sensor_msgs::Imu& imu_msg)
{
  current_imu = imu_msg;
  tf::quaternionMsgToTF(imu_msg.orientation, tmp_);
  tf::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);
}


void apriltags36h11Callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    found_36h11 = false;
    return;
  }
  else
  {
    found_36h11 = true;
    for(auto it = std::begin(apriltag_pos_msg->detections); it != std::end(apriltag_pos_msg->detections); ++ it)
    {
      if((*it).id[0] == 0)
      {
        tag_36h11_0->updateTagState((*it).pose.pose.pose);
        found_36h11_0 = true;
      }
      else if((*it).id[0] == 1)
      {
        tag_36h11_1->updateTagState((*it).pose.pose.pose);
        found_36h11_1 = true;
      }
    }
  }
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
  ROS_INFO("Listening to 36h11 apriltag detection topic: %s", tag_36h11_detection_topic.c_str());
  ROS_INFO("landing_height_threshold: %f", landing_height_threshold);
  ROS_INFO("landing_center_threshold: %f", landing_center_threshold);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "track_apriltag_landing_node");
  ROS_INFO("Starting apriltag track and landing publisher");
  ros::NodeHandle nh;
  ros::NodeHandle node_priv("~");

  node_priv.param<std::string>("tag_36h11_detection_topic", tag_36h11_detection_topic, "/tag_detections");
  node_priv.param<double>("error_threshold", error_threshold, 0.10);
  node_priv.param<double>("transition_height", transition_height, 1.3);
  node_priv.param<double>("error_threshold_small_tag", error_threshold_small_tag, 0.05);
  node_priv.param<double>("offset_x", offset_x, 0.0);
  node_priv.param<double>("offset_y", offset_y, 0.0);
  node_priv.param<double>("landing_height_threshold", landing_height_threshold, 1);
  node_priv.param<double>("landing_center_threshold", landing_center_threshold, 0.5);

  print_parameters();

  setpoint_x_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_x/setpoint", 100);
  setpoint_y_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_y/setpoint", 100);
  setpoint_yaw_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_yaw/setpoint", 100);
  found_april_tag_pub = nh.advertise<std_msgs::Bool>("/teamc/found_april_tag_pub", 10);
  yaw_state_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_yaw/state", 10);
  x_state_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_x/state", 100);
  y_state_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_y/state", 100);
  z_state_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_z/state", 100);
  z_control_pub = nh.advertise<std_msgs::Float64>("/teamc/position_track_z/control_effort", 100);

  apriltags_36h11_sub = nh.subscribe(tag_36h11_detection_topic, 1, apriltags36h11Callback);
  local_position_sub = nh.subscribe("dji_sdk/local_position", 10, localPositionCallback);
  attitude_quaternion_sub = nh.subscribe("/dji_sdk/attitude", 1, attitudeQuaternionCallback );
  flight_status_sub = nh.subscribe("/dji_sdk/flight_status", 1, flightStatusCallback);
  global_position_sub = nh.subscribe("/dji_sdk/gps_position", 10, globalPositionCallback);
  landing_enable_sub = nh.subscribe("/dji_landing/landing_enable", 1, landingEnableCallback );
  ros::Subscriber imu_subscriber = nh.subscribe("dji_sdk/imu", 100, imuMsgCallback);
  ros::Subscriber velocitySub = nh.subscribe("dji_sdk/velocity", 100, &velocity_callback);




  tag_36h11_0 = new Tag();
  // Set the translation between camera and landing center
  tag_36h11_0->setToLandingCenterTranslation(Eigen::Vector3d(offset_x, offset_y, 0.0));

  tag_36h11_1 = new Tag();
  // Set the translation between camera and landing center
  tag_36h11_1->setToLandingCenterTranslation(Eigen::Vector3d(offset_x, offset_y, 0.0));

  //camera to drone transformation
  Eigen::Matrix3d camera_to_drone_transformation;
  camera_to_drone_transformation << 0, -1, 0,
  -1, 0, 0,
  0, 0, -1;

  Eigen::Vector3d landing_center_position;

  ros::Rate loop_rate(40);

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
    if(landing_enabled)
    {
      ROS_INFO_ONCE("Landing is enabled.");
      found_tag_msg.data = found_36h11;
      found_april_tag_pub.publish(found_tag_msg);
      // Found apriltag, start landing
      control_z_msg.data = 0;
      double curr_error;
      if (found_36h11_1 && local_z < transition_height){
        tag_36h11 = tag_36h11_1;
        ROS_INFO_ONCE("Tag 1\n");
      }
      else if (found_36h11_0){
        tag_36h11 = tag_36h11_0;
        ROS_INFO_ONCE("Tag 0\n");
      }
      else{
        ROS_INFO_ONCE("NO Tag\n");
      }
      if(found_36h11)
      {
        // get the landing center position refer to the local frame
        tag_36h11->calculateDroneFramePosition(camera_to_drone_transformation);
        tag_36h11->calculateDroneFrameOrientation(camera_to_drone_transformation);
        landing_center_position = tag_36h11->getLandingCenterPosition();
        yaw_error = (tag_36h11_0->getYawError())/ M_PI * 180 + 90;

        double yaw_angle_radian = (yaw_state/180)* M_PI;
        double delta_x = landing_center_position(0)*cos(yaw) - landing_center_position(1)*sin(yaw);
        double delta_y = landing_center_position(0)*sin(yaw) + landing_center_position(1)*cos(yaw);

        curr_error = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
        setpoint_x = delta_x;
        setpoint_y = delta_y;
        ROS_INFO_ONCE("VISION\n");
        }

        else{
          setpoint_x = -local_x;
          setpoint_y = -local_y;
          curr_error = sqrt(pow(local_x, 2) + pow(local_y, 2));
          ROS_INFO_ONCE("GPS\n");
        }
        x_state_msg.data = current_velocity.vector.x;
        y_state_msg.data = current_velocity.vector.y;
        double curr_threshold = error_threshold;
        if (found_36h11_1 && local_z < transition_height){
          curr_threshold = error_threshold_small_tag;
        }
        if(curr_error < curr_threshold){
          control_z_msg.data = -0.1;
        }

        setpoint_yaw_msg.data = yaw_error;
        setpoint_x_msg.data = setpoint_x;
        setpoint_y_msg.data = setpoint_y;

        z_control_pub.publish(control_z_msg);

        setpoint_x_pub.publish(setpoint_x_msg);
        setpoint_y_pub.publish(setpoint_y_msg);
        setpoint_yaw_pub.publish(setpoint_yaw_msg);

        //x_state_msg.data = current_velocity.vector.x;
        //y_state_msg.data = current_velocity.vector.y;
        z_state_msg.data = landing_center_position(2);
        yaw_state_msg.data = current_imu.angular_velocity.z;

        x_state_pub.publish(x_state_msg);
        y_state_pub.publish(y_state_msg);
        z_state_pub.publish(z_state_msg);
        yaw_state_pub.publish(yaw_state_msg);

				// ROS_INFO_STREAM("setpoint_x: " << setpoint_x_msg.data << " setpoint_y: " << setpoint_y_msg.data);
        // ROS_INFO_STREAM("local_x: " << local_x << " local_y: " << local_y);
        // ROS_INFO_STREAM("delta_x: " << delta_x << " delta_y: " << delta_y);
        // ROS_INFO_STREAM("yaw_state: " << yaw_state_msg.data << " yaw_error: " << yaw_error);

      loop_rate.sleep();
    }
  }
}
