#ifndef ATTITUDE_NODE_H
#define ATTITUDE_NODE_H

//#include <stdio.h>
//#include <memory.h>
//#include <ros/callback_queue.h>


//#include <Eigen/Eigen>

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// Messages
#include <mav_msgs/RateThrust.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <rosgraph_msgs/Clock.h>


#include <sstream>
#include <fstream>
#include <string>
#include <iostream>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#define ToDeg(x) (x*57.2957795131)  // *180/pi
#define ToRad(x) (x*0.01745329252)  // *pi/180

#define pi 3.14159265

#define maxSpeedYbody 12

using namespace std;

float pathPredict(float x0[2], float v0[2],
                  float xd[2], float vd[2], 
                  float phi0, float phi1, float t1, 
                  float *xt, float *vt);
 void find_optimal(float *phi0, float *phi1, float *switch_time);


class attitudeNode
{
public:
  attitudeNode(ros::NodeHandle nh);
  ~attitudeNode();

  //publishers
  ros::Publisher angular_rate_cmd_pub;
  mav_msgs::RateThrust angular_rate_cmd;

  // subscribers
  ros::Subscriber keyboard_sub;
  ros::Subscriber odometry_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber gt_sub;


  void keyboard_cb(const mav_msgs::RateThrust &command);
  void gtCallback(const tf2_msgs::TFMessage &groundTruth_msg);
  void imu_cb(const sensor_msgs::Imu &imuMsg);
  /*
  void OdometryCallback(const nav_msgs::Odometry &odom_msg); 
  
  // void VelCallback(const geometry_msgs::Vector3Stamped &vel_msg);
  // void gtCallback(const tf2_msgs::TFMessage &groundTruth_msg); 
  void IMUCallback(const sensor_msgs::Imu &currentImu);
  void tofCallback(const sensor_msgs::Range &currentTof);
  */




};


#endif