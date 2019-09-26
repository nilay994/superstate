#ifndef PID_NODE_H
#define PID_NODE_H

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


#define KP_POS_X 2
#define KP_POS_Y 2

#define KD_POS_X 0.02
#define KD_POS_Y 0.02

#define KP_VEL_X 0.02
#define KP_VEL_Y 0.02
#define K_FF_THETA 0.03
#define K_FF_PHI   0.03
#define KD_VEL_X 0.8
#define KD_VEL_Y 0.8

#define MAX_VEL_X 6
#define MAX_VEL_Y 6
#define MAX_BANK 0.43

// bound a value to a range [min,max]
inline double bound_f(double val, double min, double max) {
	if (val > max) {
		val = max;
	} else if (val < min) {
		val = min;
	}
	return val;
}

#endif