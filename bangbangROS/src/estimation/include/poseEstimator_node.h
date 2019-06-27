#ifndef POSE_ESTIMATOR_NODE_H
#define POSE_ESTIMATOR_NODE_H

#include "gate.h"
#include "wayPointPlan.h"

#include <std_msgs/Empty.h>

namespace mav_estimation {

class poseEstimator
{
public:
  poseEstimator(ros::NodeHandle nh);
  ~poseEstimator();

  gate *gatePointer;
  wayPointPlan *flightPlanner;

  //PIDAttitudeController PID_attitude_controller;

  //publishers
  // ros::Publisher angular_rate_cmd_pub;
  // mav_msgs::RateThrust angular_rate_cmd;

  ros::Publisher odom_pub;
  nav_msgs::Odometry odom;

  ros::Publisher gt_pub;
  nav_msgs::Odometry gt;

  /*ros::Publisher wayPoint_pub;
  geometry_msgs::Vector3Stamped wayPointToPass; */

  ros::Publisher wayPoint_pub;
  nav_msgs::Odometry wayPointToPass;

  // ros::Publisher vel_pub;
  // geometry_msgs::Vector3Stamped vel;

  ros::Publisher pnpMeas_pub;
  geometry_msgs::Vector3Stamped pnpMeas;

  ros::Publisher finishTrack_pub;
  std_msgs::Empty finishTrack_msg;

  // subscribers
  ros::Subscriber command_roll_pitch_yawRate_heightRate_sub;
  // ros::Subscriber groundTruth_sub;
  ros::Subscriber filteredState_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber irBeacon_sub;
  ros::Subscriber crash_sub;
  ros::Subscriber gt_sub;

  void CommandRollPitchYawRateHeightRateCallback(const mav_msgs::RateThrust & command);
  // void GroundTruthCallback(const tf2_msgs::TFMessage & groundTruthMsg);
  void IMUCallback(const sensor_msgs::Imu & imuMsg);
  void FilteredStateCallback(const nav_msgs::Odometry &odom_msg) ;
  void IRMarkerCallback(const estimation::IRMarkerArrayConstPtr & IRMarkerArrayMsg); //, const estimation::IRMarkerArrayConstPtr & MarkerAmount);
  // void IRMarkerCallback(const estimation::IRMarkerArrayConstPtr & MarkerAmount);
  // bool got_first_attitude_command_;

  void crashCallback(const std_msgs::Empty &crash_msg);
  void gtCallback(const tf2_msgs::TFMessage &groundTruth_msg);
  void filter_predict(float phi, float theta, float psi, float dt, float acc_x, float acc_y, float acc_z);

  // void filter_vision_correct(void)

  void publish(ros::Time timeStamp);
  void pnpMeasPublish(ros::Time timeStamp);

  void ahrs(double gyro_x, double gyro_y, double gyro_z, double acc_x, double acc_y, double acc_z, double dt_imu);

  // void changeNextGateID();

  void wayPointGeneration(int gateID, double posi_x, double posi_y, double posi_z);

// private:
  double roll_gt_d;
  double pitch_gt_d;
  double yaw_gt_d;

  double roll_gt_r;
  double pitch_gt_r;
  double yaw_gt_r;

  double roll_cmd;
  double pitch_cmd;
  double yawRate_cmd;
  double heightRate_cmd;

  double qw_gt;
  double qx_gt;
  double qy_gt;
  double qz_gt;

  double x_error;
  double y_error;
  double z_error;

  double rollCmd;
  double pitchCmd;
  double yawCmd;

  double gyro_x;
  double gyro_y;
  double gyro_z;

  double acc_x;
  double acc_y;
  double acc_z;

  double imu_timeStamp;
  double gt_timeStamp;

  int markerAmount;
  int visibleGateAmount;

  int gateIDmarker;
  int markerIDmarker;

  int gateID;

  double roll_est;
  double pitch_est;
  double yaw_est;

  double height_est;
  double x_est;
  double y_est;

  double heightVel_est;
  double xVel_est;
  double yVel_est;

  double vel_w_x;
  double vel_w_y;
  double vel_w_z;

  double pos_w_x;
  double pos_w_y;
  double pos_w_z;

  bool firstMsg;

  double height_gt;
  double x_gt;
  double y_gt;
  double h_dot;
  double heightVel_gt;
  double xVel_gt;
  double yVel_gt;

  double dt_gt;
  double height_gt_old;
  double timeStamp_old;
  double x_gt_old;
  double y_gt_old;

  double timeStamp_imu_old;
  double dt_imu;
  bool imuFirstMsg;

  double phi_f;
  double theta_f;
  double psi_f;

  double pos_vision_x;
  double pos_vision_y;
  double pos_vision_z;

  // int nextGateID;
  int numPassedTargetGate;
  int targetGateID[11] = {10, 21, 2, 13, 9, 14, 1, 22, 15, 23, 6};

  tf2::Vector3 sum_error_arhs = {0,0,0};

  cv::Mat eulerAnglesToRotationMatrix(double roll, double pitch, double yaw);

  bool shutDown;
  bool crash;

// private:
//   template <class T>;
//   int getArrayLen(T& array);

};

}

#endif
