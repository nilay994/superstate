
#include "attitude_control.h"


// rate_hz
double rate_hz = 960.0;


// estimated eulers
double roll_est = 0;
double pitch_est = 0;
double yaw_est = 3.142/2;

// keyboard inputs
double roll_cmd = 0;
double pitch_cmd = 0;
double yawRate_cmd = 0;


double vel_z_cmd = 0;


// for Vel control
bool firstMsg = 1;


// for outerloop
double height_est = 1;
double x_est = 18;
double y_est = -23;
double z_est = 0;

double xVel_est = 0;
double yVel_est = 0;
double zVel_est = 0;


double kp_roll = 100.0;
double kd_roll = 10;

double kp_pitch = 100.0;
double kd_pitch = 10;

double kp_yaw = 2.2;
double kd_yaw = 0.03;

double kp_yawRate = 1.5;  
double kp_hVel = 10.0;
double zVel_cmd = 9.81;

attitudeNode::attitudeNode(ros::NodeHandle nh)
{
  // keyboard_sub = nh.subscribe("/controller/input/keyboard", 1000, &attitudeNode::keyboard_cb, this);
  optimalcmd_sub = nh.subscribe("/optimalcmd", 1, &attitudeNode::optimalcmd_cb, this);
  gt_sub = nh.subscribe("/tf", 1000, &attitudeNode::gtCallback, this);
  imu_sub = nh.subscribe("/uav/sensors/imu", 1000, &attitudeNode::imu_cb, this);
  
  // publish to drone dynamics 
  angular_rate_cmd_pub = nh.advertise<mav_msgs::RateThrust>("/uav/input/rateThrust", 1);
}

attitudeNode::~attitudeNode()
{
}


double p = 0;
double q = 0;
double r = 0;
/* imu callback for model prediction */
void attitudeNode::imu_cb(const sensor_msgs::Imu &imuMsg)
{
  double acc_x = 0;
  double acc_y = 0;
  double acc_z = 0;

  // imu_time = imuMsg.header.stamp.toSec(); 
  p = imuMsg.angular_velocity.x;
  q = imuMsg.angular_velocity.y;
  r = imuMsg.angular_velocity.z;

  acc_x = imuMsg.linear_acceleration.x;
  acc_y = imuMsg.linear_acceleration.y;
  acc_z = imuMsg.linear_acceleration.z;
}

void attitudeNode::optimalcmd_cb(const mav_msgs::RateThrust &cmd)
{
  roll_cmd    = cmd.angular_rates.x;
	pitch_cmd   = cmd.angular_rates.y;  
	yawRate_cmd = cmd.angular_rates.z;
	zVel_cmd    = cmd.thrust.z;
}


void attitudeNode::gtCallback(const tf2_msgs::TFMessage &groundTruth_msg) 
{
  static double curr_error_roll = 0;
  static double curr_error_pitch = 0;
  static double diff_roll = 0;
  static double old_error_roll = 0;
  static double diff_pitch = 0;
  static double old_error_pitch = 0;

  static double dt_est = 0;

  static double timeStamp_old = 0;
  static double x_est_old = 0;
  static double y_est_old = 0;
  static double z_est_old = 0;
  
  x_est = groundTruth_msg.transforms[0].transform.translation.x;
  y_est = groundTruth_msg.transforms[0].transform.translation.y;
  z_est = groundTruth_msg.transforms[0].transform.translation.z;

  if(firstMsg == 1) {

    x_est_old = x_est;
    y_est_old = y_est;
    z_est_old = z_est;
    timeStamp_old = groundTruth_msg.transforms[0].header.stamp.toSec();

    firstMsg = 0;
    xVel_est = 0;
    yVel_est = 0;
    zVel_est = 0;
  } 

  else {
    dt_est = groundTruth_msg.transforms[0].header.stamp.toSec() - timeStamp_old;

    xVel_est = (x_est - x_est_old) / dt_est;
    yVel_est = (y_est - y_est_old) / dt_est;
    zVel_est = (z_est - z_est_old) / dt_est;
  }
  
  double qx = groundTruth_msg.transforms[0].transform.rotation.x;
  double qy = groundTruth_msg.transforms[0].transform.rotation.y;
  double qz = groundTruth_msg.transforms[0].transform.rotation.z;
  double qw = groundTruth_msg.transforms[0].transform.rotation.w;
  // tf::Quaternion q(groundTruth_msg.transform.rotation.x, groundTruth_msg.transform.rotation.y, groundTruth_msg.transform.rotation.z, groundTruth_msg.transform.rotation.w);
  // tf::Matrix3x3 R(q);
  // R.getEuler
  // from quaterion to Euler in degree
  roll_est  = (atan2(2*qx*qw + 2*qy*qz, 1 - 2*qx*qx - 2*qy*qy));
  pitch_est = (asin(2*qw*qy - 2*qz*qx));
  yaw_est   = (atan2(2*qy*qx + 2*qw*qz, 1 - 2*qy*qy - 2*qz*qz));

	double d_roll_cmd  = kp_roll  * (roll_cmd - roll_est)   - kd_roll  * p; //PID_attitude_controller.roll_cmd * 0.3
	double d_pitch_cmd = kp_pitch * (pitch_cmd - pitch_est) - kd_pitch * q; //PID_attitude_controller.pitch_cmd * 0.3
	double d_yaw_cmd   = yawRate_cmd * kp_yawRate;

	angular_rate_cmd.thrust.z = 9.81 / (cos(roll_est) * cos(pitch_est)) - 0.3 * zVel_est + 0.4 * (4 - z_est); // Gate3 is at 4m
	// ROS_INFO_STREAM("thrust f " << angular_rate_cmd.thrust.z);
	angular_rate_cmd.angular_rates.x = d_roll_cmd - sin(pitch_est) * d_yaw_cmd;
	angular_rate_cmd.angular_rates.y = cos(roll_est) * d_pitch_cmd + cos(pitch_est) * sin(roll_est) * d_yaw_cmd;
	angular_rate_cmd.angular_rates.z = - sin(roll_est) * d_pitch_cmd + cos(pitch_est) * cos(roll_est) * d_yaw_cmd;
	// angular_rate_cmd.angular_rates.x = d_roll_cmd;
	// angular_rate_cmd.angular_rates.y = d_pitch_cmd;
	// angular_rate_cmd.angular_rates.z = d_yaw_cmd;

	angular_rate_cmd_pub.publish(angular_rate_cmd);
  
	timeStamp_old = groundTruth_msg.transforms[0].header.stamp.toSec();
	x_est_old = x_est;
	y_est_old = y_est;
	z_est_old = z_est;
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "controllerMavlab_node");

  // double timeLength;
  // double timeInitial;

  ros::NodeHandle nh;

  attitudeNode attitudeNode(nh);

  // 960 Hz control loop
  ros::Rate rate(rate_hz);

  firstMsg = 1;

  while (ros::ok()) {

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}


