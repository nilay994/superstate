
#include "attitude_control.h"


#define KDX 0.57 
#define KDY 0.56

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

double vel_x_cmd = 0;
double vel_y_cmd = 0;
double vel_z_cmd = 0;

double d_roll_cmd = 0;
double d_pitch_cmd = 0;
double d_yaw_cmd = 0;


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

// estimated quaternion
double qw = 1;
double qx = 0;
double qy = 0;
double qz = 0;

// for Vel control
double dt_est = 0;

double timeStamp_old = 0;
double x_est_old = 0;
double y_est_old = 0;
double z_est_old = 0;

double thr_cmd = 9.81;


double curr_error_roll = 0;
double curr_error_pitch = 0;

double diff_roll = 0;
double old_error_roll = 0;
double diff_pitch = 0;
double old_error_pitch = 0;

// constructor initializes subscribers and publishers
attitudeNode::attitudeNode(ros::NodeHandle nh)
{
  //rollPitchYawRateHeightRateCommand
  // keyboard_sub = nh.subscribe("/controller/input/keyboard", 1000, &attitudeNode::keyboard_cb, this);
  optimalcmd_sub = nh.subscribe("/optimalcmd", 1000, &attitudeNode::optimalcmd_cb, this);
  gt_sub = nh.subscribe("/tf", 1000, &attitudeNode::gtCallback, this);
  imu_sub = nh.subscribe("/uav/sensors/imu", 1000, &attitudeNode::imu_cb, this);

  // publish to drone dynamics 
  angular_rate_cmd_pub = nh.advertise<mav_msgs::RateThrust>("/uav/input/rateThrust", 1);
  
  // check pid values
  if (!ros::param::get("~/kp_roll", kp_roll)) { 
      std::cout << "kp_roll not found, defaulting to " << kp_roll << std::endl;
  }
  if (!ros::param::get("~/kd_roll", kd_roll)) { 
      std::cout << "kd_roll not found, defaulting to " << kd_roll << std::endl;
  }

  if (!ros::param::get("~/kp_pitch", kp_pitch)) { 
      std::cout << "kp_pitch not found, defaulting to " << kp_pitch << std::endl;
  }
  if (!ros::param::get("~/kd_pitch", kd_pitch)) { 
      std::cout << "kd_pitch not found, defaulting to " << kd_pitch << std::endl;
  }

  if (!ros::param::get("~/kp_yaw", kp_yaw)) { 
    std::cout << "kp_yaw not found, defaulting to " << kp_yaw << std::endl;
  }
  if (!ros::param::get("~/kd_yaw", kd_yaw)) { 
    std::cout << "kd_yaw not found, defaulting to " << kd_yaw << std::endl;
  }

  if (!ros::param::get("~/kp_yawRate", kp_yawRate)) { 
      std::cout << "kp_yawRate not found, defaulting to " << kp_yawRate << std::endl;
  }
  if (!ros::param::get("~/kp_hVel", kp_hVel)) { 
    std::cout << "kp_hVel not found, defaulting to " << kp_hVel << std::endl;
  }

}

attitudeNode::~attitudeNode()
{

}


double acc_x = 0;
double acc_y = 0;
double acc_z = 0;

double p = 0;
double q = 0;
double r = 0;


/* imu callback for model prediction */
void attitudeNode::imu_cb(const sensor_msgs::Imu &imuMsg)
{
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
	thr_cmd     = cmd.thrust.z;
  printf("in optimal_cb, roll_cmd: %f\n", roll_cmd * 180/3.142);
}


void attitudeNode::gtCallback(const tf2_msgs::TFMessage &groundTruth_msg) 
{
  x_est = groundTruth_msg.transforms[0].transform.translation.x;
  y_est = groundTruth_msg.transforms[0].transform.translation.y;
  z_est = groundTruth_msg.transforms[0].transform.translation.z;

  // char act;
  // cin >> act; act == 'g'
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

  qx = groundTruth_msg.transforms[0].transform.rotation.x;
  qy = groundTruth_msg.transforms[0].transform.rotation.y;
  qz = groundTruth_msg.transforms[0].transform.rotation.z;
  qw = groundTruth_msg.transforms[0].transform.rotation.w;

  roll_est  = (atan2(2*qx*qw + 2*qy*qz, 1 - 2*qx*qx - 2*qy*qy));
  pitch_est = (asin(2*qw*qy - 2*qz*qx));
  yaw_est   = (atan2(2*qy*qx + 2*qw*qz, 1 - 2*qy*qy - 2*qz*qz));

  d_roll_cmd  = kp_roll  * (roll_cmd - roll_est)   - kd_roll  * p;
  d_pitch_cmd = kp_pitch * (pitch_cmd - pitch_est) - kd_pitch * q;
  d_yaw_cmd   = yawRate_cmd * kp_yawRate;

  angular_rate_cmd.thrust.z = thr_cmd; // Gate3 is at 4m
  angular_rate_cmd.angular_rates.x = d_roll_cmd - sin(pitch_est) * d_yaw_cmd;
  angular_rate_cmd.angular_rates.y = cos(roll_est) * d_pitch_cmd + cos(pitch_est) * sin(roll_est) * d_yaw_cmd;
  angular_rate_cmd.angular_rates.z = - sin(roll_est) * d_pitch_cmd + cos(pitch_est) * cos(roll_est) * d_yaw_cmd;

  angular_rate_cmd_pub.publish(angular_rate_cmd);

  timeStamp_old = groundTruth_msg.transforms[0].header.stamp.toSec();
  x_est_old = x_est;
  y_est_old = y_est;
  z_est_old = z_est;

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "controllerMavlab_node");

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


  // // unsure if we can use the gyro here.
  // diff_roll  = curr_error_roll  - old_error_roll;
  // diff_pitch = curr_error_pitch - old_error_pitch;

  // d_roll_cmd  = kp_roll  * curr_error_roll  + diff_roll  * rate_hz * kd_roll;
  // d_pitch_cmd = kp_pitch * curr_error_pitch + diff_pitch * rate_hz * kd_pitch;