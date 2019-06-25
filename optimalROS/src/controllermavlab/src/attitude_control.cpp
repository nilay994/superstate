
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


// PID gains
/*
double kp_roll = 5.0;
double kd_roll = 0.9;

double kp_pitch = 6.5;
double kd_pitch = 0.95;
*/
double kp_roll = 100.0;
double kd_roll = 10;

double kp_pitch = 100.0;
double kd_pitch = 10;

double kp_yaw = 2.2;
double kd_yaw = 0.03;

double kp_yawRate = 1.5;  
double kp_hVel = 10.0;

double kp_vel_x = 0.25;
double kd_vel_x = 0.0;

double kp_vel_y = 0.15;
double kd_vel_y = 0.0;

double kp_pos_x = 1.35;
double kd_pos_x = 0.000075;

double kp_pos_y = 1.35;
double kd_pos_y = 0.000075;

double kp_pos_z = 4.0;
double kd_pos_z = 0.0;

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

double zVel_cmd = 0;


double curr_error_roll = 0;
double curr_error_pitch = 0;

double diff_roll = 0;
double old_error_roll = 0;
double diff_pitch = 0;
double old_error_pitch = 0;

FILE *fg_log;
FILE *imu_f;

// constructor initializes subscribers and publishers
attitudeNode::attitudeNode(ros::NodeHandle nh)
{
  //rollPitchYawRateHeightRateCommand
  keyboard_sub = nh.subscribe("/controller/input/keyboard", 1000, &attitudeNode::keyboard_cb, this);

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
  
  if (!ros::param::get("~/kp_vel_x", kp_vel_x)) { 
    std::cout << "kp_vel_x not found, defaulting to " << kp_vel_x << std::endl;
  }
  if (!ros::param::get("~/kd_vel_x", kd_vel_x)) { 
    std::cout << "kd_vel_x not found, defaulting to " << kd_vel_x << std::endl;
  }

  if (!ros::param::get("~/kp_vel_y", kp_vel_y)) { 
    std::cout << "kp_vel_y not found, defaulting to " << kp_vel_y << std::endl;
  }
  if (!ros::param::get("~/kd_vel_y", kd_vel_y)) { 
    std::cout << "kd_vel_y not found, defaulting to " << kd_vel_y << std::endl;
  }

  if (!ros::param::get("~/kp_pos_x", kp_pos_x)) { 
    std::cout << "kp_pos_x not found, defaulting to " << kp_pos_x << std::endl;
  }
  if (!ros::param::get("~/kd_pos_x", kd_pos_x)) { 
    std::cout << "kd_pos_x not found, defaulting to " << kd_pos_x << std::endl;
  }

  if (!ros::param::get("~/kp_pos_y", kp_pos_y)) { 
    std::cout << "kp_pos_y not found, defaulting to " << kp_pos_y << std::endl;
  }
  if (!ros::param::get("~/kd_pos_y", kd_pos_y)) { 
    std::cout << "kd_pos_y not found, defaulting to " << kd_pos_y << std::endl;
  }

  if (!ros::param::get("~/kp_pos_z", kp_pos_z)) { 
    std::cout << "kp_pos_z not found, defaulting to " << kp_pos_z << std::endl;
  }
  if (!ros::param::get("~/kd_pos_z", kd_pos_z)) { 
    std::cout << "kd_pos_z not found, defaulting to " << kd_pos_z << std::endl;
  }

  fg_log = fopen("tf.csv","w+");
  imu_f = fopen("imu.csv","w+");

}

attitudeNode::~attitudeNode()
{
  // cout << "Crashed at: " << clockTakeoff_msg.clock.toSec() 
	fclose(fg_log);
	fclose(imu_f);

}

  float phi0 = 0;
  float phi1 = 0;
  float invert_time = 0;



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

double presstime = 0;
int pressed = 0;
void attitudeNode::keyboard_cb(const mav_msgs::RateThrust &command)
{
	// vel_x_cmd   = command.angular_rates.y * 20;  
	// vel_y_cmd   = - command.angular_rates.x * 20;  
	/*position_x_cmd = command.angular_rates.y;
	position_y_cmd = command.angular_rates.x;
	yaw_cmd = command.angular_rates.z;  //   yawRate_cmd
	// heightVel_cmd  = command.thrust.z;*/

  roll_cmd  = command.angular_rates.x * 20 * 3.142 / 180;
	pitch_cmd = command.angular_rates.y * 20 * 3.142/ 180;  
	yawRate_cmd = command.angular_rates.z;
	zVel_cmd = (command.thrust.z);
}


void attitudeNode::gtCallback(const tf2_msgs::TFMessage &groundTruth_msg) 
{

  if (command.angular_rates.x > 0) {
    presstime = command.header.stamp.toSec();
    pressed = 1;
    printf("maneuver activated\n");
  }

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
  
  qx = groundTruth_msg.transforms[0].transform.rotation.x;
  qy = groundTruth_msg.transforms[0].transform.rotation.y;
  qz = groundTruth_msg.transforms[0].transform.rotation.z;
  qw = groundTruth_msg.transforms[0].transform.rotation.w;
  // tf::Quaternion q(groundTruth_msg.transform.rotation.x, groundTruth_msg.transform.rotation.y, groundTruth_msg.transform.rotation.z, groundTruth_msg.transform.rotation.w);
  // tf::Matrix3x3 R(q);
  // R.getEuler
  // from quaterion to Euler in degree
  roll_est  = (atan2(2*qx*qw + 2*qy*qz, 1 - 2*qx*qx - 2*qy*qy));
  pitch_est = (asin(2*qw*qy - 2*qz*qx));
  yaw_est   = (atan2(2*qy*qx + 2*qw*qz, 1 - 2*qy*qy - 2*qz*qz));

  if (pressed) {
    if (groundTruth_msg.transforms[0].header.stamp.toSec() < invert_time + presstime) {
      roll_cmd  = phi0;
      pitch_cmd = 20 * 3.142/ 180;
      static bool first = 1;
      if (first) {
        printf("roll_cmd: %f\n", phi0*180/3.142);
        printf("pitch_cmd: %f\n", pitch_cmd*180/3.142);
        first = 0;
      }

    }
    if (groundTruth_msg.transforms[0].header.stamp.toSec() > invert_time + presstime) {
      roll_cmd  = phi1; 
      pitch_cmd = 20 * 3.142/ 180;
      static bool first = 1;
      if (first) {
        printf("roll_cmd: %f\n", phi1*180/3.142);
        printf("pitch_cmd: %f\n", pitch_cmd*180/3.142);
        first = 0;
      }

    }
    if (groundTruth_msg.transforms[0].header.stamp.toSec() > (invert_time/0.55) + presstime) {
      static bool first = 1;
      if (first) {
        printf("maneuver stop:\n");
        first = 0;
      }
      roll_cmd  = 0; 
      pitch_cmd = 0;
      pressed   = 0;
      
    }
  }


	d_roll_cmd = kp_roll  * (roll_cmd - roll_est)  - kd_roll  * p; //PID_attitude_controller.roll_cmd * 0.3
	d_pitch_cmd = kp_pitch * (pitch_cmd - pitch_est) - kd_pitch * q; //PID_attitude_controller.pitch_cmd * 0.3
	d_yaw_cmd = yawRate_cmd * kp_yawRate;

  

	angular_rate_cmd.thrust.z = 9.81 / (cos(roll_est) * cos(pitch_est)) + (zVel_cmd - 9.81);
	// ROS_INFO_STREAM("thrust f " << angular_rate_cmd.thrust.z);
	// angular_rate_cmd.angular_rates.x = d_roll_cmd - sin(pitch_est) * d_yaw_cmd;
	// angular_rate_cmd.angular_rates.y = cos(roll_est) * d_pitch_cmd + cos(pitch_est) * sin(roll_est) * d_yaw_cmd;
	// angular_rate_cmd.angular_rates.z = - sin(roll_est) * d_pitch_cmd + cos(pitch_est) * cos(roll_est) * d_yaw_cmd;
	angular_rate_cmd.angular_rates.x = d_roll_cmd;
	angular_rate_cmd.angular_rates.y = d_pitch_cmd;
	angular_rate_cmd.angular_rates.z = d_yaw_cmd;

	angular_rate_cmd_pub.publish(angular_rate_cmd);

	fprintf(fg_log, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
	  			groundTruth_msg.transforms[0].header.stamp.toSec(), 
          x_est, y_est, z_est,
          roll_est, pitch_est, yaw_est,
          acc_x, acc_y, acc_z);

	fprintf(imu_f, "%f,%f,%f,%f\n", groundTruth_msg.transforms[0].header.stamp.toSec(),
	  			acc_x, acc_y, acc_z);
  
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

  find_optimal(&phi0, &phi1, &invert_time);
  printf("\n\nphi0: %f, phi1: %f, dt: %f\n", phi0, phi1, invert_time);

  //timeInitial = ros::Time::now().toSec();

  while (ros::ok()) {

    ros::spinOnce();
    rate.sleep();
  
  }

  return 0;
}

  float x0[2] = {-24.0, -7.0}; //GPS_take;
  float v0[2] = {5.0, 0.0}; // needs non zero initial velocity, not sure why

  float xd[2] = {-0.37, -12.23};
  float vd[2] = {5.0, 0};

#define OPT_ITER 1000
void find_optimal(float *phi0, float *phi1, float *switch_time)
{


  // flip the drone at 55% of total time
  float t1 = (xd[0]-x0[0]) / v0[0] * 0.55;
  *switch_time = t1;

  // set initial step size as 4 degrees
  float dphi0 = 4.0/57; 
  float dphi1 = 4.0/57;

  float xt[2]; 
  float vt[2];

  for (int i = 0; i < OPT_ITER; i++) {
    float c0;
    c0  = pathPredict(x0, v0, xd, vd, *phi0, *phi1, t1, xt, vt);
    // plus step size
    float c1pstep = (*phi0 + dphi0);
    float c1p = pathPredict(x0, v0, xd, vd, c1pstep, *phi1, t1, xt, vt);
    float c2pstep = (*phi1 + dphi1);
    float c2p = pathPredict(x0, v0, xd, vd, *phi0, c2pstep, t1, xt, vt);
    
    // printf("iter: %d, c1step: %f,c1p: %f, c2pstep: %f, c2p: %f\n", i, c1pstep, c1p, c2pstep, c2p);
    
    if (c0 < c1p) {
      // calc minus step size
      float c1mstep = (*phi0 - dphi0);
      float c1m = pathPredict(x0, v0, xd, vd, c1mstep, *phi1, t1, xt, vt);

      if (c1m < c0) {
        // reverse sign
        dphi0 = -dphi0;
      }
      else {
        // no need of reversing gradient, just reduce it
        dphi0 = dphi0 / 2;
      }

    }
    if (c0 < c2p) {
      // calc minus step size
      float c2mstep = (*phi1 - dphi1);
      float c2m = pathPredict(x0, v0, xd, vd, *phi0, c2mstep, t1, xt, vt);
      
      if (c2m < c0) {
        // reverse sign
        dphi1 = -dphi1;
      }
      else {
        // no need of reversing gradient, just reduce it
        dphi1 = dphi1 / 2;
      }
    
    }
    if (c1p < c2p) {
      // see if c1p can become better
      *phi0 = *phi0 + dphi0;
    }
    else {
      *phi1 = *phi1 + dphi1;
    }
  
  }

}

float pathPredict(float x0[2], float v0[2],
                  float xd[2], float vd[2], 
                  float phi0, float phi1, float t1, 
                  float *xt, float *vt)
{
  float x[2];
  x[0] = x0[0];
  x[1] = x0[1];
  
  float v[2]; 
  v[0] = v0[0];
  v[1] = v0[1];

  float deltat = 0.01;
  float simtime = 0;
  float phi;
  // Predict until passing the gate
  while (x[0] < xd[0]) {
    
    if (simtime < t1) {
      phi = phi0;
    }
    if (simtime >= t1) {
      phi = phi1;
    }

    // saturate @ 57
    if (phi > 1) {
      phi = 1;
    }
    if (phi < -1) {
      phi = -1;
    }

    float ay = - (sinf(phi) * 9.81 / cosf(phi*0.85)) - v[1] * 0.56;
    
    // Simulation
    simtime = simtime + deltat;
    v[0] = v[0] + 0 * deltat;
    x[0] = x[0] + v[0] * deltat; 

    v[1] = v[1] + ay * deltat;
    x[1] = x[1] + v[1] * deltat; 
  }
  // TODO: does it also care about x positions? this is only y
  // Position at the Gate
  xt = x;
  vt = v;
  // penalize x more than y
  float cost = fabsf(x[1] - xd[1])*fabsf(x[1] - xd[1]) * 10 + fabsf(v[1]- vd[1])*fabsf(v[1] - vd[1]);

  return cost; 
}