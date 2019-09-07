#include "pidcontrol.h"

// keyboard inputs
double roll_cmd = 0;
double pitch_cmd = 0;
double yawRate_cmd = 0;

double vel_x_cmd_velframe = 0;
double vel_y_cmd_velframe = 0;
double xVel_est_velframe = 0;
double yVel_est_velframe = 0;

double x_est = 18;
double y_est = -23;
double z_est = 0;

double xVel_est = 0;
double yVel_est = 0;
double zVel_est = 0;
bool firstMsg = 1;
double dt_est = 0.001;

int lock_pid = 0;

ros::Publisher pidcmd_pub;

void ratethrust_cb(const mav_msgs::RateThrust &command)
{
	if (!lock_pid) {
		mav_msgs::RateThrust pid_cmd;
		pid_cmd.angular_rates.x = command.angular_rates.x * 35 * 3.142/180;
		pid_cmd.angular_rates.y = command.angular_rates.y * 35 * 3.142/180;
		pid_cmd.angular_rates.z = command.angular_rates.z;
		pid_cmd.thrust.z = command.thrust.z;
		pidcmd_pub.publish(pid_cmd);
	}
}


double roll_est = 0;
double pitch_est = 0;
double yaw_est = 0;
void gtCallback(const tf2_msgs::TFMessage &groundTruth_msg) 
{
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
	timeStamp_old = groundTruth_msg.transforms[0].header.stamp.toSec();
	x_est_old = x_est;
	y_est_old = y_est;
	z_est_old = z_est;

	double qx = groundTruth_msg.transforms[0].transform.rotation.x;
	double qy = groundTruth_msg.transforms[0].transform.rotation.y;
	double qz = groundTruth_msg.transforms[0].transform.rotation.z;
	double qw = groundTruth_msg.transforms[0].transform.rotation.w;

	roll_est  = (atan2(2*qx*qw + 2*qy*qz, 1 - 2*qx*qx - 2*qy*qy));
	pitch_est = (asin(2*qw*qy - 2*qz*qx));
	yaw_est   = (atan2(2*qy*qx + 2*qw*qz, 1 - 2*qy*qy - 2*qz*qz));
}

ros::Publisher pub_resetdrone;
void resetdrone_cb(std_msgs::Empty::Ptr msg) {
	pub_resetdrone.publish(std_msgs::Empty());
}


void pidJoystick_cb(std_msgs::Empty::Ptr msg) {
    printf("latching PID controller!\n");
  lock_pid = 1;
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "pidnode");
    ros::NodeHandle nh;
    ros::Subscriber	gt_sub = nh.subscribe("/tf", 1000, gtCallback); 
  	ros::Subscriber ratethrust_sub = nh.subscribe("/controller/input/keyboard", 1000, ratethrust_cb);
    ros::Subscriber pidJoystick_sub = nh.subscribe("/control_nodes/triggeroptimal", 1, pidJoystick_cb);
    ros::Subscriber resetdrone_sub = nh.subscribe("/control_nodes/resetdrone", 1, resetdrone_cb);
    // publish to control loop commands in controllermavlab
  	pidcmd_pub = nh.advertise<mav_msgs::RateThrust>("optimalcmd", 1);
	pub_resetdrone = nh.advertise<std_msgs::Empty>("/uav/collision", 1);

    // 960 Hz control loop
    ros::Rate rate(960);

    while(ros::ok()) {
        if (lock_pid) {
            /** your faltu logic *************************/    
            static double old_error_pos_x_velFrame = 0;
            static double old_error_pos_y_velFrame = 0;

            double curr_error_pos_w_x = -0.37  - x_est;
            double curr_error_pos_w_y = -12.23 - y_est;

            double curr_error_pos_x_velframe =  cos(yaw_est)*curr_error_pos_w_x + sin(yaw_est)*curr_error_pos_w_y;
            double curr_error_pos_y_velframe = -sin(yaw_est)*curr_error_pos_w_x + cos(yaw_est)*curr_error_pos_w_y;

            double diff_pos_x_velframe = curr_error_pos_x_velframe - old_error_pos_x_velFrame;
            double diff_pos_y_velframe = curr_error_pos_y_velframe - old_error_pos_y_velFrame;

            vel_x_cmd_velframe = curr_error_pos_x_velframe * KP_POS_X - diff_pos_x_velframe * KD_POS_X;
            vel_y_cmd_velframe = curr_error_pos_y_velframe * KP_POS_Y - diff_pos_y_velframe * KD_POS_Y;

            vel_x_cmd_velframe = bound_f(vel_x_cmd_velframe, -MAX_VEL_X, MAX_VEL_X);
            vel_y_cmd_velframe = bound_f(vel_y_cmd_velframe, -MAX_VEL_Y, MAX_VEL_Y);

            // vel_x_cmd_velframe += 5; //pitch more for gate vel

            xVel_est_velframe =  cos(yaw_est) * xVel_est + sin(yaw_est) * yVel_est;
            yVel_est_velframe = -sin(yaw_est) * xVel_est + cos(yaw_est) * yVel_est;

            double curr_error_vel_x = (vel_x_cmd_velframe - xVel_est_velframe);
            double curr_error_vel_y = (vel_y_cmd_velframe - yVel_est_velframe);

            static double prev_error_vel_x = 0;
            static double prev_error_vel_y = 0;

            double diff_error_vel_x = curr_error_vel_x - prev_error_vel_x;
            double diff_error_vel_y = curr_error_vel_y - prev_error_vel_y;

            pitch_cmd =   curr_error_vel_x * KP_VEL_X - diff_error_vel_x * KD_VEL_X + K_FF_THETA * vel_x_cmd_velframe;
            roll_cmd  = -(curr_error_vel_y * KP_VEL_Y - diff_error_vel_y * KD_VEL_Y + K_FF_PHI   * vel_y_cmd_velframe);

            pitch_cmd = bound_f(pitch_cmd, -MAX_BANK, MAX_BANK);
            roll_cmd  = bound_f(roll_cmd,  -MAX_BANK, MAX_BANK);

            // 35 cms close to target, terminate
            if ((fabs(curr_error_pos_w_x) < 0.35) && (fabs(curr_error_pos_w_y) < 0.35)) {
                pitch_cmd = 0;
                roll_cmd = 0;
                lock_pid = 0;
                printf("finished pid maneuver!!\n\n");
            }
            mav_msgs::RateThrust pid_cmd;
            pid_cmd.angular_rates.x = roll_cmd;
			pid_cmd.angular_rates.y = pitch_cmd;
			pid_cmd.angular_rates.z = 0 - yaw_est;
			pid_cmd.thrust.z = 9.81;
            pidcmd_pub.publish(pid_cmd);

            old_error_pos_x_velFrame = curr_error_pos_x_velframe;
            old_error_pos_y_velFrame = curr_error_pos_y_velframe;

            prev_error_vel_x = curr_error_vel_x;
            prev_error_vel_y = curr_error_vel_y;
        }
        ros::spinOnce();
        rate.sleep();
    }
}