#include "traj.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <math.h>
#include <iostream>

using namespace std;
using namespace Eigen;

#define MAX_N 100

// for gtcallback
double x_est = -24.0;
double y_est = -2.0;
double z_est = 4;

double xVel_est = 0;
double yVel_est = 0;
double zVel_est = 0;
double timeStamp_old = 0;
double x_est_old = 0;
double y_est_old = 0;
double z_est_old = 0;
double dt_est = 0.001;
bool firstMsg = 1;


void gtCallback(const tf2_msgs::TFMessage &groundTruth_msg) 
{
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
}

int N = 30000;
ros::Publisher optimalcmd_pub;
bool lock_optimal = 0;
void keyboard_cb(const mav_msgs::RateThrust &command)
{
	mav_msgs::RateThrust opt_cmd;
	if (command.angular_rates.y >= 0 && !lock_optimal) {
		opt_cmd.angular_rates.x = command.angular_rates.x * 20 * 3.142/180;
		opt_cmd.angular_rates.y = command.angular_rates.y * 20 * 3.142/180;
		opt_cmd.angular_rates.z = command.angular_rates.z;
		opt_cmd.thrust.z = command.thrust.z;
		optimalcmd_pub.publish(opt_cmd);
		printf("manual mode \n");
	}

	if (command.angular_rates.y < 0) {
		lock_optimal = 1;
		printf("starting optimal control calc\n");
		Eigen::MatrixXd theta;
		Eigen::MatrixXd phi2;
		Eigen::MatrixXd fz1;
		double pos[3] = {x_est, y_est, z_est};
		double vel[3] = {xVel_est, yVel_est, zVel_est};
		double psi0 = 0.0;
    	traj_calc(pos, vel, psi0, theta, phi2, fz1);
		N = theta.cols();
		printf("size of N: %d \n", N);
	}

}


/** Example for qpOASES main function using the QProblemB class. */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "traj_publish_node");

	ros::NodeHandle nh;

	// publish to control loop commands in controllermavlab
  	optimalcmd_pub = nh.advertise<mav_msgs::RateThrust>("optimalcmd", 20);

	ros::Subscriber keyboard_sub;
	ros::Subscriber gt_sub;

	gt_sub = nh.subscribe("/tf", 1000, gtCallback); 
  	keyboard_sub = nh.subscribe("/controller/input/keyboard", 1000, keyboard_cb);
	
	// initial hotstart
	Eigen::MatrixXd theta;
	Eigen::MatrixXd phi2;
	Eigen::MatrixXd fz1;
	double psi0 = 0;
	double pos[3] = {0};
	double vel[3] = {0};
 	traj_calc(pos, vel, psi0, theta, phi2, fz1);
	cout << " Hotstart length: " << theta.cols() << endl;


	ros::Rate loop_rate(960);
	int i = 0;
	while(ros::ok()) {
		mav_msgs::RateThrust opt_cmd;
		if(lock_optimal && i < N) {
			opt_cmd.angular_rates.x = theta(i);
			opt_cmd.angular_rates.y = phi2(i);
			opt_cmd.angular_rates.z = 0;
			//opt_cmd.thrust.z = fz1(i);
			opt_cmd.thrust.z = 9.81 / ((cos(phi2(i))) * (cos(theta(i))));//sqrt(pow(fz1(0,i),2) + pow(fz1(1,i),2) + pow(fz1(2,i),2));  //  
			optimalcmd_pub.publish(opt_cmd);
			// cout << "roll: " << phi2(i) * 180/3.142 << ", pitch: " << theta(i) * 180/3.142 << endl;
			// cout << fz1(i) << endl;
			printf("i:%d, N:%d\n", i, N);
			
			i++; 
		}
		if (i > N-1) {
			opt_cmd.angular_rates.x = 0;
			opt_cmd.angular_rates.y = 0;
			opt_cmd.angular_rates.z = 0;
			opt_cmd.thrust.z = 9.81;
			optimalcmd_pub.publish(opt_cmd);
			cout << "Finised sequence\n";
			i = 0;
			lock_optimal = 0;
		}

		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}