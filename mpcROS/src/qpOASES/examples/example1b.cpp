/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file examples/example1b.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2007-2017
 *
 *	Very simple example for testing qpOASES using the QProblemB class.
 */


#include <qpOASES.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <math.h>
#include <iostream>

#include <ros/ros.h>
#include <mav_msgs/RateThrust.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


using namespace std;
using namespace Eigen;

#define MAX_N 1600

// for gtcallback
double x_est = -24.0;
double y_est = -2.0;
double z_est = 4;

double xVel_est = 0;
double yVel_est = 0;
double zVel_est = 0;

double dt_est = 0.001;
bool firstMsg = 1;

/*double pitch_cmd_pid = 0;
double roll_cmd_pid  = 0;
double vel_x_pid = 0;
double vel_y_pid = 0;

double pitch_cmd_opt = 0;
double roll_cmd_opt  = 0;
double vel_x_opt = 0;
double vel_y_opt = 0;*/

USING_NAMESPACE_QPOASES

float phi_cmd[MAX_N];
float theta_cmd[MAX_N];

unsigned int N;
void optimal_calc()
{
	Eigen::Matrix<double, 4, 4> A;
	A << 0.9512, 0, 0, 0,
		0.09754, 1, 0, 0,
		0, 0, 0.9512, 0,
		0, 0, 0.09754, 1;

	Eigen::Matrix<double, 4, 2> B;
	B << 0.9569, 0,
		0.04824, 0,
		0, 0.9569,
		0, 0.04824;

	Eigen::Matrix<double, 4, 4> P;
	P << 1,0,0,0,
		0,5,0,0,
		0,0,1,0,
		0,0,0,5;

	// position final is one of the gates in the arena
	double pos0[2] = {x_est, y_est};
	double posf[2] = {-0.37, -12.23};

	// go through the gate with 3.5m/s forward vel
	double vel0[2] = {xVel_est, yVel_est};
	double velf[2] = {3.5, 0};

	// above state space matrices are discretized at 100 milliseconds/10 Hz
	float dt = 0.1;

	// break optimizer if more than 30% of banging
	float bangedtheta = 0; float bangedphi = 0;
	float iterate = 1;
	while(bangedphi < 30 && bangedtheta < 30) {
		// float T = sqrt(pow((pos0[0] - posf[0]),2) + pow((pos0[1] - posf[1]),2)) / iterate;
		
		// assumption: can reach anywhere in the arena if I have ten seconds 
		float T = 10 / iterate; 
		N = round(T/dt);
		printf("Horizon: %d\n", N);  
		
		// Eigen::Matrix<double, 4, Dynamic> R;
		Eigen::MatrixXd oldR(4, 2 * MAX_N);
		oldR.resize(4, 2*N);
		
		oldR.block(0, 2*N-2, 4, 2) =  B;
		Eigen::Matrix<double, 4, 4> AN = A;

		for(int i=1; i<N; i++) {
			oldR.block(0, 2*N-2*(i+1), 4, 2) =  A * oldR.block(0, 2*N-2*i, 4, 2);
			AN = A * AN; 
		}

		Eigen::MatrixXd R = oldR.block(0,0,4,2*N);
		Eigen::MatrixXd old_H = 2 * (R.transpose() * P * R);
		Eigen::MatrixXd eye(2* MAX_N, 2 * MAX_N); 
		eye.resize(2*N, 2*N);
		eye.setIdentity();
		Eigen::MatrixXd H = 0.5 * (old_H + old_H.transpose() + eye);

		Eigen::Matrix<double, 4, 1> x0; Eigen::Matrix<double, 4, 1> xd;
		x0 << vel0[0], pos0[0], vel0[1], pos0[1];
		xd << velf[0], posf[0], velf[1], posf[1];
		
		Eigen::MatrixXd f;
		f = (2 * ((AN * x0)- xd)).transpose() * P * R;

		// saturate at not more than 25 degrees of banking in roll or pitch
		float maxbank = 25.0 * 3.142 / 180.0;
		int sizes = 2 * N;

		real_t ub[sizes]; real_t lb[sizes];
		
		for (int i=0; i<sizes; i++) {
			ub[i] = maxbank;
			lb[i] = -maxbank;
		}
		// populate hessian and linear term
		real_t newH[sizes * sizes];	real_t newf[sizes];
		Eigen::Map<MatrixXd>(newH, sizes, sizes) = H.transpose();
		Eigen::Map<MatrixXd>(newf, 1, sizes) = f;

		// our class of problem, we don't have any constraints on position or velocity, just the inputs
		/* Setting up QProblemB object. */
		QProblemB mpctry(2*N);  

		// constructor can be initialized by Hessian type, so it can stop checking for positive definiteness
		Options optionsmpc;
		//options.enableFlippingBounds = BT_FALSE;
		optionsmpc.printLevel = PL_HIGH;  // DEBUG level high, but the prints are muted in the library :(
		optionsmpc.initialStatusBounds = ST_INACTIVE;
		optionsmpc.numRefinementSteps = 1;
		// optionsmpc.enableCholeskyRefactorisation = 1;
		mpctry.setOptions(optionsmpc);

		/* not sure: take 100 tries to solve the QP? */
		int nWSR = 100;
		
		/* Initialize QP. */
		real_t xOpt[sizes];
		if (mpctry.init(newH,newf, lb,ub, nWSR, 0) == SUCCESSFUL_RETURN) {
			float bangedtheta_acc = 0;
			float bangedphi_acc = 0;
			/* solve the QP */
			if (mpctry.getPrimalSolution(xOpt) == SUCCESSFUL_RETURN) {
				/* populate the command buffers */
				for (int i=0; i<N; i++) {
					theta_cmd[i] = (float) xOpt[2*i];
					phi_cmd[i]   = (float) -1 * xOpt[2*i + 1];
					if ((fabs(fabs(theta_cmd[i])-maxbank) < 0.01) || (fabs(fabs(phi_cmd[i])-maxbank) < 0.01)) {
						bangedtheta_acc += fabs(theta_cmd[i]);
						bangedphi_acc   += fabs(phi_cmd[i]);
					}
				}
				bangedtheta = bangedtheta_acc / (N * maxbank) * 100;
				bangedphi   = bangedphi_acc   / (N * maxbank) * 100;
				printf("fval = %e, bangedtheta: %f, bangedphi: %f\n", mpctry.getObjVal(), bangedtheta, bangedphi);
			}
			else {
				printf("QP couldn't be solved! \n");
			}
		}
		else {
			printf("QP couldn't be initialized! \n");
		}
		iterate = iterate * 1.2;
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

bool lock_optimal = 0;
double start = 0;
void optimalJoystick_cb(std_msgs::Empty::Ptr msg) {
	printf("starting optimal control calc\n");
	optimal_calc();
	lock_optimal = 1;
	start = ros::Time::now().toSec();
}

ros::Publisher optimalcmd_pub;
ros::Publisher pub_resetdrone;
void ratethrust_cb(const mav_msgs::RateThrust &command)
{
	if (!lock_optimal) {
		mav_msgs::RateThrust opt_cmd;
		opt_cmd.angular_rates.x = command.angular_rates.x * 35 * 3.142/180;
		opt_cmd.angular_rates.y = command.angular_rates.y * 35 * 3.142/180;
		opt_cmd.angular_rates.z = command.angular_rates.z;
		opt_cmd.thrust.z = command.thrust.z;
		optimalcmd_pub.publish(opt_cmd);
	}
}

void resetdrone_cb(std_msgs::Empty::Ptr msg) {

	pub_resetdrone.publish(std_msgs::Empty());
}

/** Example for qpOASES main function using the QProblemB class. */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "qpoases_node");

	ros::NodeHandle nh;

	// publish to control loop commands in controllermavlab
  	optimalcmd_pub = nh.advertise<mav_msgs::RateThrust>("optimalcmd", 1);
	pub_resetdrone = nh.advertise<std_msgs::Empty>("/uav/collision", 1);

	ros::Subscriber ratethrust_sub;
	ros::Subscriber gt_sub;
	ros::Subscriber joystick_sub;
	ros::Subscriber resetdrone_sub;

	gt_sub = nh.subscribe("/tf", 1000, gtCallback); 
  	ratethrust_sub = nh.subscribe("/controller/input/keyboard", 1000, ratethrust_cb);
	joystick_sub   = nh.subscribe("/control_nodes/triggeroptimal", 1, optimalJoystick_cb);
	resetdrone_sub = nh.subscribe("/control_nodes/resetdrone", 1, resetdrone_cb);

	// optimal_calc();
	/*
	This function returns one of the following values:
	• 0: QP was solved,
	• 1: QP could not be solved within the given number of iterations,
	• -1: QP could not be solved due to an internal error,
	• -2: QP is infeasible and thus could not be solved,
	• -3: QP is unbounded and thus could not be solved.
	*/
	// int getSimpleStatus();

	ros::Rate loop_rate(10);
	int i = 0;
	double newAng_theta = 0;
	double newAng_phi = 0;
	
	while(ros::ok()) {
		mav_msgs::RateThrust opt_cmd;
		if(lock_optimal && i < N) {
			
			newAng_theta = cos(yaw_est) * theta_cmd[i] - sin(yaw_est) * phi_cmd[i];
			newAng_phi = sin(yaw_est) * theta_cmd[i] + cos(yaw_est) * phi_cmd[i];
			
			opt_cmd.angular_rates.x = newAng_phi;
			opt_cmd.angular_rates.y = newAng_theta;
			opt_cmd.angular_rates.z = 0;
			opt_cmd.thrust.z = 9.81;
			optimalcmd_pub.publish(opt_cmd);
			cout << "roll: " << phi_cmd[i] * 180/3.142 << ", pitch: " << theta_cmd[i] * 180/3.142 << endl;
			i++;
		}
		if (i > N-1) {
			opt_cmd.angular_rates.x = 0;
			opt_cmd.angular_rates.y = 0;
			opt_cmd.angular_rates.z = 0;
			opt_cmd.thrust.z = 9.81;
			optimalcmd_pub.publish(opt_cmd);
			double completion_time = ros::Time::now().toSec() - start;
			printf("Finised sequence in %f seconds \n", completion_time);
			i = 0;
			lock_optimal = 0;
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}


/*
 *	end of file
 */
