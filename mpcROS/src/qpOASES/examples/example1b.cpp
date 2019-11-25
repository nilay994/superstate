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

#define MAX_N 160

#define KP_POS_X 2
#define KP_POS_Y 2

#define KP_VEL_X 0.02
#define KP_VEL_Y 0.02

#define MAX_VEL_X 6
#define MAX_VEL_Y 6

// for gtcallback
int i = 0;
double x_est = -24.0;
double y_est = -2.0;
double z_est = 4;

double xVel_est = 0;
double yVel_est = 0;
double zVel_est = 0;

double dt_est = 0.001;
bool firstMsg = 1;
int invoke_cnt = 1;

FILE *plotopt_f;
FILE *states_f;
USING_NAMESPACE_QPOASES

float phi_ff[MAX_N];
float theta_ff[MAX_N];

unsigned int N;
bool lock_optimal = 0;

// saturate at not more than 25 degrees of banking in roll or pitch
float maxbank = 25.0 * 3.142 / 180.0;

// constructor can be initialized by Hessian type, so it can stop checking for positive definiteness
Options optionsmpc;

Eigen::MatrixXd oldR(4, 2 * MAX_N);
Eigen::MatrixXd states(4, MAX_N);
Eigen::MatrixXd eye(2* MAX_N, 2 * MAX_N); 
Eigen::MatrixXd old_H(2 * MAX_N, 2 * MAX_N);
Eigen::MatrixXd R(4, 2 * MAX_N);
Eigen::MatrixXd f(1, 2 * MAX_N);


// bound a value to a range [min,max]
double bound_f(double val, double min, double max) {
	if (val > max) {
		val = max;
	} else if (val < min) {
		val = min;
	}
	return val;
}



void optimal_calc() {
	invoke_cnt = invoke_cnt + 1;
	Eigen::Matrix<double, 4, 4> A;

	A << 0.9448, 0, 0, 0,
		0.09721, 1, 0, 0,
		0, 0, 0.9458, 0,
		0, 0, 0.09726, 1;

	Eigen::Matrix<double, 4, 2> B;
	B << 0.9537, 0,
		0.04813, 0,
		0, 0.9542,
		0, 0.04815;

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
	double velf[2] = {5.0, 0};

	// above state space matrices are discretized at 100 milliseconds/10 Hz
	float dt = 0.1;

	// break optimizer if more than 30% of banging
	float bangedtheta = 0; float bangedphi = 0;
	float iterate = 1;
	double calctimestart = ros::Time::now().toSec();
	while(bangedphi < 35 && bangedtheta < 35) {
		// float T = sqrt(pow((pos0[0] - posf[0]),2) + pow((pos0[1] - posf[1]),2)) / iterate;
		
		// assumption: can reach anywhere in the arena if I have ten seconds 
		float T = 8 / iterate; 
		N = round(T/dt);
		printf("Horizon: %d\n", N);  
		
		// Eigen::Matrix<double, 4, Dynamic> R;

		oldR.resize(4, 2*N);
		
		oldR.block(0, 2*N-2, 4, 2) =  B;
		Eigen::Matrix<double, 4, 4> AN = A;

		for(int i=1; i<N; i++) {
			oldR.block(0, 2*N-2*(i+1), 4, 2) =  A * oldR.block(0, 2*N-2*i, 4, 2);
			AN = A * AN; 
		}

		
		R.resize(4, 2 * N);
		R = oldR.block(0,0,4,2*N);
		
		old_H.resize(2*N, 2*N);
		old_H = 2 * (R.transpose() * P * R);
		
		eye.resize(2*N, 2*N);
		eye.setIdentity();
		eye = 0.3 * eye; // TODO: verify after this change - augnment to keep hessian invertable says harvard
		// WARNING: augmenting on diagonals to improve invertability has an impact on the rate of change of input sequence
		Eigen::MatrixXd H = 0.5 * (old_H + old_H.transpose() + eye);

		Eigen::Matrix<double, 4, 1> x0; Eigen::Matrix<double, 4, 1> xd;
		x0 << vel0[0], pos0[0], vel0[1], pos0[1];
		xd << velf[0], posf[0], velf[1], posf[1];
		
		f.resize(1, 2 * N);
		f = (2 * ((AN * x0)- xd)).transpose() * P * R;

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
					theta_ff[i] = (float) xOpt[2*i];
					phi_ff[i]   = (float) -1 * xOpt[2*i + 1];
					if ((fabs(fabs(theta_ff[i])-maxbank) < 0.01) || (fabs(fabs(phi_ff[i])-maxbank) < 0.01)) {
						bangedtheta_acc += fabs(theta_ff[i]);
						bangedphi_acc   += fabs(phi_ff[i]);
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
		iterate = iterate * 1.3;
	}
	
	double calctimestop = ros::Time::now().toSec() - calctimestart;
	printf("calc time: %f\n", calctimestop);
	lock_optimal = 1;
	
	states.resize(4, N);
	Eigen::Matrix<double,4,1> x0;
	x0 << vel0[0], pos0[0], vel0[1], pos0[1];
	states.col(0) = x0;
	Eigen::Matrix<double, 2, 1> inputs;
	double timeitisnow = ros::Time::now().toSec();
	fprintf(states_f, "%f,%f,%f\n", timeitisnow, states(0,1), states(0,3));

	printf("here!\n\n\n");
	
	for (int i=0; i<N-1; i++) {
		inputs(0) = theta_ff[i];
		inputs(1) =  -1 * phi_ff[i];
		states.col(i+1) = A * states.col(i) + B * inputs; 
		fprintf(states_f, "%f,%f,%f\n", timeitisnow, states(1,i+1), states(3,i+1));
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
	/*
	if (lock_optimal) {
		fprintf(plotopt_f, "%f,%f,%f,%f,%f,%f\n", groundTruth_msg.transforms[0].header.stamp.toSec(),
        x_est, y_est, z_est, pitch_est, roll_est);
	}
	*/
}


double start = 0;
ros::Time startrostime;

void optimalJoystick_cb(std_msgs::Empty::Ptr msg) {
	printf("[%d] starting optimal control calc\n", invoke_cnt);
	i = 0;
	optimal_calc();
	start = ros::Time::now().toSec();
	startrostime = ros::Time::now();
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
	plotopt_f = fopen("plotopt.csv", "w+");
	states_f = fopen("states.csv", "w+");


	//options.enableFlippingBounds = BT_FALSE;
	optionsmpc.printLevel = PL_HIGH;  // DEBUG level high, but the prints are muted in the library :(
	optionsmpc.initialStatusBounds = ST_INACTIVE;
	optionsmpc.numRefinementSteps = 1;

	ros::NodeHandle nh;

	// for showing the rosbag replay
	ros::Publisher optimalcmd_cpy = nh.advertise<mav_msgs::RateThrust>("optimalcmdcpy",1);

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
	
	double newAng_theta = 0;
	double newAng_phi = 0;
	mav_msgs::RateThrust opt_cmd;
	mav_msgs::RateThrust opt_cmd_cpy;
	while(ros::ok()) {
		
		if(lock_optimal && i < N) {


            double curr_error_pos_w_x = states(i,1) - x_est;
            double curr_error_pos_w_y = states(i,3) - y_est;

            double curr_error_pos_x_velframe =  cos(yaw_est)*curr_error_pos_w_x + sin(yaw_est)*curr_error_pos_w_y;
            double curr_error_pos_y_velframe = -sin(yaw_est)*curr_error_pos_w_x + cos(yaw_est)*curr_error_pos_w_y;

            double vel_x_cmd_velframe = curr_error_pos_x_velframe * KP_POS_X;
            double vel_y_cmd_velframe = curr_error_pos_y_velframe * KP_POS_Y;

            vel_x_cmd_velframe = bound_f(vel_x_cmd_velframe, -MAX_VEL_X, MAX_VEL_X);
            vel_y_cmd_velframe = bound_f(vel_y_cmd_velframe, -MAX_VEL_Y, MAX_VEL_Y);

            // vel_x_cmd_velframe += 5; //pitch more for gate vel

            double xVel_est_velframe =  cos(yaw_est) * xVel_est + sin(yaw_est) * yVel_est;
            double yVel_est_velframe = -sin(yaw_est) * xVel_est + cos(yaw_est) * yVel_est;

            double curr_error_vel_x = (vel_x_cmd_velframe - xVel_est_velframe);
            double curr_error_vel_y = (vel_y_cmd_velframe - yVel_est_velframe);

            double pitch_fb =   curr_error_vel_x * KP_VEL_X; 
            double roll_fb  = -(curr_error_vel_y * KP_VEL_Y); 

            pitch_fb = 0; // bound_f(pitch_fb, -maxbank, maxbank);
            roll_fb  = 0; //bound_f(roll_fb,  -maxbank, maxbank);

			double theta_cmd = theta_ff[i] + pitch_fb;
			double phi_cmd   = phi_ff[i]   + roll_fb;

			newAng_theta = cos(yaw_est) * theta_cmd - sin(yaw_est) * phi_cmd;
			newAng_phi   = sin(yaw_est) * theta_cmd + cos(yaw_est) * phi_cmd;	
			opt_cmd.angular_rates.x = newAng_phi;
			opt_cmd.angular_rates.y = newAng_theta;
			opt_cmd.angular_rates.z = 0;
			opt_cmd.thrust.z = 9.81;
			optimalcmd_pub.publish(opt_cmd);
			// cout << "roll: " << phi_ff[i] * 180/3.142 << ", pitch: " << theta_ff[i] * 180/3.142 << endl;
			opt_cmd_cpy.header.stamp = ros::Time::now(); //- startrostime;
			opt_cmd_cpy.angular_rates.x = newAng_phi * 180/3.142;
			opt_cmd_cpy.angular_rates.y = newAng_theta * 180/3.142;
			opt_cmd_cpy.angular_rates.z = maxbank * 180/3.142;
			opt_cmd_cpy.thrust.z = -maxbank * 180/3.142;
			optimalcmd_cpy.publish(opt_cmd_cpy);

			i++;
		}
		if (i > N-1) {
			opt_cmd.angular_rates.x = 0;
			opt_cmd.angular_rates.y = 0;
			opt_cmd.angular_rates.z = 0;
			opt_cmd.thrust.z = 9.81;
			optimalcmd_pub.publish(opt_cmd);
			double completion_time = ros::Time::now().toSec() - start;
			printf("Finished sequence in %f seconds \n", completion_time);
			i = 0;
			lock_optimal = 0;

			opt_cmd_cpy.angular_rates.x = 0;
			opt_cmd_cpy.angular_rates.y = 0;
			opt_cmd_cpy.angular_rates.z = maxbank * 180/3.142;
			opt_cmd_cpy.thrust.z = -maxbank * 180/3.142;
			optimalcmd_cpy.publish(opt_cmd_cpy);

		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}


/*
 *	end of file
 */
