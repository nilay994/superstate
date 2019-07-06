#ifndef TRAJ_H
#define TRAJ_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/LU>
#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <mav_msgs/RateThrust.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>



// MAKE FUNCTIONS
struct polymat {
	Eigen::Matrix<double, 1, 8> p  ;
	Eigen::Matrix<double, 1, 8> pd ;
	Eigen::Matrix<double, 1, 8> pd2;
	Eigen::Matrix<double, 1, 8> pd3;
	Eigen::Matrix<double, 1, 8> pd4;
	Eigen::Matrix<double, 1, 8> pd5;
	Eigen::Matrix<double, 1, 8> pd6;
};


struct Vel {
	double vx;
	double vy;
	double vz;
	double psid;
};


struct Parameters {
	double g;
	double m;
};


Eigen::MatrixXd calcTimepersegmentAvgVel(Eigen::MatrixXd x, Eigen::MatrixXd y, Eigen::MatrixXd z,Eigen::MatrixXd psi, int n, double avgvel) ;
void mellinger_kumarplanner(Eigen::MatrixXd T, int n,double t0, Eigen::MatrixXd x, Eigen::MatrixXd y, Eigen::MatrixXd z, Eigen::MatrixXd psi, Vel initvel,Eigen::MatrixXd& pos2_p, Eigen::MatrixXd& pos2_pd, Eigen::MatrixXd& pos2_pd2, Eigen::MatrixXd& pos2_pd4, Eigen::MatrixXd& t, Eigen::MatrixXd& theta, Eigen::MatrixXd& phi2, Eigen::MatrixXd& fz1, Parameters param, int freq) ;
 
Eigen::MatrixXd polynom_p  (double Ta, double s, double t); 
Eigen::MatrixXd polynom_pd (double Ta, double s, double t); 
Eigen::MatrixXd polynom_pd2(double Ta, double s, double t); 
Eigen::MatrixXd polynom_pd3(double Ta, double s, double t); 
Eigen::MatrixXd polynom_pd4(double Ta, double s, double t); 
Eigen::MatrixXd polynom_pd5(double Ta, double s, double t); 
Eigen::MatrixXd polynom_pd6(double Ta, double s, double t); 

// Constructing A matrix
Eigen::MatrixXd make_A1 (int n, Eigen::MatrixXd T, Eigen::MatrixXd S);
Eigen::MatrixXd make_A2 (int n, Eigen::MatrixXd T, Eigen::MatrixXd S);
Eigen::MatrixXd make_A3 (int n, Eigen::MatrixXd T, Eigen::MatrixXd S);

Eigen::MatrixXd make_A_way1 (Eigen::MatrixXd adash2, Eigen::MatrixXd adash3, Eigen::MatrixXd adash5, int n);
Eigen::MatrixXd make_A_way2 (Eigen::MatrixXd adash2, Eigen::MatrixXd adash5, int n);

Eigen::MatrixXd make_B (int n, Eigen::MatrixXd x, Eigen::MatrixXd y, Eigen::MatrixXd z, Eigen::MatrixXd psi, Vel initvel);

Eigen::MatrixXd compute_sz(Eigen::MatrixXd S, int var, double gap, int n) ;

Eigen::MatrixXd construct_S (Eigen::MatrixXd T, int n, double t0);

void func_gener_v2(Eigen::MatrixXd sz, Eigen::MatrixXd coef,Eigen::MatrixXd S,int idt,int idx,double gap,Eigen::MatrixXd T, Eigen::MatrixXd& t, Eigen::MatrixXd& pos2_p, Eigen::MatrixXd& pos2_pd, Eigen::MatrixXd& pos2_pd2, Eigen::MatrixXd& pos2_pd4);
void func_vars_generatv2(Eigen::MatrixXd S, Eigen::MatrixXd T, double t0, Eigen::MatrixXd coef, double gap, int n, int idt,Eigen::MatrixXd sz, Eigen::MatrixXd& t, Eigen::MatrixXd& pos2_p, Eigen::MatrixXd& pos2_pd, Eigen::MatrixXd& pos2_pd2, Eigen::MatrixXd& pos2_pd4);
Eigen::MatrixXd my_atan2(Eigen::MatrixXd y, Eigen::MatrixXd x) ;

void func_inputs4(Eigen::MatrixXd pos2_p,Eigen::MatrixXd pos2_pd,Eigen::MatrixXd pos2_pd2,Eigen::MatrixXd pos2_pd4,Eigen::MatrixXd& theta,Eigen::MatrixXd& phi2,Eigen::MatrixXd& fz1, Parameters param);
void traj_calc(double pos[3], double vel[3], double psi0, Eigen::MatrixXd& theta, Eigen::MatrixXd& phi2, Eigen::MatrixXd& fz1);
#endif