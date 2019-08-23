/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

// Standalone code generation for a parameter-free quadrotor model
// with thrust and rates input. 

int main( ){
	// Use Acado
	USING_NAMESPACE_ACADO

	/*
	Switch between code generation and analysis.

	If CODE_GEN is true the system is compiled into an optimizaiton problem
	for real-time iteration and all code to run it online is generated.
	Constraints and reference structure is used but the values will be set on
	runtinme.

	If CODE_GEN is false, the system is compiled into a standalone optimization
	and solved on execution. The reference and constraints must be set in here.
	*/
	const bool CODE_GEN = 1;

	// Parameters with exemplary values. These are set/overwritten at runtime.
	const double t_start = 0.0;     // Initial time [s]
	const double t_end = 2.0;       // Time horizon [s]
	const double dt = 0.1;          // Discretization time [s]
	const int N = round(t_end/dt);  // Number of nodes
	const double g_z = 9.8066;      // Gravity is everywhere [m/s^2]

	// System variables
	DifferentialState        px,py,vx,vy;     // the differential states
	Control                  theta,phi  ;     // the control input u
	Parameter                T          ;     // the time horizon T
	DifferentialEquation     f( 0.0, T );     // the differential equation
	Function              	 h, hN;

	f << dot(px) == vx;
	f << dot(py) == vy;
	f << dot(vx) == tan(theta) * 9.81 - 0.5 * vx;               // an implementation
	f << dot(vy) == -tan(phi) / cos(theta) * 9.81 - 0.5 * vy;   // of the model equations

	// Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
	// Running cost vector consists of all states and inputs.
	// h << px << py << vx << vy << theta << phi;
	h << 0;

	// End cost vector consists of all states (no inputs at last state).
	hN << px << py << vx << vy;

	// Running cost weight matrix
	DMatrix Q(h.getDim(), h.getDim());
	Q.setZero();

	// End cost weight matrix
	DMatrix QN(hN.getDim(), hN.getDim());
	QN.setIdentity(); // increase penalty on position

	// Set a reference for the analysis (if CODE_GEN is false).
	// Reference is at x = 2.0m in hover (qw = 1).
	// no need of mid reference stuff
	// DVector r(h.getDim());    // Running cost reference
	// r.setZero();
	// r(0) = 2.0;
	// r(3) = 1.0;
	// r(10) = g_z;

	DVector rN(hN.getDim());   // End cost reference
	rN.setZero();
	rN(0) = 5.0;
	rN(1) = -5.0;
	rN(2) = 2.0; // 2m/s
	rN(3) = 0.0; // no lateral speed

	// DEFINE AN OPTIMAL CONTROL PROBLEM:
	// ----------------------------------
	// OCP ocp( t_start, t_end, N );
	// System Dynamics
	OCP ocp( 0.0, T );                        // time horizon of the OCP: [0,T]
	ocp.minimizeMayerTerm( T );               // the time T should be optimized

	if(!CODE_GEN) {
		// For analysis, set references.
		//ocp.minimizeLSQ( Q, h, r );
		ocp.minimizeLSQEndTerm( QN, hN, rN );
	} 
	else {
		// For code generation, references are set during run time.
		BMatrix Q_sparse(h.getDim(), h.getDim());
		Q_sparse.setIdentity();
		ocp.minimizeLSQ(Q_sparse, h);
		BMatrix QN_sparse(hN.getDim(), hN.getDim());
		QN_sparse.setIdentity();
		ocp.minimizeLSQEndTerm( QN_sparse, hN );
	}

    ocp.subjectTo(f                   );     // minimize T s.t. the model,
    ocp.subjectTo(AT_START, px == 0.0);     // the initial values for s,
    ocp.subjectTo(AT_START, py == 0.0);     // v,
    ocp.subjectTo(AT_START, vx == 0.0);  
    ocp.subjectTo(AT_START, vy == 0.0);  

    // ocp.subjectTo( AT_END, px ==  10.0 );
    // ocp.subjectTo( AT_END, py ==  10.0 );  
    // ocp.subjectTo( AT_END, vy ==  0.0 );  

    ocp.subjectTo(-1 <= theta <= 1);     // as well as the bounds on v
    ocp.subjectTo(-1 <= phi <= 1);     // the control input u,
    // ocp.subjectTo(0.5 <= T <= 10.0);     // and the time horizon T.

	// ocp.setNOD(10);

	if(!CODE_GEN) {
		GnuplotWindow window;
		window.addSubplot( px, "pos x" );
		window.addSubplot( py, "pos y" );
		window.addSubplot( vx, "vel x" );
		window.addSubplot( vy, "vel y" );
		window.addSubplot( theta, "theta" );
		window.addSubplot( phi, "phi" );
		OptimizationAlgorithm algorithm(ocp);     // the optimization algorithm
		algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
		algorithm.set( KKT_TOLERANCE, 1e-3 );
		algorithm << window;
		algorithm.solve();                        // solves the problem.

	} else {
		// For code generation, we can set some properties.
		// The main reason for a setting is given as comment.
		OCPexport mpc(ocp);

		mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
		mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
		mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
		mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
		mpc.set(NUM_INTEGRATOR_STEPS,   N);
		mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
		mpc.set(HOTSTART_QP,            NO);
		mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
		mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
		mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
		mpc.set( USE_SINGLE_PRECISION,            YES);       // Single precision

		// Do not generate tests, makes or matlab-related interfaces.
		mpc.set( GENERATE_TEST_FILE,          YES);
		mpc.set( GENERATE_MAKE_FILE,          YES);
		mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
		mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

		// Finally, export everything.
		if(mpc.exportCode("quadrotor_mpc_codegen") != SUCCESSFUL_RETURN)
			exit( EXIT_FAILURE );
		
		mpc.printDimensionsQP( );
	}

  return EXIT_SUCCESS;
}



