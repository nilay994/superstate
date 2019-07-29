/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */



 /**
 *    \file   examples/getting_started/simple_ocp.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <math.h>

int main( ){

    USING_NAMESPACE_ACADO


    DifferentialState        px,py,vx,vy;     // the differential states
    Control                  theta,phi  ;     // the control input u
    Parameter                T          ;     // the time horizon T
    DifferentialEquation     f( 0.0, T );     // the differential equation

//  -------------------------------------
    OCP ocp( 0.0, T );                        // time horizon of the OCP: [0,T]
    ocp.minimizeMayerTerm( T );               // the time T should be optimized

    f << dot(px) == vx;
    f << dot(py) == vy;
    f << dot(vx) == tan(theta) * 9.81 - 0.5 * vx;               // an implementation
    f << dot(vy) == -tan(phi) / cos(theta) * 9.81 - 0.5 * vy;   // of the model equations

    ocp.subjectTo( f                   );     // minimize T s.t. the model,
    ocp.subjectTo( AT_START, px ==  0.0 );     // the initial values for s,
    ocp.subjectTo( AT_START, py ==  0.0 );     // v,
    ocp.subjectTo( AT_START, vx ==  0.0 );  
    ocp.subjectTo( AT_START, vy ==  0.0 );  

    ocp.subjectTo( AT_END, px ==  10.0 );
    ocp.subjectTo( AT_END, py ==  10.0 );  
    ocp.subjectTo( AT_END, vy ==  0.0 );  

    ocp.subjectTo( -1 <= theta <=  1   );     // as well as the bounds on v
    ocp.subjectTo( -1 <= phi <=  1   );     // the control input u,
    ocp.subjectTo(  0.5 <= T <= 10.0   );     // and the time horizon T.
//  -------------------------------------

    GnuplotWindow window;
        window.addSubplot( px, "pos x" );
        window.addSubplot( py, "pos y" );
        window.addSubplot( vx, "vel x" );
        window.addSubplot( vy, "vel y" );
        window.addSubplot( theta, "theta" );
        window.addSubplot( phi, "phi" );
    OptimizationAlgorithm algorithm(ocp);     // the optimization algorithm
    algorithm << window;
    algorithm.solve();                        // solves the problem.


    return 0;
}
