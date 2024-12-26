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
 *    \file   examples/controller/getting_started.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2010
 */




#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


using namespace std;
    
USING_NAMESPACE_ACADO


int main( )
{
    USING_NAMESPACE_ACADO
    
    
    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState         theta_1; // Link 1 angular position
    DifferentialState         theta_2; // Link 2 angular position
    DifferentialState         theta_3; // Link 3 angular position
    DifferentialState         theta_4; // Link 4 angular position
    DifferentialState         theta_5; // Link 5 angular position
    DifferentialState         theta_6; // Link 6 angular position


    DifferentialState         theta_dot_1; // Link 1 angular velocity
    DifferentialState         theta_dot_2; // Link 2 angular velocity
    DifferentialState         theta_dot_3; // Link 3 angular velocity
    DifferentialState         theta_dot_4; // Link 4 angular velocity
    DifferentialState         theta_dot_5; // Link 5 angular velocity
    DifferentialState         theta_dot_6; // Link 6 angular velocity


    Control                   u_1; // Link 1 angular acceleration
    Control                   u_2; // Link 2 angular acceleration
    Control                   u_3; // Link 1 angular acceleration
    Control                   u_4; // Link 1 angular acceleration
    Control                   u_5; // Link 1 angular acceleration
    Control                   u_6; // Link 1 angular acceleration
 
    double d_1 =  0.08920;           // Offset of link 1
    double d_4 =  0.10900;           // Offset of link 4
    double d_5 =  0.09300;           // Offset of link 5
    double d_6 =  0.08200;           // Offset of link 6
    double a_2 = -0.42500;           // Length of link 2
    double a_3 = -0.39243;			 // Length of link 3
    
    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    DifferentialEquation f;
    
    f << dot(theta_1) == theta_dot_1;
    f << dot(theta_2) == theta_dot_2;
    f << dot(theta_3) == theta_dot_3;
    f << dot(theta_4) == theta_dot_4;
    f << dot(theta_5) == theta_dot_5;
    f << dot(theta_6) == theta_dot_6;
    f << dot(theta_dot_1) == u_1;
    f << dot(theta_dot_2) == u_2;
    f << dot(theta_dot_3) == u_3;
    f << dot(theta_dot_4) == u_4;
    f << dot(theta_dot_5) == u_5;
    f << dot(theta_dot_6) == u_6;


    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h, hN;
    // x
    h << d_6*(cos(theta_5)*sin(theta_1)-cos(theta_2+theta_3+theta_4)*cos(theta_1)*sin(theta_5)) + \
    	   d_4*sin(theta_1) + \
    	   cos(theta_1)*(a_3*cos(theta_2+theta_3) + a_2*cos(theta_2)) + \
    	   d_5*sin(theta_2+theta_3+theta_4)*cos(theta_1);
   	// y
   	h << sin(theta_1)*(a_3*cos(theta_2+theta_3) + a_2*cos(theta_2)) - \
   		   d_4*cos(theta_1) - \
   		   d_6*(cos(theta_1)*cos(theta_5) + cos(theta_2+theta_3+theta_4)*sin(theta_1)*sin(theta_5)) + \
   		   d_5*sin(theta_2+theta_3+theta_4)*sin(theta_1);
   	// z
   	h << d_1 + \
   		   a_3*sin(theta_2+theta_3) + \
   		   a_2*sin(theta_2) - \
   		   d_5*cos(theta_2+theta_3+theta_4) - \
   		   d_6*sin(theta_2+theta_3+theta_4)*sin(theta_5);


    // q0
    h << 0.35355*cos(0.5*theta_1 - 0.5*theta_2 - 0.5*theta_3 - 0.5*theta_4 + 0.5*theta_5 + 0.5*theta_6) -\
         0.35355*cos(0.5*theta_2 - 0.5*theta_1 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 - 0.5*theta_6) -\
         0.35355*cos(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 - 0.5*theta_5 + 0.5*theta_6) -\
         0.35355*cos(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 + 0.5*theta_6);


    // q1
    h << 0.35355*cos(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 - 0.5*theta_6) -\
         0.35355*cos(0.5*theta_2 - 0.5*theta_1 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 + 0.5*theta_6) -\
         0.35355*cos(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 - 0.5*theta_5 - 0.5*theta_6) -\
         0.35355*cos(0.5*theta_2 - 0.5*theta_1 + 0.5*theta_3 + 0.5*theta_4 - 0.5*theta_5 + 0.5*theta_6);


    // q2
    h << 0.35355*sin(0.5*theta_2 - 0.5*theta_1 + 0.5*theta_3 + 0.5*theta_4 - 0.5*theta_5 + 0.5*theta_6) +\
         0.35355*sin(0.5*theta_2 - 0.5*theta_1 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 + 0.5*theta_6) -\
         0.35355*sin(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 - 0.5*theta_5 - 0.5*theta_6) +\
         0.35355*sin(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 - 0.5*theta_6);


    // q3
    h << 0.35355*sin(0.5*theta_2 - 0.5*theta_1 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 - 0.5*theta_6) +\
         0.35355*sin(0.5*theta_1 - 0.5*theta_2 - 0.5*theta_3 - 0.5*theta_4 + 0.5*theta_5 + 0.5*theta_6) -\
         0.35355*sin(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 - 0.5*theta_5 + 0.5*theta_6) -\
         0.35355*sin(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 + 0.5*theta_6);


   	/*** the final homegenous matrix is:
    final form is in the following order: x, y, z, q0, q1, q2, q3 


	  q0, q1, q2, q3 
    ***/


   	// velocities
   	h << theta_dot_1;
   	h << theta_dot_2;
   	h << theta_dot_3;
   	h << theta_dot_4;
   	h << theta_dot_5;
   	h << theta_dot_6;


   	// inputs: acceleration
    h << u_1;
    h << u_2;
    h << u_3;
    h << u_4;
    h << u_5;
    h << u_6;




    hN << d_6*(cos(theta_5)*sin(theta_1)-cos(theta_2+theta_3+theta_4)*cos(theta_1)*sin(theta_5)) + \
       d_4*sin(theta_1) + \
       cos(theta_1)*(a_3*cos(theta_2+theta_3) + a_2*cos(theta_2)) + \
       d_5*sin(theta_2+theta_3+theta_4)*cos(theta_1);
    hN << sin(theta_1)*(a_3*cos(theta_2+theta_3) + a_2*cos(theta_2)) - \
       d_4*cos(theta_1) - \
       d_6*(cos(theta_1)*cos(theta_5) + cos(theta_2+theta_3+theta_4)*sin(theta_1)*sin(theta_5)) + \
       d_5*sin(theta_2+theta_3+theta_4)*sin(theta_1);
    hN << d_1 + \
       a_3*sin(theta_2+theta_3) + \
       a_2*sin(theta_2) - \
       d_5*cos(theta_2+theta_3+theta_4) - \
       d_6*sin(theta_2+theta_3+theta_4)*sin(theta_5);


    // q0
    hN << 0.35355*cos(0.5*theta_1 - 0.5*theta_2 - 0.5*theta_3 - 0.5*theta_4 + 0.5*theta_5 + 0.5*theta_6) -\
         0.35355*cos(0.5*theta_2 - 0.5*theta_1 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 - 0.5*theta_6) -\
         0.35355*cos(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 - 0.5*theta_5 + 0.5*theta_6) -\
         0.35355*cos(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 + 0.5*theta_6);


    // q1
    hN << 0.35355*cos(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 - 0.5*theta_6) -\
         0.35355*cos(0.5*theta_2 - 0.5*theta_1 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 + 0.5*theta_6) -\
         0.35355*cos(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 - 0.5*theta_5 - 0.5*theta_6) -\
         0.35355*cos(0.5*theta_2 - 0.5*theta_1 + 0.5*theta_3 + 0.5*theta_4 - 0.5*theta_5 + 0.5*theta_6);


    // q2
    hN << 0.35355*sin(0.5*theta_2 - 0.5*theta_1 + 0.5*theta_3 + 0.5*theta_4 - 0.5*theta_5 + 0.5*theta_6) +\
         0.35355*sin(0.5*theta_2 - 0.5*theta_1 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 + 0.5*theta_6) -\
         0.35355*sin(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 - 0.5*theta_5 - 0.5*theta_6) +\
         0.35355*sin(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 - 0.5*theta_6);


    // q3
    hN << 0.35355*sin(0.5*theta_2 - 0.5*theta_1 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 - 0.5*theta_6) +\
         0.35355*sin(0.5*theta_1 - 0.5*theta_2 - 0.5*theta_3 - 0.5*theta_4 + 0.5*theta_5 + 0.5*theta_6) -\
         0.35355*sin(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 - 0.5*theta_5 + 0.5*theta_6) -\
         0.35355*sin(0.5*theta_1 + 0.5*theta_2 + 0.5*theta_3 + 0.5*theta_4 + 0.5*theta_5 + 0.5*theta_6);


    // velocities
    hN << theta_dot_1;
    hN << theta_dot_2;
    hN << theta_dot_3;
    hN << theta_dot_4;
    hN << theta_dot_5;
    hN << theta_dot_6;
    
     // weights
    /* mpc_dll_1.dll
    DMatrix Q(19,19);
    Q(0,0) = 1.0; // x
    Q(1,1) = 1.0; // y
    Q(2,2) = 1.0; // z
    Q(3,3) = 1.0; // q0
    Q(4,4) = 1.0; // q1
    Q(5,5) = 1.0; // q2
    Q(6,6) = 1.0; // q3
    Q(7,7) = 5.0e-1; // velocity weight starts here
    Q(8,8) = 5.0e-1;
    Q(9,9) = 5.0e-1;
    Q(10,10) = 5.0e-1;
    Q(11,11) = 5.0e-1;
    Q(12,12) = 5.0e-1;
    Q(13,13) = 1.0e-6;// acceleration weights starts here
    Q(14,14) = 1.0e-6;
    Q(15,15) = 1.0e-6;
    Q(16,16) = 1.0e-6;
    Q(17,17) = 1.0e-6;
    Q(18,18) = 1.0e-6;*/


    DMatrix Q(19,19);
    Q(0,0) = 1.0; // x
    Q(1,1) = 1.0; // y
    Q(2,2) = 1.0; // z
    Q(3,3) = 1.0; // q0
    Q(4,4) = 1.0; // q1
    Q(5,5) = 1.0; // q2
    Q(6,6) = 1.0; // q3
    Q(7,7) = 1.0e-3; // velocity weight starts here
    Q(8,8) = 1.0e-3;
    Q(9,9) = 1.0e-3;
    Q(10,10) = 1.0e-3;
    Q(11,11) = 1.0e-3;
    Q(12,12) = 1.0e-3;
    Q(13,13) = 1.0e-3;// acceleration weights starts here
    Q(14,14) = 1.0e-3;
    Q(15,15) = 1.0e-3;
    Q(16,16) = 1.0e-3;
    Q(17,17) = 1.0e-3;
    Q(18,18) = 1.0e-3;


    DMatrix QN(13,13);
    QN(0,0) = 1.0e-8;
    QN(1,1) = 1.0e-8;
    QN(2,2) = 1.0e-8;
    QN(3,3) = 1.0e-8;
    QN(4,4) = 1.0e-8;
    QN(5,5) = 1.0e-8;
    QN(6,6) = 1.0e-8;
    QN(7,7) = 1.0e-8; // velocity weight starts here
    QN(8,8) = 1.0e-8;
    QN(9,9) = 1.0e-8;
    QN(10,10) = 1.0e-8;
    QN(11,11) = 1.0e-8;
    QN(12,12) = 1.0e-8;
    
    // Reference
    /*DVector r(19);
    r.setAll( 0.0 );
    
    r(0)=-0.318130;
    r(1)=-0.391535;
    r(2)=-0.504154;
    r(3)=-0.129410;
    r(4)=-0.612372;
    r(5)=0.612372;
    r(6)=-0.482963;*/
    
    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    const double tStart = 0.0;
    const double tEnd   = 1.0;
    
    OCP ocp( tStart, tEnd, 10 );
    
    ocp.minimizeLSQ( Q, h );
    ocp.minimizeLSQEndTerm( QN, hN);
    
    ocp.subjectTo( f );
    
    ocp.subjectTo( -5.0 <= u_1 <= 5.0 );
    ocp.subjectTo( -5.0 <= u_2 <= 5.0 );
    ocp.subjectTo( -5.0 <= u_3 <= 5.0 );
    ocp.subjectTo( -5.0 <= u_4 <= 5.0 );
    ocp.subjectTo( -5.0 <= u_5 <= 5.0 );
    ocp.subjectTo( -5.0 <= u_6 <= 5.0 );
    ocp.subjectTo( -2.5 <= theta_dot_1 <= 2.5 );
    ocp.subjectTo( -2.5 <= theta_dot_2 <= 2.5 );
    ocp.subjectTo( -2.5 <= theta_dot_3 <= 2.5 );
    ocp.subjectTo( -1.5 <= theta_dot_4 <= 1.5 );
    ocp.subjectTo( -2.5 <= theta_dot_5 <= 2.5 );
    ocp.subjectTo( -2.5 <= theta_dot_6 <= 2.5 );
    //ocp.subjectTo( -180.0 <= theta_2 <= 0.0 );
    ocp.subjectTo( -3.14 <= theta_4 <= 0.0 );
    ocp.subjectTo( -2.0  <= theta_5 <= 2.0 );
    //ocp.subjectTo( -90.0  <= theta_6 <= 90.0 );


	// above z axis
	/*double offset_above_z = 0.2;
	IntermediateState z1 = -offset_above_z + d_1 + a_2*sin(theta_2);
	IntermediateState z2 = z1 + a_3*sin(theta_2 + theta_3);
	IntermediateState z3 = z2 - d_5*cos(theta_2 + theta_3 + theta_4);
	ocp.subjectTo( z1 >= 0.0 );
	ocp.subjectTo( z2 >= 0.0 );
	ocp.subjectTo( z3 >= 0.0 );*/


   	// singularity avoidance
   	/*double zeta_singularity = 1.0e-18;
   	IntermediateState a_2xa_3xsinth_3 = a_2*a_3*sin(theta_3);
   	IntermediateState a_3xcosth_2_th_3 = a_3*cos(theta_2+theta_3);
   	IntermediateState a_2costh_2 = a_2*cos(theta_2);
   	IntermediateState d_5xsinth_2_3_4 = d_5*cos(theta_2 + theta_3 + theta_4);
   	IntermediateState det_J = - (a_2xa_3xsinth_3)*(a_2xa_3xsinth_3)*(-sin(theta_5)*sin(theta_5))*((a_3xcosth_2_th_3)*(a_3xcosth_2_th_3) + a_2costh_2*a_2costh_2 - d_5xsinth_2_3_4*d_5xsinth_2_3_4 + d_5*d_5 + a_3*d_5*sin(theta_4) + a_3*d_5xsinth_2_3_4 + 2*d_5xsinth_2_3_4*a_2costh_2 + 2*a_3xcosth_2_th_3*a_2costh_2);
   	ocp.subjectTo(-zeta_singularity + det_J*det_J >= 0);*/


    // Export the code:
    OCPexport mpc( ocp );


    mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
    mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );
    mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
    mpc.set( NUM_INTEGRATOR_STEPS,        30        );


    mpc.set( QP_SOLVER,                   QP_QPOASES      );
//  mpc.set( HOTSTART_QP,                 YES             );
//  mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
//  mpc.set( GENERATE_TEST_FILE,          YES             );
//  mpc.set( GENERATE_MAKE_FILE,          YES             );
    // mpc.set( GENERATE_MATLAB_INTERFACE,   YES             );
    // mpc.set( GENERATE_SIMULINK_INTERFACE, YES             );


    if (mpc.exportCode( "mpc_export_pre" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );


    mpc.printDimensionsQP( );
    
    return EXIT_SUCCESS;
}
