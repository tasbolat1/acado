/*
 *    This file is part of ACADO Toolkit.
 *
 */






/**
 *    \file   examples/controller/panda.cpp
 *    \author Tasbolat Taunyazov
 *    \date   2024
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
    DifferentialState         theta_7; // Link 6 angular position



    DifferentialState         theta_dot_1; // Link 1 angular velocity
    DifferentialState         theta_dot_2; // Link 2 angular velocity
    DifferentialState         theta_dot_3; // Link 3 angular velocity
    DifferentialState         theta_dot_4; // Link 4 angular velocity
    DifferentialState         theta_dot_5; // Link 5 angular velocity
    DifferentialState         theta_dot_6; // Link 6 angular velocity
    DifferentialState         theta_dot_7; // Link 6 angular velocity


    Control                   u_1; // Link 1 angular acceleration
    Control                   u_2; // Link 2 angular acceleration
    Control                   u_3; // Link 1 angular acceleration
    Control                   u_4; // Link 1 angular acceleration
    Control                   u_5; // Link 1 angular acceleration
    Control                   u_6; // Link 1 angular acceleration
    Control                   u_7; // Link 1 angular acceleration
    
    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    DifferentialEquation f;
    
    f << dot(theta_1) == theta_dot_1;
    f << dot(theta_2) == theta_dot_2;
    f << dot(theta_3) == theta_dot_3;
    f << dot(theta_4) == theta_dot_4;
    f << dot(theta_5) == theta_dot_5;
    f << dot(theta_6) == theta_dot_6;
    f << dot(theta_7) == theta_dot_7;
    f << dot(theta_dot_1) == u_1;
    f << dot(theta_dot_2) == u_2;
    f << dot(theta_dot_3) == u_3;
    f << dot(theta_dot_4) == u_4;
    f << dot(theta_dot_5) == u_5;
    f << dot(theta_dot_6) == u_6;
    f << dot(theta_dot_7) == u_7;


    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h, hN;
    // x
    h << -0.107*(((sin(theta_1)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3))*cos(theta_4) - sin(theta_2)*sin(theta_4)*cos(theta_1))*cos(theta_5) + (sin(theta_1)*cos(theta_3) + sin(theta_3)*cos(theta_1)*cos(theta_2))*sin(theta_5))*sin(theta_6) - 0.088*(((sin(theta_1)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3))*cos(theta_4) - sin(theta_2)*sin(theta_4)*cos(theta_1))*cos(theta_5) + (sin(theta_1)*cos(theta_3) + sin(theta_3)*cos(theta_1)*cos(theta_2))*sin(theta_5))*cos(theta_6) + 0.088*((sin(theta_1)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3))*sin(theta_4) + sin(theta_2)*cos(theta_1)*cos(theta_4))*sin(theta_6) - 0.107*((sin(theta_1)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3))*sin(theta_4) + sin(theta_2)*cos(theta_1)*cos(theta_4))*cos(theta_6) + 0.384*(sin(theta_1)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3))*sin(theta_4) + 0.0825*(sin(theta_1)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3))*cos(theta_4) - 0.0825*sin(theta_1)*sin(theta_3) - 0.0825*sin(theta_2)*sin(theta_4)*cos(theta_1) + 0.384*sin(theta_2)*cos(theta_1)*cos(theta_4) + 0.316*sin(theta_2)*cos(theta_1) + 0.0825*cos(theta_1)*cos(theta_2)*cos(theta_3);

   	// y
     h << 0.107*(((sin(theta_1)*cos(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*cos(theta_4) + sin(theta_1)*sin(theta_2)*sin(theta_4))*cos(theta_5) - (sin(theta_1)*sin(theta_3)*cos(theta_2) - cos(theta_1)*cos(theta_3))*sin(theta_5))*sin(theta_6) + 0.088*(((sin(theta_1)*cos(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*cos(theta_4) + sin(theta_1)*sin(theta_2)*sin(theta_4))*cos(theta_5) - (sin(theta_1)*sin(theta_3)*cos(theta_2) - cos(theta_1)*cos(theta_3))*sin(theta_5))*cos(theta_6) - 0.088*((sin(theta_1)*cos(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*sin(theta_4) - sin(theta_1)*sin(theta_2)*cos(theta_4))*sin(theta_6) + 0.107*((sin(theta_1)*cos(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*sin(theta_4) - sin(theta_1)*sin(theta_2)*cos(theta_4))*cos(theta_6) - 0.384*(sin(theta_1)*cos(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*sin(theta_4) - 0.0825*(sin(theta_1)*cos(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*cos(theta_4) - 0.0825*sin(theta_1)*sin(theta_2)*sin(theta_4) + 0.384*sin(theta_1)*sin(theta_2)*cos(theta_4) + 0.316*sin(theta_1)*sin(theta_2) + 0.0825*sin(theta_1)*cos(theta_2)*cos(theta_3) + 0.0825*sin(theta_3)*cos(theta_1);

     // z
   	h << -0.107*((sin(theta_2)*cos(theta_3)*cos(theta_4) - sin(theta_4)*cos(theta_2))*cos(theta_5) - sin(theta_2)*sin(theta_3)*sin(theta_5))*sin(theta_6) - 0.088*((sin(theta_2)*cos(theta_3)*cos(theta_4) - sin(theta_4)*cos(theta_2))*cos(theta_5) - sin(theta_2)*sin(theta_3)*sin(theta_5))*cos(theta_6) + 0.088*(sin(theta_2)*sin(theta_4)*cos(theta_3) + cos(theta_2)*cos(theta_4))*sin(theta_6) - 0.107*(sin(theta_2)*sin(theta_4)*cos(theta_3) + cos(theta_2)*cos(theta_4))*cos(theta_6) + 0.384*sin(theta_2)*sin(theta_4)*cos(theta_3) + 0.0825*sin(theta_2)*cos(theta_3)*cos(theta_4) - 0.0825*sin(theta_2)*cos(theta_3) - 0.0825*sin(theta_4)*cos(theta_2) + 0.384*cos(theta_2)*cos(theta_4) + 0.316*cos(theta_2) + 0.333;\
     
    // qw
    h << -sin(theta_2/2)*sin(theta_1/2 - theta_3/2 + theta_5/2 + theta_7/2)*cos(theta_4/2 - theta_6/2)/2 + sin(theta_2/2)*sin(theta_1/2 - theta_3/2 - theta_6/2 + theta_7/2)*cos(theta_4/2 - theta_5/2)/2 - sin(theta_3/2)*sin(theta_1/2 + theta_2/2 + theta_4/2 + theta_6/2 + theta_7/2)*cos(theta_5/2)/2 + sin(theta_2/2 + theta_3/2)*sin(theta_1/2 - theta_4/2 - theta_5/2 + theta_7/2)*cos(theta_6/2)/2 + cos(theta_2/2)*cos(theta_1/2 + theta_3/2 + theta_7/2)*cos(-theta_4/2 + theta_5/2 + theta_6/2)/2 + cos(theta_3/2)*cos(theta_5/2)*cos(theta_1/2 - theta_2/2 + theta_4/2 + theta_6/2 + theta_7/2)/2 - cos(theta_4/2)*cos(theta_1/2 + theta_2/2 + theta_3/2 + theta_5/2 - theta_6/2 + theta_7/2)/4 - cos(theta_1/2 + theta_7/2)*cos(-theta_2/2 + theta_3/2 + theta_4/2 + theta_5/2 - theta_6/2)/4 - cos(theta_2/2 - theta_3/2)*cos(-theta_1/2 + theta_4/2 + theta_5/2 + theta_6/2 - theta_7/2)/4 - cos(theta_5/2 - theta_6/2)*cos(theta_1/2 - theta_2/2 + theta_3/2 - theta_4/2 + theta_7/2)/4;

    // qx
    h << -sin(theta_3/2)*sin(theta_1/2 + theta_2/2 + theta_4/2 + theta_5/2 - theta_7/2)*cos(theta_6/2)/2 - sin(theta_3/2 + theta_4/2)*sin(theta_1/2 - theta_2/2 + theta_5/2 - theta_7/2)*cos(theta_6/2)/2 + cos(theta_2/2)*cos(theta_4/2 - theta_5/2)*cos(theta_1/2 + theta_3/2 + theta_6/2 - theta_7/2)/2 + cos(theta_5/2)*cos(theta_2/2 - theta_3/2)*cos(-theta_1/2 + theta_4/2 + theta_6/2 + theta_7/2)/2 + cos(theta_3/2 - theta_4/2)*cos(-theta_1/2 - theta_2/2 + theta_5/2 + theta_6/2 + theta_7/2)/4 + cos(theta_3/2 - theta_6/2)*cos(theta_1/2 + theta_2/2 - theta_4/2 + theta_5/2 - theta_7/2)/4 + cos(theta_4/2 + theta_5/2)*cos(-theta_1/2 + theta_2/2 + theta_3/2 - theta_6/2 + theta_7/2)/4 - cos(theta_4/2 - theta_6/2)*cos(theta_1/2 + theta_2/2 + theta_3/2 - theta_5/2 - theta_7/2)/4 + cos(theta_5/2 - theta_6/2)*cos(-theta_1/2 + theta_2/2 + theta_3/2 - theta_4/2 + theta_7/2)/4 - cos(-theta_1/2 + theta_5/2 + theta_7/2)*cos(theta_2/2 - theta_3/2 - theta_4/2 + theta_6/2)/4 - cos(theta_2/2 + theta_4/2 + theta_6/2)*cos(-theta_1/2 + theta_3/2 + theta_5/2 + theta_7/2)/4 - cos(theta_3/2 - theta_4/2 + theta_6/2)*cos(-theta_1/2 + theta_2/2 + theta_5/2 + theta_7/2)/4;

    // qy
    h << sin(theta_3/2)*cos(theta_6/2)*cos(theta_1/2 + theta_2/2 + theta_4/2 + theta_5/2 - theta_7/2)/2 - sin(theta_2/2 + theta_4/2)*cos(-theta_1/2 + theta_3/2 + theta_5/2 - theta_6/2 + theta_7/2)/4 + sin(theta_3/2 + theta_4/2)*cos(theta_6/2)*cos(theta_1/2 - theta_2/2 + theta_5/2 - theta_7/2)/2 + sin(-theta_1/2 + theta_5/2 + theta_7/2)*cos(theta_2/2 - theta_3/2 - theta_4/2 + theta_6/2)/4 + sin(theta_4/2 + theta_5/2 + theta_6/2)*cos(-theta_1/2 + theta_2/2 + theta_3/2 + theta_7/2)/4 + sin(-theta_1/2 + theta_2/2 + theta_5/2 + theta_7/2)*cos(theta_3/2 - theta_4/2 + theta_6/2)/4 - sin(-theta_1/2 + theta_4/2 + theta_6/2 + theta_7/2)*cos(theta_5/2)*cos(theta_2/2 - theta_3/2)/2 + sin(theta_1/2 + theta_3/2 + theta_6/2 - theta_7/2)*cos(theta_2/2)*cos(theta_4/2 - theta_5/2)/2 - sin(-theta_1/2 - theta_2/2 + theta_5/2 + theta_6/2 + theta_7/2)*cos(theta_3/2 - theta_4/2)/4 - sin(-theta_1/2 + theta_2/2 + theta_3/2 - theta_4/2 + theta_7/2)*cos(theta_5/2 - theta_6/2)/4 - sin(theta_1/2 + theta_2/2 + theta_3/2 - theta_5/2 - theta_7/2)*cos(theta_4/2 - theta_6/2)/4 + sin(theta_1/2 + theta_2/2 - theta_4/2 + theta_5/2 - theta_7/2)*cos(theta_3/2 - theta_6/2)/4;

    // qz
    h << -sin(theta_2/2)*cos(theta_4/2 - theta_5/2)*cos(theta_1/2 - theta_3/2 - theta_6/2 + theta_7/2)/2 + sin(theta_2/2)*cos(theta_4/2 - theta_6/2)*cos(theta_1/2 - theta_3/2 + theta_5/2 + theta_7/2)/2 + sin(theta_3/2)*cos(theta_5/2)*cos(theta_1/2 + theta_2/2 + theta_4/2 + theta_6/2 + theta_7/2)/2 - sin(theta_1/2 + theta_7/2)*cos(-theta_2/2 + theta_3/2 + theta_4/2 + theta_5/2 - theta_6/2)/4 - sin(theta_2/2 + theta_3/2)*cos(theta_6/2)*cos(theta_1/2 - theta_4/2 - theta_5/2 + theta_7/2)/2 + sin(theta_1/2 + theta_3/2 + theta_7/2)*cos(theta_2/2)*cos(-theta_4/2 + theta_5/2 + theta_6/2)/2 + sin(-theta_1/2 + theta_4/2 + theta_5/2 + theta_6/2 - theta_7/2)*cos(theta_2/2 - theta_3/2)/4 - sin(theta_1/2 - theta_2/2 + theta_3/2 - theta_4/2 + theta_7/2)*cos(theta_5/2 - theta_6/2)/4 + sin(theta_1/2 - theta_2/2 + theta_4/2 + theta_6/2 + theta_7/2)*cos(theta_3/2)*cos(theta_5/2)/2 - sin(theta_1/2 + theta_2/2 + theta_3/2 + theta_5/2 - theta_6/2 + theta_7/2)*cos(theta_4/2)/4;


   	/*** the final homegenous matrix is:
    final form is in the following order: x, y, z, qw, qx, qy, qz 


	  q0, q1, q2, q3 
    ***/


   	// velocities
   	h << theta_dot_1;
   	h << theta_dot_2;
   	h << theta_dot_3;
   	h << theta_dot_4;
   	h << theta_dot_5;
   	h << theta_dot_6;
     h << theta_dot_7;

    // inputs: acceleration
    h << u_1;
    h << u_2;
    h << u_3;
    h << u_4;
    h << u_5;
    h << u_6;
    h << u_7;

    // Terminal side
    hN << -0.107*(((sin(theta_1)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3))*cos(theta_4) - sin(theta_2)*sin(theta_4)*cos(theta_1))*cos(theta_5) + (sin(theta_1)*cos(theta_3) + sin(theta_3)*cos(theta_1)*cos(theta_2))*sin(theta_5))*sin(theta_6) - 0.088*(((sin(theta_1)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3))*cos(theta_4) - sin(theta_2)*sin(theta_4)*cos(theta_1))*cos(theta_5) + (sin(theta_1)*cos(theta_3) + sin(theta_3)*cos(theta_1)*cos(theta_2))*sin(theta_5))*cos(theta_6) + 0.088*((sin(theta_1)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3))*sin(theta_4) + sin(theta_2)*cos(theta_1)*cos(theta_4))*sin(theta_6) - 0.107*((sin(theta_1)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3))*sin(theta_4) + sin(theta_2)*cos(theta_1)*cos(theta_4))*cos(theta_6) + 0.384*(sin(theta_1)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3))*sin(theta_4) + 0.0825*(sin(theta_1)*sin(theta_3) - cos(theta_1)*cos(theta_2)*cos(theta_3))*cos(theta_4) - 0.0825*sin(theta_1)*sin(theta_3) - 0.0825*sin(theta_2)*sin(theta_4)*cos(theta_1) + 0.384*sin(theta_2)*cos(theta_1)*cos(theta_4) + 0.316*sin(theta_2)*cos(theta_1) + 0.0825*cos(theta_1)*cos(theta_2)*cos(theta_3);
    hN << 0.107*(((sin(theta_1)*cos(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*cos(theta_4) + sin(theta_1)*sin(theta_2)*sin(theta_4))*cos(theta_5) - (sin(theta_1)*sin(theta_3)*cos(theta_2) - cos(theta_1)*cos(theta_3))*sin(theta_5))*sin(theta_6) + 0.088*(((sin(theta_1)*cos(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*cos(theta_4) + sin(theta_1)*sin(theta_2)*sin(theta_4))*cos(theta_5) - (sin(theta_1)*sin(theta_3)*cos(theta_2) - cos(theta_1)*cos(theta_3))*sin(theta_5))*cos(theta_6) - 0.088*((sin(theta_1)*cos(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*sin(theta_4) - sin(theta_1)*sin(theta_2)*cos(theta_4))*sin(theta_6) + 0.107*((sin(theta_1)*cos(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*sin(theta_4) - sin(theta_1)*sin(theta_2)*cos(theta_4))*cos(theta_6) - 0.384*(sin(theta_1)*cos(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*sin(theta_4) - 0.0825*(sin(theta_1)*cos(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*cos(theta_4) - 0.0825*sin(theta_1)*sin(theta_2)*sin(theta_4) + 0.384*sin(theta_1)*sin(theta_2)*cos(theta_4) + 0.316*sin(theta_1)*sin(theta_2) + 0.0825*sin(theta_1)*cos(theta_2)*cos(theta_3) + 0.0825*sin(theta_3)*cos(theta_1);
    hN << -0.107*((sin(theta_2)*cos(theta_3)*cos(theta_4) - sin(theta_4)*cos(theta_2))*cos(theta_5) - sin(theta_2)*sin(theta_3)*sin(theta_5))*sin(theta_6) - 0.088*((sin(theta_2)*cos(theta_3)*cos(theta_4) - sin(theta_4)*cos(theta_2))*cos(theta_5) - sin(theta_2)*sin(theta_3)*sin(theta_5))*cos(theta_6) + 0.088*(sin(theta_2)*sin(theta_4)*cos(theta_3) + cos(theta_2)*cos(theta_4))*sin(theta_6) - 0.107*(sin(theta_2)*sin(theta_4)*cos(theta_3) + cos(theta_2)*cos(theta_4))*cos(theta_6) + 0.384*sin(theta_2)*sin(theta_4)*cos(theta_3) + 0.0825*sin(theta_2)*cos(theta_3)*cos(theta_4) - 0.0825*sin(theta_2)*cos(theta_3) - 0.0825*sin(theta_4)*cos(theta_2) + 0.384*cos(theta_2)*cos(theta_4) + 0.316*cos(theta_2) + 0.333;
    hN << -sin(theta_2/2)*sin(theta_1/2 - theta_3/2 + theta_5/2 + theta_7/2)*cos(theta_4/2 - theta_6/2)/2 + sin(theta_2/2)*sin(theta_1/2 - theta_3/2 - theta_6/2 + theta_7/2)*cos(theta_4/2 - theta_5/2)/2 - sin(theta_3/2)*sin(theta_1/2 + theta_2/2 + theta_4/2 + theta_6/2 + theta_7/2)*cos(theta_5/2)/2 + sin(theta_2/2 + theta_3/2)*sin(theta_1/2 - theta_4/2 - theta_5/2 + theta_7/2)*cos(theta_6/2)/2 + cos(theta_2/2)*cos(theta_1/2 + theta_3/2 + theta_7/2)*cos(-theta_4/2 + theta_5/2 + theta_6/2)/2 + cos(theta_3/2)*cos(theta_5/2)*cos(theta_1/2 - theta_2/2 + theta_4/2 + theta_6/2 + theta_7/2)/2 - cos(theta_4/2)*cos(theta_1/2 + theta_2/2 + theta_3/2 + theta_5/2 - theta_6/2 + theta_7/2)/4 - cos(theta_1/2 + theta_7/2)*cos(-theta_2/2 + theta_3/2 + theta_4/2 + theta_5/2 - theta_6/2)/4 - cos(theta_2/2 - theta_3/2)*cos(-theta_1/2 + theta_4/2 + theta_5/2 + theta_6/2 - theta_7/2)/4 - cos(theta_5/2 - theta_6/2)*cos(theta_1/2 - theta_2/2 + theta_3/2 - theta_4/2 + theta_7/2)/4;
    hN << -sin(theta_3/2)*sin(theta_1/2 + theta_2/2 + theta_4/2 + theta_5/2 - theta_7/2)*cos(theta_6/2)/2 - sin(theta_3/2 + theta_4/2)*sin(theta_1/2 - theta_2/2 + theta_5/2 - theta_7/2)*cos(theta_6/2)/2 + cos(theta_2/2)*cos(theta_4/2 - theta_5/2)*cos(theta_1/2 + theta_3/2 + theta_6/2 - theta_7/2)/2 + cos(theta_5/2)*cos(theta_2/2 - theta_3/2)*cos(-theta_1/2 + theta_4/2 + theta_6/2 + theta_7/2)/2 + cos(theta_3/2 - theta_4/2)*cos(-theta_1/2 - theta_2/2 + theta_5/2 + theta_6/2 + theta_7/2)/4 + cos(theta_3/2 - theta_6/2)*cos(theta_1/2 + theta_2/2 - theta_4/2 + theta_5/2 - theta_7/2)/4 + cos(theta_4/2 + theta_5/2)*cos(-theta_1/2 + theta_2/2 + theta_3/2 - theta_6/2 + theta_7/2)/4 - cos(theta_4/2 - theta_6/2)*cos(theta_1/2 + theta_2/2 + theta_3/2 - theta_5/2 - theta_7/2)/4 + cos(theta_5/2 - theta_6/2)*cos(-theta_1/2 + theta_2/2 + theta_3/2 - theta_4/2 + theta_7/2)/4 - cos(-theta_1/2 + theta_5/2 + theta_7/2)*cos(theta_2/2 - theta_3/2 - theta_4/2 + theta_6/2)/4 - cos(theta_2/2 + theta_4/2 + theta_6/2)*cos(-theta_1/2 + theta_3/2 + theta_5/2 + theta_7/2)/4 - cos(theta_3/2 - theta_4/2 + theta_6/2)*cos(-theta_1/2 + theta_2/2 + theta_5/2 + theta_7/2)/4;
    hN << sin(theta_3/2)*cos(theta_6/2)*cos(theta_1/2 + theta_2/2 + theta_4/2 + theta_5/2 - theta_7/2)/2 - sin(theta_2/2 + theta_4/2)*cos(-theta_1/2 + theta_3/2 + theta_5/2 - theta_6/2 + theta_7/2)/4 + sin(theta_3/2 + theta_4/2)*cos(theta_6/2)*cos(theta_1/2 - theta_2/2 + theta_5/2 - theta_7/2)/2 + sin(-theta_1/2 + theta_5/2 + theta_7/2)*cos(theta_2/2 - theta_3/2 - theta_4/2 + theta_6/2)/4 + sin(theta_4/2 + theta_5/2 + theta_6/2)*cos(-theta_1/2 + theta_2/2 + theta_3/2 + theta_7/2)/4 + sin(-theta_1/2 + theta_2/2 + theta_5/2 + theta_7/2)*cos(theta_3/2 - theta_4/2 + theta_6/2)/4 - sin(-theta_1/2 + theta_4/2 + theta_6/2 + theta_7/2)*cos(theta_5/2)*cos(theta_2/2 - theta_3/2)/2 + sin(theta_1/2 + theta_3/2 + theta_6/2 - theta_7/2)*cos(theta_2/2)*cos(theta_4/2 - theta_5/2)/2 - sin(-theta_1/2 - theta_2/2 + theta_5/2 + theta_6/2 + theta_7/2)*cos(theta_3/2 - theta_4/2)/4 - sin(-theta_1/2 + theta_2/2 + theta_3/2 - theta_4/2 + theta_7/2)*cos(theta_5/2 - theta_6/2)/4 - sin(theta_1/2 + theta_2/2 + theta_3/2 - theta_5/2 - theta_7/2)*cos(theta_4/2 - theta_6/2)/4 + sin(theta_1/2 + theta_2/2 - theta_4/2 + theta_5/2 - theta_7/2)*cos(theta_3/2 - theta_6/2)/4;
    hN << -sin(theta_2/2)*cos(theta_4/2 - theta_5/2)*cos(theta_1/2 - theta_3/2 - theta_6/2 + theta_7/2)/2 + sin(theta_2/2)*cos(theta_4/2 - theta_6/2)*cos(theta_1/2 - theta_3/2 + theta_5/2 + theta_7/2)/2 + sin(theta_3/2)*cos(theta_5/2)*cos(theta_1/2 + theta_2/2 + theta_4/2 + theta_6/2 + theta_7/2)/2 - sin(theta_1/2 + theta_7/2)*cos(-theta_2/2 + theta_3/2 + theta_4/2 + theta_5/2 - theta_6/2)/4 - sin(theta_2/2 + theta_3/2)*cos(theta_6/2)*cos(theta_1/2 - theta_4/2 - theta_5/2 + theta_7/2)/2 + sin(theta_1/2 + theta_3/2 + theta_7/2)*cos(theta_2/2)*cos(-theta_4/2 + theta_5/2 + theta_6/2)/2 + sin(-theta_1/2 + theta_4/2 + theta_5/2 + theta_6/2 - theta_7/2)*cos(theta_2/2 - theta_3/2)/4 - sin(theta_1/2 - theta_2/2 + theta_3/2 - theta_4/2 + theta_7/2)*cos(theta_5/2 - theta_6/2)/4 + sin(theta_1/2 - theta_2/2 + theta_4/2 + theta_6/2 + theta_7/2)*cos(theta_3/2)*cos(theta_5/2)/2 - sin(theta_1/2 + theta_2/2 + theta_3/2 + theta_5/2 - theta_6/2 + theta_7/2)*cos(theta_4/2)/4;

    // velocities
    hN << theta_dot_1;
    hN << theta_dot_2;
    hN << theta_dot_3;
    hN << theta_dot_4;
    hN << theta_dot_5;
    hN << theta_dot_6;
    hN << theta_dot_7;
    
     // weights
    DMatrix Q(21,21);
    Q(0,0) = 1.0; // x
    Q(1,1) = 1.0; // y
    Q(2,2) = 1.0; // z
    Q(3,3) = 1.0; // qw
    Q(4,4) = 1.0; // qx
    Q(5,5) = 1.0; // qy
    Q(6,6) = 1.0; // qz
    Q(7,7) = 1.0e-3; // velocity weight starts here
    Q(8,8) = 1.0e-3;
    Q(9,9) = 1.0e-3;
    Q(10,10) = 1.0e-3;
    Q(11,11) = 1.0e-3;
    Q(12,12) = 1.0e-3;
    Q(13,13) = 1.0e-3;
    Q(14,14) = 1.0e-3;// acceleration weights starts here
    Q(15,15) = 1.0e-3;
    Q(16,16) = 1.0e-3;
    Q(17,17) = 1.0e-3;
    Q(18,18) = 1.0e-3;
    Q(19,19) = 1.0e-3;
    Q(20,20) = 1.0e-3;


    DMatrix QN(14,14);
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
    QN(13,13) = 1.0e-8;
    
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
    
    OCP ocp( tStart, tEnd, 5 );
    
    ocp.minimizeLSQ( Q, h );
    ocp.minimizeLSQEndTerm( QN, hN);
    
    ocp.subjectTo( f );

    // theta limits
    ocp.subjectTo( -2.3093 <= theta_1 <= 2.3093 );
    ocp.subjectTo( -1.5133 <= theta_2 <= 1.5133 );
    ocp.subjectTo( -2.4937 <= theta_3 <= 2.4937 );
    ocp.subjectTo( -2.7478 <= theta_4 <= -0.4461 );
    ocp.subjectTo( -2.4800 <= theta_5 <= 2.4800 );
    ocp.subjectTo(  0.8521 <= theta_6 <= 4.2094 );
    ocp.subjectTo( -2.6895 <= theta_7 <= 2.6895 );

    // // theta_dot limits
    ocp.subjectTo( -2.00 <= theta_dot_1 <= 2.00 );
    ocp.subjectTo( -1.00 <= theta_dot_2 <= 1.00 );
    ocp.subjectTo( -1.50 <= theta_dot_3 <= 1.50 );
    ocp.subjectTo( -1.25 <= theta_dot_4 <= 1.25 );
    ocp.subjectTo( -3.00 <= theta_dot_5 <= 3.00 );
    ocp.subjectTo( -1.50 <= theta_dot_6 <= 1.50 );
    ocp.subjectTo( -3.00 <= theta_dot_7 <= 3.00 );

    // // theta_dot limits
    // ocp.subjectTo( -20.00 <= theta_dot_1 <= 20.00 );
    // ocp.subjectTo( -10.00 <= theta_dot_2 <= 10.00 );
    // ocp.subjectTo( -10.50 <= theta_dot_3 <= 10.50 );
    // ocp.subjectTo( -10.25 <= theta_dot_4 <= 10.25 );
    // ocp.subjectTo( -30.00 <= theta_dot_5 <= 30.00 );
    // ocp.subjectTo( -10.50 <= theta_dot_6 <= 10.50 );
    // ocp.subjectTo( -30.00 <= theta_dot_7 <= 30.00 );
    
    // theta_ddot limits
    ocp.subjectTo( -15.0 <= u_1 <= 5.00 );
    ocp.subjectTo( -7.50 <= u_2 <= 5.00 );
    ocp.subjectTo( -10.0 <= u_3 <= 5.00 );
    ocp.subjectTo( -12.5 <= u_4 <= 5.00 );
    ocp.subjectTo( -15.0 <= u_5 <= 5.00 );
    ocp.subjectTo( -20.0 <= u_6 <= 5.00 );
    ocp.subjectTo( -20.0 <= u_6 <= 5.00 );

    // // theta_ddot limits
    // ocp.subjectTo( -150.0 <= u_1 <= 50.00 );
    // ocp.subjectTo( -70.50 <= u_2 <= 50.00 );
    // ocp.subjectTo( -100.0 <= u_3 <= 50.00 );
    // ocp.subjectTo( -120.5 <= u_4 <= 50.00 );
    // ocp.subjectTo( -150.0 <= u_5 <= 50.00 );
    // ocp.subjectTo( -200.0 <= u_6 <= 50.00 );
    // ocp.subjectTo( -200.0 <= u_6 <= 50.00 );


    // Export the code:
    OCPexport mpc( ocp );


    mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
    mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );
    mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
    mpc.set( NUM_INTEGRATOR_STEPS,        30        );
    mpc.set( QP_SOLVER,                   QP_QPOASES      );
    mpc.set( GENERATE_TEST_FILE,          YES             );
    mpc.set( GENERATE_MAKE_FILE,          YES             );


    if (mpc.exportCode( "mpc_export_panda_simple" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );


    mpc.printDimensionsQP( );
    
    return EXIT_SUCCESS;
}
