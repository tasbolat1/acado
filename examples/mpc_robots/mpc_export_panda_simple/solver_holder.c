
/*

Modified by Tasbolat

*/


#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include "quaternion.h"

#include <stdio.h>
#include <stdlib.h>  // Required for EXIT_FAILURE and exit()

/* HI */
/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;


/* Some temporary variables. */
int    i, iter;
acado_timer t1, t2;

/* flag for start*/
int started = 0;

/*for File */
// FILE *f;

/* Previous values for shifting */
real_t pre_u[NU * N];
real_t pre_x[NX * (N + 1)];

real_t te1, te2;

double* solve_mpc(const double current_states[14], 
				  const double current_position[3], const double current_quaternion[4],
				  const double target_position[3], const double target_quaternion[4]) {
	/* currentState_targetValue parsing:
	0 - 6: current theta
	7 - 13: current theta_dot
	14 - 16: current x y z
	17 - 20: current q0, q1, q2, q3
	21 - 23: target x y z
	24 - 27: target q0, q1, q2, q3

	output:

	*/

	printf("solver_holder.c -------- \n");
	printf("current states: %0.3f, %0.3f, %0.3f, %0.3f\n", current_states[0], current_states[1], current_states[12], current_states[13]);
	printf("current posit: %0.3f, %0.3f, %0.3f\n", current_position[0], current_position[1], current_position[2]);
	printf("current quat: %0.3f, %0.3f, %0.3f, %0.3f\n", current_quaternion[0], current_quaternion[1], current_quaternion[2], current_quaternion[3]);
	printf("target posit: %0.3f, %0.3f, %0.3f\n", target_position[0], target_position[1], target_position[2]);
	printf("target quat: %0.3f, %0.3f, %0.3f, %0.3f\n", target_quaternion[0], target_quaternion[1], target_quaternion[2], target_quaternion[3]);



	if (started == 0) {
		/* Initialize the solver. */
		acado_initializeSolver();
		/* Initialize the states and controls. */
		for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[i] = 0.0; // change only first step
		for (i = 0; i < NU * N; ++i)  acadoVariables.u[i] = 0.0; // don't change!

		// fill current thetas of x
		for (i = 0; i < NX * (N + 1); ++i) {
			int rem = i % NX;
			if (rem >= 0 && rem <= (int)(NX/2)) {
				acadoVariables.x[i] = current_states[rem];
			}
		}

		started = 1;
	}
	else {
		/* New values */
		for (i = 0; i < NU * (N-1); ++i)  acadoVariables.u[i] = pre_u[i+NU]; // 1:54 = 7:60
		for (i = NU * (N - 1); i < NU * N; ++i)  acadoVariables.u[i] = pre_u[i]; // 55:60 = 55:60

		for (i = NX; i < NX * N; ++i)  acadoVariables.x[i] = pre_x[i+NX]; // 13:120 = 25:132
		for (i = NX * N; i < NX * (N+1); ++i)  acadoVariables.x[i] = pre_x[i]; // 120:132 = 120:132
	}
	

	// assign ---> CHECK: it's measurement
	for (i = 0; i < NX; ++i) {
		acadoVariables.x[i] = current_states[i]; // only first step of states
	}



	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[i] = 0.0; // this is MUST be changed

	/* extrapolation */
	// extrapolation for x y z and fill first row
	double diff_of_value[3] = { 0.0, 0.0, 0.0 };

	for (i = 0; i < 7; i++) {
		
		if(i < 3) {
			diff_of_value[i] = target_position[i] - current_position[i]; // find differences for xyz and quat
			acadoVariables.y[i] = current_position[i]; // assign the r(t) to the target: we're at r(t-1)
		}		  
		else {
			acadoVariables.y[i] = current_quaternion[i-3];
		}
	}


	// double delta_q[4] = {0.0, 0.0, 0.0, 0.0};

	// delta_q[0] = target_quaternion[0] * current_quaternion[0] \
    //         + target_quaternion[1] * current_quaternion[1]\
    //         + target_quaternion[2] * current_quaternion[2]\
    //         + target_quaternion[3] * current_quaternion[3];

	// delta_q[1] = target_quaternion[0] * current_quaternion[1]\
	// 			- target_quaternion[1] * current_quaternion[0]\
	// 			- target_quaternion[2] * current_quaternion[3]\
	// 			+ target_quaternion[3] * current_quaternion[2];

	// delta_q[2] = target_quaternion[0] * current_quaternion[2]\
	// 			+ target_quaternion[1] * current_quaternion[3]\
	// 			- target_quaternion[2] * current_quaternion[0]\
	// 			- target_quaternion[3] * current_quaternion[1];

	// delta_q[3] = target_quaternion[0] * current_quaternion[3]\
	// 			- target_quaternion[1] * current_quaternion[2]\
	// 			+ target_quaternion[2] * current_quaternion[1]\
	// 			- target_quaternion[3] * current_quaternion[0];
	
	
	//n1 = r(1)*q(1) - r(2)*q(2) - r(3)*q(3) - r(4)*q(4);
	//n2 = r(1)*q(2) + r(2)*q(1) - r(3)*q(4) + r(4)*q(3);
	//n3 = r(1)*q(3) + r(2)*q(4) + r(3)*q(1) - r(4)*q(2);
	//n4 = r(1)*q(4) - r(2)*q(3) + r(3)*q(2) + r(4)*q(1);
    //currentState_targetValue[22] * delta_q;

	printf("pos diff: %0.3f, %0.3f, %0.3f\n", diff_of_value[0], diff_of_value[1], diff_of_value[2]);
	// printf("orient diff: %0.3f, %0.3f, %0.3f, %0.3f\n", delta_q[0], delta_q[1], delta_q[2], delta_q[3]);

	double theta_angle = compute_angle_q1_to_q2(current_quaternion, target_quaternion);

	printf("orient diff: %0.3f \n", theta_angle);

	double int_quats[4];

	printf("printing future steps:\n");
	double step_size = 1.0/(N-1); // each step
	double current_step = step_size;
	for (i = NY; i < NY*N; i = i + NY) { // extrapolate
		acadoVariables.y[i] = current_position[0] + current_step*diff_of_value[0];
		acadoVariables.y[i + 1] = current_position[1] + current_step*diff_of_value[1];
		acadoVariables.y[i + 2] = current_position[2] + current_step*diff_of_value[2];
		// acadoVariables.y[i + 3] = delta_q[0] * acadoVariables.y[i - 16]\
		// 						- delta_q[1] * acadoVariables.y[i - 15]\
		// 						- delta_q[2] * acadoVariables.y[i - 14]\
		// 						- delta_q[3] * acadoVariables.y[i - 13];
		// acadoVariables.y[i + 4] = delta_q[0] * acadoVariables.y[i - 15]\
		// 						+ delta_q[1] * acadoVariables.y[i - 16]\
		// 						- delta_q[2] * acadoVariables.y[i - 13]\
		// 						+ delta_q[3] * acadoVariables.y[i - 14];
		// acadoVariables.y[i + 5] = delta_q[0] * acadoVariables.y[i - 14]\
		// 						+ delta_q[1] * acadoVariables.y[i - 13]\
		// 						+ delta_q[2] * acadoVariables.y[i - 16]\
		// 						- delta_q[3] * acadoVariables.y[i - 15];
		// acadoVariables.y[i + 6] = delta_q[0] * acadoVariables.y[i - 13]\
		// 						- delta_q[1] * acadoVariables.y[i - 14]\
		// 						+ delta_q[2] * acadoVariables.y[i - 15]\
		// 						+ delta_q[3] * acadoVariables.y[i - 16];

		
		slerp(current_quaternion, target_quaternion, current_step, theta_angle, int_quats);
		acadoVariables.y[i + 3] = int_quats[0];
		acadoVariables.y[i + 4] = int_quats[1];
		acadoVariables.y[i + 5] = int_quats[2];
		acadoVariables.y[i + 6] = int_quats[3];

		printf("%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f:\n", 
		acadoVariables.y[i], acadoVariables.y[i+1],acadoVariables.y[i+2],
		acadoVariables.y[i+3], acadoVariables.y[i+4],acadoVariables.y[i+5],acadoVariables.y[i+6]);
		printf("current_step:%0.3f\n", current_step);
		// printf("%0.3f, %0.3f, %0.3f:\n", 
		// acadoVariables.y[i], acadoVariables.y[i+1],acadoVariables.y[i+2]);
		current_step += step_size;
	}

	// this seems to work
	// for (i = 0; i < NYN; ++i)  acadoVariables.yN[i] = 0.0; // this is final node, leave as zeros
	for (i = 0; i < NYN; ++i)  acadoVariables.yN[i] = acadoVariables.y[NY*N-NYN+i]; // this is final node, leave as zeros



	/* MPC: initialize the current state feedback. */
	for (i = 0; i < NX; ++i)
		acadoVariables.x0[i] = current_states[i];

	/* The "real-time iterations" loop. */
	for (iter = 0; iter < NUM_STEPS; ++iter)
	{
		/* time for preparationStep */
		acado_tic(&t1);

		/* Prepare first step */
		acado_preparationStep();

		te1 = acado_toc(&t1);

		/* time for feedbackStep */
		acado_tic(&t2);

		/* Perform the feedback step. */
		acado_feedbackStep();

		te2 = acado_toc(&t2);
	}
	
	/* Return our result */

	double* next_state = (double*)malloc(NX * sizeof(double));  // Allocate on the heap
    if (next_state == NULL) {
        perror("Failed to allocate memory");
        exit(EXIT_FAILURE);
    }

	/* get velocity states */
	for (i = 0; i < NX; ++i) {
		//printf("\t%e", acadoVariables.x[NX + i]); // r(t): we're at r(t-1)
		next_state[i] = acadoVariables.x[NX + i];
	}
		
    // TASBOLAT
	// speedAndAcceleration[14] = acadoVariables.u[NU + indOfMax];
	
	/* Save to the next iteration*/
	for (i = 0; i < NU * N; ++i)  pre_u[i] = acadoVariables.u[i]; // control value
	for (i = 0; i < NX * (N + 1); ++i)  pre_x[i] = acadoVariables.x[i]; // state value

	// real_t te = te1 + te2;
	// printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te );

	return next_state;
}


void ReleaseMemory(double* array) {
	free(array);
}
