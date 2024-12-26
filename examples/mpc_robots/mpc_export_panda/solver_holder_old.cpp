
/*

Modified by Tasbolat

*/

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <stdio.h>
/* HI */
/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   5        /* Number of real-time iterations. */
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
FILE *f;

/* Previous values for shifting */
real_t pre_u[NU * N];
real_t pre_x[NX * (N + 1)];

real_t te1, te2;

extern "C" __declspec(dllexport) double * solve_mpc(double currentState_targetValue[26]) {
	/* currentState_targetValue parsing:
	0 - 5: current theta
	6 - 11: current theta_dot
	12 - 14: current x y z
	15 - 18: current q0, q1, q2, q3
	19 - 21: target x y z
	22 - 25: target q0, q1, q2, q3

	output:

	*/

	if (started == 0) {
		/* Initialize the solver. */
		acado_initializeSolver();
		/* Initialize the states and controls. */
		for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[i] = 0.0; // change only first step
		for (i = 0; i < NU * N; ++i)  acadoVariables.u[i] = 0.0; // don't change!

		// thetas of x
		
		for (i = 0; i < NX * (N + 1); ++i) {
			int rem = i % 12;
			if (rem >= 0 && rem <= 5) {
				acadoVariables.x[i] = currentState_targetValue[rem];
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
		acadoVariables.x[i] = currentState_targetValue[i]; // only first step of states
	}



	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[i] = 0.0; // this is MUST be changed
	for (i = 0; i < NYN; ++i)  acadoVariables.yN[i] = 0.0; // this is final node, leave as zeros

	/* extrapolation */
	// extrapolation for x y z and fill first row
	double diff_of_value[3] = { 0.0, 0.0, 0.0 };

	for (i = 0; i < 7; i++) {
		
		if(i < 3)
			diff_of_value[i] = currentState_targetValue[i+19] - currentState_targetValue[NX + i]; // find differences for xyz and quat
		
		acadoVariables.y[i] = currentState_targetValue[i+19]; // assign the r(t) to the target: we're at r(t-1)
	}

	// find delta_q = q(t-1).conj * q(t)
	double delta_q[4] = {0.0, 0.0, 0.0, 0.0};
	
	delta_q[0] = currentState_targetValue[22] * currentState_targetValue[15]\
		       + currentState_targetValue[23] * currentState_targetValue[16]\
		       + currentState_targetValue[24] * currentState_targetValue[17]\
		       + currentState_targetValue[25] * currentState_targetValue[18];

	delta_q[1] = currentState_targetValue[22] * currentState_targetValue[16]\
			   - currentState_targetValue[23] * currentState_targetValue[15]\
			   + currentState_targetValue[24] * currentState_targetValue[18]\
			   - currentState_targetValue[25] * currentState_targetValue[17];

	delta_q[2] = currentState_targetValue[22] * currentState_targetValue[17]\
		       - currentState_targetValue[23] * currentState_targetValue[18]\
		       - currentState_targetValue[24] * currentState_targetValue[15]\
		       + currentState_targetValue[25] * currentState_targetValue[16];
	
	delta_q[3] = currentState_targetValue[22] * currentState_targetValue[18]\
			   + currentState_targetValue[23] * currentState_targetValue[17]\
		       - currentState_targetValue[24] * currentState_targetValue[16]\
		       - currentState_targetValue[25] * currentState_targetValue[15];

	//n1 = r(1)*q(1) - r(2)*q(2) - r(3)*q(3) - r(4)*q(4);
	//n2 = r(1)*q(2) + r(2)*q(1) - r(3)*q(4) + r(4)*q(3);
	//n3 = r(1)*q(3) + r(2)*q(4) + r(3)*q(1) - r(4)*q(2);
	//n4 = r(1)*q(4) - r(2)*q(3) + r(3)*q(2) + r(4)*q(1);
    //currentState_targetValue[22] * delta_q;


	int count_step = 1; // each step
	for (i = NY; i < NY*N; i = i + NY) { // extrapolate
		acadoVariables.y[i] = currentState_targetValue[19] + count_step*diff_of_value[0];
		acadoVariables.y[i + 1] = currentState_targetValue[20] + count_step*diff_of_value[1];
		acadoVariables.y[i + 2] = currentState_targetValue[21] + count_step*diff_of_value[2];
		acadoVariables.y[i + 3] = delta_q[0] * acadoVariables.y[i - 16]\
								- delta_q[1] * acadoVariables.y[i - 15]\
								- delta_q[2] * acadoVariables.y[i - 14]\
								- delta_q[3] * acadoVariables.y[i - 13];
		acadoVariables.y[i + 4] = delta_q[0] * acadoVariables.y[i - 15]\
								+ delta_q[1] * acadoVariables.y[i - 16]\
								- delta_q[2] * acadoVariables.y[i - 13]\
								+ delta_q[3] * acadoVariables.y[i - 14];
		acadoVariables.y[i + 5] = delta_q[0] * acadoVariables.y[i - 14]\
								+ delta_q[1] * acadoVariables.y[i - 13]\
								+ delta_q[2] * acadoVariables.y[i - 16]\
								- delta_q[3] * acadoVariables.y[i - 15];
		acadoVariables.y[i + 6] = delta_q[0] * acadoVariables.y[i - 13]\
								- delta_q[1] * acadoVariables.y[i - 14]\
								+ delta_q[2] * acadoVariables.y[i - 15]\
								+ delta_q[3] * acadoVariables.y[i - 16];
		count_step++;
	}


	/* MPC: initialize the current state feedback. */
	for (i = 0; i < NX; ++i)
		acadoVariables.x0[i] = currentState_targetValue[i];


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
	double* speedAndAcceleration = new double[13];

	/* get velocity states */
	for (i = 0; i < NX; ++i) {
		//printf("\t%e", acadoVariables.x[NX + i]); // r(t): we're at r(t-1)
		speedAndAcceleration[i] = acadoVariables.x[NX + i];
	}
		

	/*get control variable*/
	
	double maxAcc = acadoVariables.u[0];
	if (acadoVariables.u[0] < 0) maxAcc *= -1.0;
	int indOfMax = 0;

	for (i = 1; i < NU; ++i) {

		double tempMaxAcc = acadoVariables.u[i];
		if (acadoVariables.u[i] < 0) tempMaxAcc *= -1.0;

		if (tempMaxAcc > maxAcc) {
			maxAcc = tempMaxAcc;
			indOfMax = i;
		}
	}
		
    // TASBOLAT
	speedAndAcceleration[12] = acadoVariables.u[NU + indOfMax];
	
	/* Save to the next iteration*/
	for (i = 0; i < NU * N; ++i)  pre_u[i] = acadoVariables.u[i]; // control value
	for (i = 0; i < NX * (N + 1); ++i)  pre_x[i] = acadoVariables.x[i]; // state value

	//printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);
	
	/* write into file*/
	fopen_s(&f, "data1.txt", "a");

	fprintf(f, "0"); // from unity (feedback): all input
	for (i = 0; i < 25; ++i) fprintf(f, ",%g ", currentState_targetValue[i]);
	fprintf(f, "\n");
	

	fprintf(f, "1"); // mpc theta, theta_dot
	for (i = 0; i < 12; ++i) fprintf(f, ",%g ", acadoVariables.x[NX + i]);
	fprintf(f, "\n");

	fprintf(f, "2"); // mpc theta_ddot
	for (i = 0; i < NU; ++i) fprintf(f, ",%g ", acadoVariables.u[i]);
	fprintf(f, "\n");

	fprintf(f, "3,"); // tic toc time
	fprintf(f, "%.3g, %.3g \n", 1e6 * te1 / NUM_STEPS, 1e6 * te2 / NUM_STEPS);
	fprintf(f, "\n");
	fclose(f);

	// RECORD theta_ddot
	 
	/*
	// for debugging purposes
	// write resulst into file
	FILE *f;
	fopen_s(&f, "randomFile.txt", "w");
	fprintf(f, "speedAndAcceleration:\n");
	for (i = 0; i < 12; ++i) fprintf(f, "\t%g ", speedAndAcceleration[i]);
	fprintf(f,"\n");

	fprintf(f, "real speed:\n");
	for (i = 6; i < 12; ++i) fprintf(f, "\t%g ", acadoVariables.x[NX + i]);
	fprintf(f, "\n");

	fprintf(f, "real acceleration:\n");
	for (i = 0; i < 6; ++i) fprintf(f, "\t%g ", acadoVariables.u[NU + i]);
	fprintf(f, "\n");

	fprintf(f, "received data:\n");
	for (i = 0; i < 26; ++i) fprintf(f, "\t%g ", currentState_targetValue[i]);
	fprintf(f, "\n\n\n\n");

	int j;
	fprintf(f, "\nDifferential variables:\n[\n");
	for (i = 0; i < N + 1; ++i)
	{
		for (j = 0; j < NX; ++j)
			fprintf(f, "\t%e", acadoVariables.x[i * NX + j]);
		fprintf(f, "\n");
	}
	fprintf(f, "]\n\n");

	fprintf(f,"\nControl variables:\n[\n");
	for (i = 0; i < N; ++i)
	{
		for (j = 0; j < NU; ++j)
			fprintf(f,"\t%e", acadoVariables.u[i * NU + j]);
		fprintf(f,"\n");
	}
	fprintf(f,"]\n\n\n\n");

	fprintf(f, "Linearization:\n");
	fprintf(f, "\t%e", currentState_targetValue[12]);
	for (i = 0; i < NY*N; i = i + NY)  fprintf(f, "\t%e", acadoVariables.y[i]);
	fprintf(f, "\n");
	fprintf(f, "\t%e", currentState_targetValue[13]);
	for (i = 0; i < NY*N; i = i + NY)  fprintf(f, "\t%e", acadoVariables.y[i + 1]);
	fprintf(f, "\n");
	fprintf(f, "\t%e", currentState_targetValue[14]);
	for (i = 0; i < NY*N; i = i + NY)  fprintf(f, "\t%e", acadoVariables.y[i + 2]);
	fprintf(f, "\n");
	fprintf(f, "\t%e", currentState_targetValue[15]);
	for (i = 0; i < NY*N; i = i + NY)  fprintf(f, "\t%e", acadoVariables.y[i + 3]);
	fprintf(f, "\n");
	fprintf(f, "\t%e", currentState_targetValue[16]);
	for (i = 0; i < NY*N; i = i + NY)  fprintf(f, "\t%e", acadoVariables.y[i + 4]);
	fprintf(f, "\n");
	fprintf(f, "\t%e", currentState_targetValue[17]);
	for (i = 0; i < NY*N; i = i + NY)  fprintf(f, "\t%e", acadoVariables.y[i + 5]);
	fprintf(f, "\n");
	fprintf(f, "\t%e", currentState_targetValue[18]);
	for (i = 0; i < NY*N; i = i + NY)  fprintf(f, "\t%e", acadoVariables.y[i + 6]);
	fprintf(f, "\n");

	fclose(f);*/

	return speedAndAcceleration;
}

extern "C" __declspec(dllexport) int ReleaseMemory(double* pArray) {
	delete[] pArray;
	return 0;
}