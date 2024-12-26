// quaternion.h
#ifndef QUATERNION_H
#define QUATERNION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

// Function declarations
double quaternion_dot(const double q1[4], const double q2[4]);
double compute_angle_q1_to_q2(const double q1[4], const double q2[4]);
void normalize_quaternion(double q[4]);
void slerp(const double q1[4], const double q2[4], double t, const double theta, double result[4]);

#ifdef __cplusplus
}
#endif

#endif // QUATERNION_H
