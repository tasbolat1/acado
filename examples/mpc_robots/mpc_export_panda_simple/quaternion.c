// quaternion.c
#include "quaternion.h"
#include <stdio.h>

// Compute the dot product of two quaternions
double quaternion_dot(const double q1[4], const double q2[4]) {
    return q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
}

// Normalize a quaternion
void normalize_quaternion(double q[4]) {
    double magnitude = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int i = 0; i < 4; i++) {
        q[i] /= magnitude;
    }
}


double compute_angle_q1_to_q2(const double q1[4], const double q2[4]) {
    
    double q1_copy[4] = {q1[0], q1[1], q1[2], q1[3]};
    double q2_copy[4] = {q2[0], q2[1], q2[2], q2[3]};

    // Normalize input quaternions
    normalize_quaternion(q1_copy);
    normalize_quaternion(q2_copy);

    // Compute the dot product
    double dot = quaternion_dot(q1_copy, q2_copy);

    // Handle the shortest path
    if (dot < 0.0) {
        dot = -dot;
        for (int i = 0; i < 4; i++) {
            q2_copy[i] = -q2_copy[i];
        }
    }

    // Clamp dot product to avoid numerical issues
    dot = fmin(fmax(dot, -1.0), 1.0);

    // Compute the angle between the quaternions
    double theta = acos(dot);

    return theta;   
}

// Perform spherical linear interpolation (SLERP) between two quaternions
void slerp(const double q1[4], const double q2[4], double t, const double theta, double result[4]) {
    
    double q1_copy[4] = {q1[0], q1[1], q1[2], q1[3]};
    double q2_copy[4] = {q2[0], q2[1], q2[2], q2[3]};

    // printf("inside the slerp:\n");
    // printf("q1: %0.3f, %0.3f, %0.3f, %0.3f:\n", q1_copy[0], q1_copy[1], q1_copy[2], q1_copy[3]);
    // printf("q2: %0.3f, %0.3f, %0.3f, %0.3f:\n", q2_copy[0], q2_copy[1], q2_copy[2], q2_copy[3]);

    // If the angle is very small, use linear interpolation
    if (fabs(theta) < 1e-6) {
        for (int i = 0; i < 4; i++) {
            result[i] = (1.0 - t) * q1_copy[i] + t * q2_copy[i];
        }
        return;
    }

    // Perform SLERP
    double sin_theta = sin(theta);
    double scale_1 = sin((1.0 - t) * theta) / sin_theta;
    double scale_2 = sin(t * theta) / sin_theta;

    for (int i = 0; i < 4; i++) {
        result[i] = scale_1 * q1_copy[i] + scale_2 * q2_copy[i];
    }

    // printf("q_res: %0.3f, %0.3f, %0.3f, %0.3f:\n", result[0], result[1], result[2], result[3]);


}
