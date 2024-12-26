// example.h
#ifndef SOLVER_HOLDER_H
#define SOLVER_HOLDER_H

// double* solve_mpc(const double current_states[14],const  double current_position[3],const  double current_quaternion[4],const double target_position[3],const double target_quaternion[4]);



#ifdef __cplusplus
extern "C" {
#endif

double* solve_mpc(const double current_states[14], 
                  const double current_position[3], 
                  const double current_quaternion[4],
                  const double target_position[3], 
                  const double target_quaternion[4]);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
extern "C" {
#endif

void ReleaseMemory(double* array);

#ifdef __cplusplus
}
#endif

#endif // SOLVER_HOLDER_H
