

from libc.stdlib cimport free
from libc.stddef cimport size_t

cdef extern from "solver_holder.h":
    double* solve_mpc(const double* current_states,
                      const double* current_position,
                      const double* current_quaternion,
                      const double* target_position,
                      const double* target_quaternion)
    void ReleaseMemory(double* array)

import numpy as np
cimport numpy as np

def solve_mpc_wrapper(np.ndarray[double, ndim=1] current_states,
                          np.ndarray[double, ndim=1] current_position,
                          np.ndarray[double, ndim=1] current_quaternion,
                          np.ndarray[double, ndim=1] target_position,
                          np.ndarray[double, ndim=1] target_quaternion):
    
    print('pyx -------')
    print('states: ', current_states)
    print("current posit:", current_position)
    print("current quat:", current_quaternion)
    print("target posit:", target_position)
    print("target quat:", current_quaternion)
    
    
    cdef double* results_c = solve_mpc(<const double*>current_states.data,
                                      <const double*>current_position.data,
                                      <const double*>current_quaternion.data,
                                      <const double*>target_position.data,
                                      <const double*>target_quaternion.data)

    if results_c == NULL:
        raise MemoryError("Failed to allocate memory in process_array")
    
    cdef size_t size = current_states.shape[0] 
    # Convert the C array to a NumPy array
    result_np = np.empty(size, dtype=np.float64)
    for i in range(size):
        result_np[i] = results_c[i]
    
    # Free the allocated memory in C
    ReleaseMemory(results_c)
    
    return result_np
