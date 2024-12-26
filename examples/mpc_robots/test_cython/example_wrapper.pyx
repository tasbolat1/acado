# example.pyx
from libc.stdlib cimport free
from libc.stddef cimport size_t
cdef extern from "example.h":
    double* process_array(const double* input, size_t size, double scale)
    void free_array(double* array)

import numpy as np
cimport numpy as np

def process_array_wrapper(np.ndarray[double, ndim=1] input_array, double scale):
    cdef size_t size = input_array.shape[0]
    cdef double* output_c = process_array(<const double*>input_array.data, size, scale)
    
    if output_c == NULL:
        raise MemoryError("Failed to allocate memory in process_array")
    
    # Convert the C array to a NumPy array
    output_array = np.empty(size, dtype=np.float64)
    for i in range(size):
        output_array[i] = output_c[i]
    
    # Free the allocated memory in C
    free_array(output_c)
    
    return output_array
