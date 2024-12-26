// example.c
#include <stdlib.h>

double* process_array(const double* input, size_t size, double scale) {
    double* output = (double*)malloc(size * sizeof(double));
    if (!output) {
        return NULL; // Handle memory allocation failure
    }
    for (size_t i = 0; i < size; i++) {
        output[i] = input[i] * scale; // Scale each element
    }
    return output;
}

void free_array(double* array) {
    free(array); // Free memory allocated for the output array
}
