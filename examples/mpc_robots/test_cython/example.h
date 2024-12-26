// example.h
#ifndef EXAMPLE_H
#define EXAMPLE_H

#include <stddef.h>

double* process_array(const double* input, size_t size, double scale);
void free_array(double* array);

#endif
