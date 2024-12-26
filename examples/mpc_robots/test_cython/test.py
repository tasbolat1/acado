import numpy as np
from example_wrapper import process_array_wrapper

# Input array
input_array = np.array([1.0, 2.0, 3.0, 4.0], dtype=np.float64)

# Scaling factor
scale = 2.0

# Call the function
output_array = process_array_wrapper(input_array, scale)

print("Input Array:", input_array)
print("Output Array:", output_array)