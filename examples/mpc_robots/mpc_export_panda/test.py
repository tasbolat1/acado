import numpy as np
from solver_holder_wrapper import solve_mpc_wrapper

import time

def reverse_quat(q):
    new_q = np.zeros_like(q)
    new_q[0] = q[1]
    new_q[1] = q[2]
    new_q[2] = q[3]
    new_q[3] = q[0]
    return new_q

current_states = np.zeros(14)

p = np.array([0.088, -7.148712732885087e-13, 0.9259999999999999])
q = np.array([1.0, 0.0, 0.0, 4.896583138958022e-12])


p_target = np.array([3.06892058e-01, 1.80330352e-07, 5.90325177e-01])
quat_target = np.array([9.23879903e-01, -3.82682536e-01, 3.99066510e-05 -1.52781514e-05])


q = reverse_quat(q)
quat_target = reverse_quat(q)

start_time = time.time()
res = solve_mpc_wrapper(current_states, p, q, p_target, quat_target)
print(time.time()-start_time)

print(res)