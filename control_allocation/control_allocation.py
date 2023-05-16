import cvxpy as cp
import numpy as np
import time

# define thruster positions relative to vessel frame
l1_x = -1
l1_y = -1

l2_x = -1
l2_y = 1

l3_x = 1
l3_y = -1

l4_x = 1
l4_y = 1

u_max = 50

# the problem is formulated in cartesian coordinates
# where u = [u_x ; u_y], which contains x and y force components of each
# thruster

t_d = np.array([0, 0, 40])  # desired generalized force
B = np.matrix(np.array([[  1,     0,     1,     0,     1,     0,     1,     0  ],  # thruster configuration matrix
                        [  0,     1,     0,     1,     0,     1,     0,     1  ],
                        [-l1_y,  l1_x, -l2_y,  l2_x, -l3_y,  l3_x, -l4_y,  l4_x]]))


A = np.array(np.hstack((B, np.eye(3))))  # reformulating the Tc - Bu = s constraint
b = cp.Parameter(3)
b.value = t_d #desired force


x = cp.Variable(11) # x = [u ; s]
P = np.array(np.vstack((np.hstack((np.eye(8), np.zeros((8, 3)))), np.hstack((np.zeros((3, 8)), 10000 * np.eye(3)))))) # weight matrix, weighting s heavily
prob = cp.Problem(cp.Minimize((1/2)*cp.quad_form(x, P)), [A @ x == b])

prob.solve()

#individual thruster forces
u = x.value[0:8]
u_1 = u[0:2]
u_2 = u[2:4]
u_3 = u[4:6]
u_4 = u[6:8]

print(u_1)
print(u_2)
print(u_3)
print(u_4)