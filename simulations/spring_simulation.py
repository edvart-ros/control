from scipy import signal
import control
import numpy as np
import matplotlib.pyplot as plt
#Simulation of mass-spring-damper system, formulated in state space form and simulated with scipy and control

#Differential equation:
#mx'' = u - kx - cx'
m = 1
k = 0.3
c = 0.1

#State space form:
A = np.matrix([[0, 1],[-k/m, -c/m]])
B = 1/m
C = np.eye(2)
D = 0

sys = signal.StateSpace(A, B, C, D)
x0 = [0, 0]
t, y = signal.step(sys, x0)

#plot the step response
plt.plot(t, y)