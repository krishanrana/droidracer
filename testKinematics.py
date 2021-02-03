import math
import numpy as np
import time

theta = np.pi *3
print(theta)

t0 = time.time()
B = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])

X = np.array([10,1]).T

A = np.dot(B,X)
print(A)
print('Trig:',time.time() - t0)
# Using Tangent half angle substitution
t0 = time.time()

t = np.tan(theta/2)
Bt = np.array([[(1-t**2)/(1+t**2), -2*t/(1+t**2)],[2*t/(1+t**2), (1-t**2)/(1+t**2)]])
At = np.dot(Bt,X)
print(At)
print('Sub:',time.time() - t0)