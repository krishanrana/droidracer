# Derivation of droid kinematics from:
# Yunardi et al (2021), Journal of Robotics and Control, Vol 2, Issue 2
# DOI: 10.18196/jrc.2254

# NOTE: This article is erroneous in its matrix formulation, published matrices should be transposed.

import numpy as np

deg2rad = np.pi/180
droidRadius = 0.150

# A1 =  deg2rad * 30
# A2 =  deg2rad * 150
# A3 =  deg2rad * 270
# b = deg2rad * 90

# Minv1 = np.array([[np.cos(A1 + b), np.cos(A2 + b),np.cos(A3 + b)],
#      [-np.sin(A1 + b), -np.sin(A2 + b),np.sin(A3 + b)],
#      [1          ,1           ,1          ]])
# Minv1rot = np.matrix.transpose(Minv1)

#-----------------------------------
# Forward Kinematics: Motor speeds to droid velocity

# Angle between motor axis and CS frame (90deg)
Minv = np.array([[-0.5,-0.866  , 1],
                 [-0.5, 0.866, 1],
                [1    , 0    , 1]])
M = np.linalg.inv(Minv)

m1 = 0.005
m2 = 0.005
m3 = 0.005

droidVel = M @ np.array([m1,m2,m3])
droidVel[2] = droidVel[2] /(2*droidRadius)

print('Velocity x: {0:0.3f}, y: {1:0.3f}, Omega: {2:0.3f}'.format(droidVel[0],droidVel[1],droidVel[2] *180/np.pi))
print(np.round(M,decimals=3))
 # Inverse Kinematics: Droid velocity to motor speeds
vX = 0
vY = 0
omega = -0.917

vEst = np.array([vX,vY,omega])
vEst[2] = vEst[2] * droidRadius * 2
motors = Minv @ np.array(vEst)

print('Motors 1: {0:0.3f}, 2: {1:0.3f}, 3: {2:0.3f}'.format(motors[0],motors[1],motors[2]))
print(np.round(Minv,decimals=3))


      