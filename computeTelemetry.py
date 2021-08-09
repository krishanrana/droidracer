import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

def rot2eul(rot):
    eul = np.arctan2(rot[1][0],rot[0][0])
    return eul




data = np.loadtxt('./Telemetry/telemetry_Table.csv', delimiter=',', dtype = "float")

vCommand = data[:,0:3]
vEncoder = data[:,3:6]
vImu = data[:,6:9]
vTime = data[:,9]

t0 = vTime[0]
vTime = vTime - t0

# Apply filter
filtCoeff_A = np.array([1,1,1])
filtCoeff_B = np.subtract(np.array([1,1,1]),filtCoeff_A)

vEstR = [(filtCoeff_A *  vEncoder[0] + filtCoeff_B * vImu[0]).tolist()]
# vEstR = [(filtCoeff_A *  vEncoder[0] + filtCoeff_B * vCommand[0]).tolist()]
vEstN = [vCommand[0].tolist()]
pEstN = [[0,0,0]]
# Initialise rotation matrix
dT = 0
for idx in range(1,len(vTime)):

    #...........ATTITUDE AT TIME K.............
    # Fuse sensor readings
    vEstR_K = (filtCoeff_A * vEncoder[idx-1] + filtCoeff_B * vImu[idx-1]).tolist()
    # vEstR_K = vCommand[idx-1]
    # Assume that rotation is 2D and body frame inertial measurements == Nav frame 
    dT = vTime[idx] - vTime[idx-1]
    print('dT: {0:0.3f}'.format(dT))
    Omega = vEstR_K[2] # Last reported angular velocity from odometry filter
    print('Omega: {0:0.3f}'.format(Omega))
    ThetaK = pEstN[idx-1][2] + (Omega * dT)
    Cnr =np.array([[np.cos(ThetaK), -np.sin(ThetaK)],[np.sin(ThetaK),np.cos(ThetaK)]])

    #..........VELOCITY AT TIME K..................
    # Convert Velocity estimate to Nav frame
    nVel = np.dot(Cnr,vEstR_K[0:2])
    # Integrate velocity to position
    # Optional: Add GPS  / Abs position + filter here  
    nPos = ((dT * nVel) + pEstN[idx-1][0:2]).tolist()
    nPos.append(ThetaK)

    print('Position X: {0:0.3f} '.format(nPos[0]))
    print('Position Y: {0:0.3f} '.format(nPos[1]))
    print('Theta: {0:0.3f} '.format(nPos[2]))

    # Store velocity estimate in Nav frame
    nVec = (np.append(nVel,Omega)).tolist()
    vEstN.append(nVec)
    # Store velocity estimate in Robot frame
    vEstR.append(vEstR_K)
    # Store position estimate in Nav frame
    pEstN.append(nPos)
    
robotPos = np.array(pEstN)
robotVel = np.array(vEstR)
plt.plot(robotPos[:,0],robotPos[:,1],'r--')
plt.plot(robotPos[0,0],robotPos[0,1],'ko')
plt.text(robotPos[0,0],robotPos[0,1],'START',color='black')
plt.plot(robotPos[-1,0],robotPos[-1,1],'b*')
plt.text(robotPos[-1,0],robotPos[-1,1],'STOP',color='blue')

plt.xlabel('X axis - m')
plt.ylabel('Y axis -m')
plt.axis('equal')
# plt.show()

# Plot velocity from sensors
fig, ax = plt.subplots(1,3,figsize=(17,4))

ax[0].plot(vTime,vCommand[:,0],color='red')
ax[0].set_ylabel('X Velocity - m/s',labelpad= -4)
ax[0].plot(vTime,vEncoder[:,0],color='blue')
ax[0].plot(vTime,vImu[:,0],color='green')
ax[0].set_xlabel('Time - s')

ax[1].plot(vTime,vCommand[:,1],color='red')
ax[1].set_ylabel('Y Velocity - m/s',labelpad= -4)
ax[1].plot(vTime,vEncoder[:,1],color='blue')
ax[1].plot(vTime,vImu[:,1],color='green')
ax[1].set_xlabel('Time - s')

ax[2].plot(vTime,vCommand[:,2],color='red',label='Command')
ax[2].set_ylabel('Rotation rate - rad/s',labelpad= -4)
ax[2].plot(vTime,vEncoder[:,2],color='blue',label='Encoder')
ax[2].plot(vTime,vImu[:,2],color='green',label='IMU')
ax[2].plot(vTime,robotVel[:,2],color='black',label='FUSION')

ax[2].set_xlabel('Time - s')
fig.legend(loc="lower left", bbox_to_anchor=(3.4,0.75), bbox_transform=ax[0].transAxes)

plt.show()

