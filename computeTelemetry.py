import numpy as np
import matplotlib.pyplot as plt


data = np.loadtxt('telemetry.csv', delimiter=',', dtype = "float")

vControl = data[:,0:3]
vEncoder = data[:,3:6]
vImu = data[:,6:9]
vTime = data[:,9]

t0 = vTime[0]
vTime = vTime - t0

# Plot velocity from sensors
fig, ax = plt.subplots(1,3,figsize=(17,4))

ax[0].plot(vTime,vControl[:,0],color='red')
ax[0].set_ylabel('X Velocity - m/s',labelpad= -4)
ax[0].plot(vTime,vEncoder[:,0],color='blue')
ax[0].plot(vTime,vImu[:,0],color='green')
ax[0].set_xlabel('Time - s')

ax[1].plot(vTime,vControl[:,1],color='red')
ax[1].set_ylabel('Y Velocity - m/s',labelpad= -4)
ax[1].plot(vTime,vEncoder[:,1],color='blue')
ax[1].plot(vTime,vImu[:,1],color='green')
ax[1].set_xlabel('Time - s')

ax[2].plot(vTime,vControl[:,2],color='red',label='Command')
ax[2].set_ylabel('Rotation rate - rad/s',labelpad= -4)
ax[2].plot(vTime,vEncoder[:,2],color='blue',label='Encoder')
ax[2].plot(vTime,vImu[:,2],color='green',label='IMU')
ax[2].set_xlabel('Time - s')
fig.legend(loc="lower left", bbox_to_anchor=(3.4,0.75), bbox_transform=ax[0].transAxes)

plt.show()


