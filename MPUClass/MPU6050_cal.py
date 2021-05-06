from MPU6050 import MPU6050
from SimplePID import SimplePID
import time
import numpy as np
import matplotlib.pyplot as plt

try:
    
    from MPUClass.MPUConstants import MPUConstants as C
    
except:
    from MPUConstants import MPUConstants as C
    



def avg_from_array(a_array):
    sum = 0.0
    for index in range(0, len(a_array)):
        sum += a_array[index]

    return sum/len(a_array)


i2c_bus = 1
device_address = 0x68
# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
x_accel_offset = 0
y_accel_offset = 0
z_accel_offset = 0
x_gyro_offset = 0
y_gyro_offset = 0
z_gyro_offset = 0
enable_debug_output = True

mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)
# Set IMU sensitivity from MPUConstants.py

mpu.set_full_scale_accel_range(C.MPU6050_ACCEL_FS_2) 
mpu.set_full_scale_gyro_range(C.MPU6050_GYRO_FS_250)
mpu.set_rate(9)
             

kp = 0.03125
ki = 0.25
kd = 0

delay = 20
pidax = SimplePID(0, -15000, 15000, kp, ki, kd, delay, True)
piday = SimplePID(0, -15000, 15000, kp, ki, kd, delay, True)
pidaz = SimplePID(0, -15000, 15000, kp, ki, kd, delay, True)
pidgx = SimplePID(0, -15000, 15000, kp, ki, kd, delay, True)
pidgy = SimplePID(0, -15000, 15000, kp, ki, kd, delay, True)
pidgz = SimplePID(0, -15000, 15000, kp, ki, kd, delay, True)

accel_reading = mpu.get_acceleration()

x_accel_reading = accel_reading[0]
y_accel_reading = accel_reading[1]
z_accel_reading = accel_reading[2]

nAverage = 20

x_accel_avg = [0]*nAverage
y_accel_avg = [0]*nAverage
z_accel_avg = [0]*nAverage

x_accel_offset_avg = [0]*nAverage
y_accel_offset_avg = [0]*nAverage
z_accel_offset_avg = [0]*nAverage

axindex = 0
ayindex = 0
azindex = 0

gyro_reading = mpu.get_rotation()

x_gyro_reading = gyro_reading[0]
y_gyro_reading = gyro_reading[1]
z_gyro_reading = gyro_reading[2]

x_gyro_avg = [0]*nAverage
y_gyro_avg = [0]*nAverage
z_gyro_avg = [0]*nAverage

x_gyro_offset_avg = [0]*nAverage
y_gyro_offset_avg = [0]*nAverage
z_gyro_offset_avg = [0]*nAverage

gxindex = 0
gyindex = 0
gzindex = 0

xAcOffHist = []
yAcOffHist = []
zAcOffHist = []
xGyOffHist = []
yGyOffHist = []
zGyOffHist = []

xAcHist = []
yAcHist = []
zAcHist = []
xGyHist = []
yGyHist = []
zGyHist = []

accelError = 5
gyroError = 5
Error = np.abs(np.array(accel_reading + gyro_reading))
allowableError = np.array([accelError,accelError,accelError,gyroError,gyroError,gyroError])

runIDX = 0
t0 = time.time()
print('Initializing')
try:
    while np.any(Error > allowableError): 
        
        accel_reading = mpu.get_acceleration()
        x_accel_reading = accel_reading[0]
        y_accel_reading = accel_reading[1]
        z_accel_reading = accel_reading[2]

        gyro_reading = mpu.get_rotation()
        x_gyro_reading = gyro_reading[0]
        y_gyro_reading = gyro_reading[1]
        z_gyro_reading = gyro_reading[2]
        
        
        # For each valid reading (>delay ms)
        if pidax.check_time():
            x_accel_offset = pidax.get_output_value(x_accel_reading)

            #mpu.set_x_accel_offset(int(x_accel_offset))
            x_accel_avg[axindex] = x_accel_reading
            x_accel_offset_avg[axindex] = x_accel_offset

            axindex += 1
            if axindex == len(x_accel_avg):
                axindex = 0
                xAcAvg = avg_from_array(x_accel_avg)
                xAcOffAvg = avg_from_array(x_accel_offset_avg)
                xAcOffHist.append(int(xAcOffAvg))
                xAcHist.append(int(xAcAvg))
                #Update Offset value 
                mpu.set_x_accel_offset(int(xAcOffAvg))
                
        if piday.check_time():
            y_accel_offset = piday.get_output_value(y_accel_reading)

            #mpu.set_y_accel_offset(int(y_accel_offset))

            y_accel_avg[ayindex] = y_accel_reading
            y_accel_offset_avg[ayindex] = y_accel_offset

            ayindex += 1
            if ayindex == len(y_accel_avg):
                ayindex = 0
                yAcAvg = avg_from_array(y_accel_avg)
                yAcOffAvg = avg_from_array(y_accel_offset_avg)
                yAcOffHist.append(int(yAcOffAvg))
                yAcHist.append(int(yAcAvg))

                #Update Offset value 
                mpu.set_y_accel_offset(int(yAcOffAvg))

        if pidaz.check_time():
            z_accel_offset = pidaz.get_output_value(z_accel_reading)

            #mpu.set_z_accel_offset(int(z_accel_offset))

            z_accel_avg[azindex] = z_accel_reading
            z_accel_offset_avg[azindex] = z_accel_offset

            azindex += 1
            if azindex == len(z_accel_avg):
                azindex = 0
                zAcAvg = avg_from_array(z_accel_avg)
                zAcOffAvg = avg_from_array(z_accel_offset_avg)
                zAcOffHist.append(int(zAcOffAvg))
                zAcHist.append(int(zAcAvg))

                #Update Offset value 
                mpu.set_z_accel_offset(int(zAcOffAvg))

        # Gyro calibration
        if pidgx.check_time():
            x_gyro_offset = pidgx.get_output_value(x_gyro_reading)

            x_gyro_avg[gxindex] = x_gyro_reading
            x_gyro_offset_avg[gxindex] = x_gyro_offset

            gxindex += 1
            if gxindex == len(x_gyro_avg):
                gxindex = 0
                xGyAvg = avg_from_array(x_gyro_avg)
                xGyOffAvg = avg_from_array(x_gyro_offset_avg)
                xGyOffHist.append(int(xGyOffAvg))
                xGyHist.append(int(xGyAvg))

                #Update Offset value 
                mpu.set_x_gyro_offset(int(xGyOffAvg))
                

        if pidgy.check_time():
            y_gyro_offset = pidgy.get_output_value(y_gyro_reading)

            y_gyro_avg[gyindex] = y_gyro_reading
            y_gyro_offset_avg[gyindex] = y_gyro_offset

            gyindex += 1
            if gyindex == len(y_gyro_avg):
                gyindex = 0
                yGyAvg = avg_from_array(y_gyro_avg)
                yGyOffAvg = avg_from_array(y_gyro_offset_avg)
                yGyOffHist.append(int(yGyOffAvg))
                yGyHist.append(int(yGyAvg))
                #Update Offset value 
                mpu.set_y_gyro_offset(int(yGyOffAvg))

        if pidgz.check_time():
            z_gyro_offset = pidgz.get_output_value(z_gyro_reading)

            z_gyro_avg[gzindex] = z_gyro_reading
            z_gyro_offset_avg[gzindex] = z_gyro_offset

            gzindex += 1
            if gzindex == len(z_gyro_avg):
                gzindex = 0
                zGyAvg = avg_from_array(z_gyro_avg)
                zGyOffAvg = avg_from_array(z_gyro_offset_avg)
                zGyOffHist.append(int(zGyOffAvg))
                zGyHist.append(int(zGyAvg))
                #Update Offset value 
                mpu.set_z_gyro_offset(int(zGyOffAvg))
        try:
            Error = np.abs(np.array([xAcAvg, yAcAvg, zAcAvg,xGyAvg, yGyAvg, zGyAvg ]))
            runIDX += 1
        except:  
            runIDX += 1
            
        if runIDX %1000 == 0:
            print(".",)               
                

except KeyboardInterrupt:
    pass




print('runIDX:'+ str(runIDX))  
print('elapsed time:'+ str(time.time()-t0)) 

print('X_avg_read: ' +
                      str(xAcAvg) +
                      ' X_avg_offset: ' +
                      str(xAcOffAvg))
print('Y_avg_read: ' +
                      str(yAcAvg) +
                      ' Y_avg_offset: ' +
                      str(yAcOffAvg))
print('Z_avg_read: ' +
                      str(zAcAvg) +
                      ' Z_avg_offset: ' +
                      str(zAcOffAvg)) 
print('x_avg_read: ' +
                      str(xGyAvg) +
                      ' x_avg_offset: ' +
                      str(xGyOffAvg))
print('y_avg_read: ' +
                      str(yGyAvg) +
                      ' y_avg_offset: ' +
                      str(yGyOffAvg))
print('z_avg_read: ' +
                      str(zGyAvg) +
                      ' z_avg_offset: ' +
                      str(zGyOffAvg))
fig = plt.figure()
ax = fig.add_subplot(1,1,1)                    
ax.plot(xAcOffHist) 
ax.plot(yAcOffHist)  
ax.plot(zAcOffHist)  
ax.plot(xGyOffHist)  
ax.plot(yGyOffHist)   
ax.plot(zGyOffHist) 

fig2 = plt.figure()
ax2 = fig2.add_subplot(1,1,1) 
ax2.plot(xAcHist) 
ax2.plot(yAcHist)  
ax2.plot(zAcHist)  
ax2.plot(xGyHist)  
ax2.plot(yGyHist)  
ax2.plot(zGyHist)  
plt.show() 
ImuOffsets = np.array([xAcOffAvg,yAcOffAvg,zAcOffAvg,xGyOffAvg,yGyOffAvg,zGyOffAvg ])
 
np.savetxt('../imuBiases.csv',ImuOffsets,delimiter=',')
print('IMU biases saved to file')              
