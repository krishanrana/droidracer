# -*- coding: utf-8 -*-
"""
Created on Mon Jan 18 11:29:54 2021

@author: doug palmer

NOTE: Some code from MPU6050.py and iC2 library by Jeff Rowland via Geir Istad
"""
# Import packages
import sys

# Add path for modules, to be reorganised
sys.path.append('/home/pi/droidracer/MPUClass')

import numpy as np
from MPU6050 import MPU6050 # Rewrite class using native numpy
import logging
import time
import matplotlib.pyplot as plt


from DroidControl import droidControl

# Setup logging (Use droidlogging.conf as alternative)

# Log file location
logfile = 'debug.log'
# Define your own logger name
logger = logging.getLogger("IMUlogger")
# Set default logging level to DEBUG
logger.setLevel(logging.DEBUG)

# create console handler
print_format = logging.Formatter('%(levelname)-8s %(name)-12s %(message)s')
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.WARNING)
console_handler.setFormatter(print_format)

# create log file handler
# and define a custom log format, set its log level to DEBUG
log_format = logging.Formatter('[%(asctime)s] %(levelname)-8s %(name)-12s %(message)s')
file_handler = logging.FileHandler(logfile)
file_handler.setLevel(logging.DEBUG)
file_handler.setFormatter(log_format)

#Add handlers to the logger
logger.addHandler(file_handler)
logger.addHandler(console_handler)
            
                      


# Initialise class as Child of MPU6050
class droidInertial(MPU6050):
    
    def __init__(self):
        
        self.TimeKminus1 = time.time()        
        self.TimeK = time.time()
        self.gravity = 9.80665
        self.FIFO_buffer = [0]*42
        self.dmpAccel = np.array([0,0,0])
        self.rawAccel = np.array([0,0,0])
        self.velocity = np.array([0,0,0])
        self.displacement = np.array([0,0,0])
        self.accelBias = np.array([0,0,0])
        
        self.rawOmega = np.array([0,0,0])
        self.theta = np.array([0,0,0])
        self.omegaBias = np.array([0,0,0])
        
        self.imuOut = []
        
        
        debug_output = False
        i2c_bus = 1
        device_address = 0x68

        # Check for IMU bias file
        try:
            [x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset] = np.loadtxt('/home/pi/droidracer/MPUClass/imuBiases.csv', delimiter=',', dtype = "int")
            logger.info('Initialising IMU biases to file.') 
            # Use existing IMU bias estimates
            self.accelBias = np.array([[x_accel_offset, y_accel_offset,z_accel_offset]])
            self.omegaBias = np.array([[x_gyro_offset, y_gyro_offset, z_gyro_offset]])
        
        except:
            [x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset] = np.array([0,0,0,0,0,0]) # Placeholder, run cal routine
            logger.info('No Bias file found. Static Calibration required')

        # Check for IMU Offset file (Not yet implemented)
        try:
            self.imuOffset = np.loadtxt('/home/pi/droidracer/MPUClass/imuOffsets.csv', delimiter=',', dtype = "int")
            logger.info('Initialising IMU offsets to file.')
        except:
            self.imuOffset = np.array([0,0,0]) # Placeholder, run cal routine
            logger.warning('No Offset file found. Dynamic calibration required')
             
        # Setup thread safe timer
        
        # Initialise device using inheritance
        MPU6050.__init__(self, i2c_bus, device_address, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              debug_output)
              
        
        
        # Initialise DMP
        self.dmp_initialize()
#        # Acelerometer sensitivity set to 2g full scale
#        self.set_full_scale_accel_range(0x00) 
#        # Gyro set to 250 deg/sec full scale
#        self.set_full_scale_gyro_range(0x00)
#        # Set sample rate to 1000/(1 + rate)
#        self.set_rate(9)
#        #98Hz low pass filter
#        self.set_DLF_mode(0x02)
        # Start DMP
        self.set_DMP_enabled(True)
        self.FIFO_packet_size = self.DMP_get_FIFO_packet_size()
        
        logger.debug(('Interrupt status: %s ' % hex(self.get_int_status())))
        logger.debug('FIFO Packet size: %s ' % self.FIFO_packet_size)
        logger.debug(('FIFO count: %s ' % hex(self.get_FIFO_count())))
        
        
        
    def readIMUraw(self):
        self.TimeKminus1 = self.TimeK
        self.rawAccelMinus1 = self.rawAccel
        self.rawOmegaMinus1 = self.rawOmega
        self.TimeK = time.time()
        # Remove constant bias;
        # : Store as numpy arrays as we will be using matrix operations for Kalman filter
        self.rawAccel = (np.array([self.get_acceleration()]) * (self.gravity / 16384.0)) - self.accelBias
        self.rawOmega = (np.array([self.get_rotation()]) / 131.0) - self.omegaBias
        
        
    def readDMP(self):
        self.TimeKminus1 = self.TimeK
        FIFO_count = self.get_FIFO_count()
        mpu_int_status = self.get_int_status()
        
        while FIFO_count < self.FIFO_packet_size:
                FIFO_count = self.get_FIFO_count()
        # If overflow is detected by status or fifo count we want to reset
        if (FIFO_count == 1024) or (mpu_int_status & 0x10):
            self.reset_FIFO()
            logger.warning('FIFO overflow')
        # Check if fifo data is ready
        elif (mpu_int_status & 0x02):
            # Wait until packet_size number of bytes are ready for reading, default
            # is 42 bytes
            while FIFO_count < self.FIFO_packet_size:
                FIFO_count = self.get_FIFO_count()
            
            self.TimeK = time.time()
            self.FIFO_buffer = self.get_FIFO_bytes(self.FIFO_packet_size)
            self.reset_FIFO()
            Accel = self.DMP_get_acceleration(self.FIFO_buffer)
            self.dmpAccel = np.array([[Accel.x,Accel.y,Accel.z]])
            self.dmpQuaternion = self.DMP_get_quaternion(self.FIFO_buffer)
            self.dmpGrav = self.DMP_get_gravity(self.dmpQuaternion)
            Omega = self.DMP_get_euler_roll_pitch_yaw(self.dmpQuaternion, self.dmpGrav)
            self.dmpEulerOmega = np.array([[Omega.x, Omega.y, Omega.z]])
    
    def propagateLinear(self):
        # Replace as part of Kalman filter
        dt = self.TimeK - self.TimeKminus1
        self.velocityMinus1 = self.velocity
        self.displacementMinus1 = self.displacement
        self.velocity = (self.rawAccel + self.rawAccelMinus1)/2 * dt + self.velocityMinus1
        self.displacement = (self.rawAccel + self.rawAccelMinus1)/4 * (dt**2) + (self.velocity + self.velocityMinus1)/2 * dt + self.displacementMinus1

    def storeIMUdata(self):
        temp = self.rawAccel.tolist() + self.rawOmega.tolist() +  [self.TimeK]
        self.imuOut.append(temp)
    
    
    def calFindOffsets (self,calTime = 5, calOmega = 0.5,saveData = False):
        # Dynamic calibration
        if self.accelBias == [0,0,0]:
            logger.warning('No Bias file found. Static Calibration required')
        
        else:
            # Set motor control as (speed,direction,omega)
            dc = droidControl()
            t0 = time.time()
            idx = int(0)
            dc.setSpeed(0,0,calOmega)
            while time.time() - t0 < calTime:
                # Get raw IMU data
                self.readIMUraw()
                self.storeIMUdata()
                idx +=1

            dc.setSpeed(0,0,0)
            dc.close()
            if saveData:
                np.savetxt('dynamicData.csv',np.array(self.imuOut),delimiter=',')
                logger.info('Saving dynamic data to file: %s entries' % idx)
            
            
            logger.info('Dynamic data gathering complete.')

            self.imuOffset = np.array([0,0,0])

if __name__ == "__main__":
    di = droidInertial()

    print("Class Initialised")
    print("Test read time - RAW")
    
#    counter = 0
#    t0 = time.time()
#    di.readIMUraw()
#    accelRaw = di.rawAccel
#    while counter < 100:
#        di.readIMUraw()
#        accelRaw = np.concatenate((accelRaw, di.rawAccel),axis=0)
#        time.sleep(0.008)
#        counter += 1
#    print('100 Raw values in: %f' % (di.TimeK - t0))
#    print('Average read time: %f' % ((di.TimeK - t0) / 100))
#    
#    print("Test read time - DMP")
#    counter = 0
#    di.reset_FIFO()
#    t0 = time.time()
#    di.readDMP()
#    accelDmp = di.dmpAccel
#    while counter < 100:
#        di.readDMP()
#        accelDmp = np.concatenate((accelDmp, di.dmpAccel),axis=0)
#        counter += 1
#    print('100 DMP values in: %f' % (di.TimeK - t0))
#    print('Average read time: %f' % ((di.TimeK - t0) / 100))
    
    print("Test linear propagation")
    
    t0 = time.time()
    dispRaw = np.array([[0,0,0]])
    counter = 0
    while counter < 5000:
        di.readIMUraw()
        di.propagateLinear()
        dispRaw = np.concatenate((dispRaw, di.displacement),axis=0)
        counter +=1
        
    tend = time.time() 
    print('Time: %f' % (tend-t0))
    fig = plt.figure()
    ax = fig.add_subplot(311)
    ax.plot(dispRaw[:,0])
    ax = fig.add_subplot(312)
    ax.plot(dispRaw[:,1])
    ax = fig.add_subplot(313)
    ax.plot(dispRaw[:,2])
    
    
    

    
        
