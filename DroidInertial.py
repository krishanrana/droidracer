# -*- coding: utf-8 -*-
"""
Created on Mon Jan 18 11:29:54 2021

@author: doug palmer

NOTE: Some code from MPU6050.py and iC2 library by Jeff Rowland via Geir Istad
"""
# Import packages
import numpy as np
import sys
from MPUClass.MPU6050 import MPU6050 # Rewrite class using native numpy
import logging
import time
import matplotlib.pyplot as plt
import PySimpleGUI as psg
from scipy.optimize import least_squares



from DroidControl import droidControl

# Setup logging (Use droidlogging.conf as alternative)

# Log file location
logfile = 'debugIMU.txt'
# Define your own logger name
logger = logging.getLogger("Imulog")
# Set default logging level to DEBUG
logger.setLevel(logging.DEBUG)

# create console handler
print_format = logging.Formatter('[%(levelname)s] (%(threadName)-9s) %(message)s')
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.DEBUG)
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
        #self.gravity = np.array([0,0,9.80665])
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
            # Use existing IMU bias            
            self.accelBias = np.array([x_accel_offset, y_accel_offset,z_accel_offset])
            print(self.accelBias)
            self.omegaBias = np.array([x_gyro_offset, y_gyro_offset, z_gyro_offset])
            print(self.omegaBias)
        
        except:
            [x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset] = np.array([0,0,0,0,0,0]) # Placeholder, run cal routine
            logger.warning('No Bias file found. Static Calibration required')

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
        # Start DMP
        self.set_DMP_enabled(True)
        self.FIFO_packet_size = self.DMP_get_FIFO_packet_size()
                
        logger.debug(('Interrupt status: %s ' % hex(self.get_int_status())))
        logger.debug('FIFO Packet size: %s ' % self.FIFO_packet_size)
        logger.debug(('FIFO count: %s ' % hex(self.get_FIFO_count())))
        
        self.set_x_accel_offset(x_accel_offset)
        self.set_y_accel_offset(y_accel_offset)
        self.set_z_accel_offset(z_accel_offset)
               
    def readIMUraw(self):
        self.TimeKminus1 = self.TimeK
        self.rawAccelMinus1 = self.rawAccel
        self.rawOmegaMinus1 = self.rawOmega
        self.TimeK = time.time()
        # : Store as numpy arrays as we will be using matrix operations for Kalman filter
        self.rawAccel = np.array([self.get_acceleration()])/ 16384.0  * self.gravity
        self.rawOmega = np.array([self.get_rotation()]) / 131.0
               
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
            self.dmpAccel = np.array([Accel.x,Accel.y,Accel.z])
            self.dmpQuaternion = self.DMP_get_quaternion(self.FIFO_buffer)
            self.dmpGrav = self.DMP_get_gravity(self.dmpQuaternion)
            Omega = self.DMP_get_euler_roll_pitch_yaw(self.dmpQuaternion, self.dmpGrav)
            self.dmpEulerOmega = np.array([Omega.x, Omega.y, Omega.z])
    
    def propagateLinear(self):
        # Replace as part of Kalman filter
        dt = self.TimeK - self.TimeKminus1
        self.dt = dt
        self.velocityMinus1 = self.velocity
        self.displacementMinus1 = self.displacement
        self.velocity = (self.rawAccel + self.rawAccelMinus1)/2 * dt + self.velocityMinus1
        self.displacement =  (self.velocity + self.velocityMinus1)/2 * dt + self.displacementMinus1

    def storeIMUdata(self):
        temp = self.rawAccel.tolist() + self.rawOmega.tolist() +  [self.TimeK]
        self.imuOut.append(temp)
    # TODO: Wrap calibration functions in seperate calibration class
    def recordStaticIMU(self, calTime = 5, saveData = True):
        # GUI asks user to prepare droid for test n
        poses = {'pose 1':(0,'all'),
                 'pose 2':(1,'1'),
                 'pose 3':(2,'1'),
                 'pose 4':(1,'2'),
                 'pose 5':(2,'2'),
                 'pose 6':(1,'3'),
                 'pose 7':(2,'3'),
                 'pose 8':(1,'1 & 2'),
                 'pose 9':(2,'1 & 2'),}
        
        self.meanAccel=[]
        self.varAccel=[]
        self.accelData = []
        # Get data at various poses
        for pose, blocks in poses.items():
            print('\n','Set up robot in ',pose, 'with ',blocks[0], 'blocks under wheel ',blocks[1],'\n')
            input('Press any key to continue, ctrl C to quit')
            poseAccel = []
            t00 = time.time()
            while time.time() - t00 < calTime:
                # Get raw IMU data
                self.readIMUraw()
                poseAccel.append(self.rawAccel.tolist())
                
            
            meanPoseAccel = np.mean(poseAccel,axis=0)
            varPoseAccel = np.var(poseAccel,axis=0)
            self.accelData.append(poseAccel)
            self.meanAccel.append(meanPoseAccel)
            self.varAccel.append(varPoseAccel)
            
            print(len(poseAccel[0]),' samples collected in %0.2f seconds'%  calTime)
            print('\n')
            print('Mean accel (x,y,z) = ', meanPoseAccel)
            print('Variance accel (x,y,z) = ', varPoseAccel,'\n'*3)
            # Optimse for biases
        if saveData:
                np.savetxt('staticData.csv',np.array(self.accelData),delimiter=',')
                logger.info('Saving static data to file: %s entries' % len(self.accelData[0]))
    
    def calStatic(self, getStaticData = False):
        if getStaticData is True:
            logger.info('Get ready to acquire static calibration data from robot!')
            self.recordStaticIMU()
        else:
            try:
               self.accelData = np.loadtxt('/home/pi/droidracer/MPUClass/staticData.csv', delimiter=',', dtype = "int")
            except:
                logger.info('Get ready to acquire static calibration data from robot!')
                self.recordStaticIMU()
        b0 = self.accelBias
        self.biasEstimate = least_squares(costFunction, b0, args=(self.gravity, self.accelData))
    


        
             
    def calFindOffsets (self,calTime = 5, calOmega = 0.5,saveData = False):
        # Dynamic calibration
        if self.accelBias == [0,0,0]:
            logger.warning('No Bias file found. Static Calibration required')
        
        else:
            # Set motor control as (speed,direction,omega)
            # Note: Change droidControl to implement new PID control
            dc = droidControl()
            t00 = time.time()
            idx = int(0)
            dc.setSpeed(0,0,calOmega)
            while time.time() - t00 < calTime:
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
            
            
#-------Methods to test functionality--------------------
            
            
    def testReadSpeed(self):
       print("Test read time - RAW")
       counter = 0
       t0 = time.time()
       self.readIMUraw()
       accelRaw = self.rawAccel
       while counter < 100:
           self.readIMUraw()
           accelRaw = np.concatenate((accelRaw, self.rawAccel),axis=0)
           time.sleep(0.008)
           counter += 1
       print('100 Raw values in: %f' % (self.TimeK - t0))
       print('Average read time: %f' % ((self.TimeK - t0) / 100))
       
       print("Test read time - DMP")
       counter = 0
       self.reset_FIFO()
       t0 = time.time()
       self.readDMP()
       accelDmp = self.dmpAccel
       while counter < 100:
           self.readDMP()
           accelDmp = np.concatenate((accelDmp, self.dmpAccel),axis=0)
           counter += 1
       print('100 DMP values read in: %f' % (self.TimeK - t0))
       print('Average read time: %f' % ((self.TimeK - t0) / 100))
       
    def testLinProp(self):
        print("Test linear propagation")
        # Warm up IMU
        for idx in range(10):
            self.readIMUraw()
            print(idx)
            time.sleep(0.1)
            
        # Initialise storage containers   
        self.propagateLinear()
        accelRaw = self.rawAccel
        dispRaw = self.displacement
        velRaw = self.velocity
        
        t0 = time.time()
        counter = 0
        while counter < 2000:
            self.readIMUraw()
            self.propagateLinear()
            dispRaw = np.concatenate((dispRaw, self.displacement),axis=0)
            velRaw = np.concatenate((velRaw, self.velocity),axis=0)
            accelRaw = np.concatenate((accelRaw, self.rawAccel),axis=0)
            counter +=1
            
        tend = time.time()
        
        print('Time: %f' % (tend-t0))
        print('xAc bias: %0.4f' % np.median(accelRaw[:,0]))
        print('yAc bias: %0.4f' % np.median(accelRaw[:,1]))
        print('zAc bias: %0.4f' % np.median(accelRaw[:,2]))
        
        fig, ax = plt.subplots(3,3,figsize=(12,9))
        
        ax[0,0].plot(dispRaw[:,0], 'r',label='Displacement - x (m)')
        ax[0,0].set_ylabel('Displacement - m')
        ax[0,1].plot(dispRaw[:,1], 'r',label='Displacement - y (m)')
        ax[0,2].plot(dispRaw[:,2], 'r',label='Displacement - z (m)')
        
        ax[1,0].plot(velRaw[:,0],'g',label='Velocity - x (m/s)')
        ax[1,0].set_ylabel('Velocity - (m/s)')
        ax[1,1].plot(velRaw[:,1],'g',label='Velocity - y (m/s)')
        ax[1,2].plot(velRaw[:,2],'g',label='Velocity - z (m/s)')
        
        ax[2,0].plot(accelRaw[:,0],'b',label='Accel - x (m/s^2)')
        ax[2,0].set_ylabel('Accel - (m/s^2)')
        ax[2,1].plot(accelRaw[:,1],'b',label='Accel - y (m/s^2)')
        ax[2,2].plot(accelRaw[:,2],'b',label='Accel - z (m/s^2)')
        
        plt.show()
        
def costFunction(grav, obs, bias):
    return grav**2 - ((obs[0] - bias[0])**2 + (obs[1] - bias[1])**2 + (obs[2] - bias[2])**2)
                
                

if __name__ == "__main__":
    di = droidInertial()
#     di.testReadSpeed()
#     di.testLinProp()
#     di.recordStaticIMU()
    di.calStatic()
    
   
    
    
    
    

    
        
