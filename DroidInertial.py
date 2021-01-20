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
        
        
        debug_output = False
        i2c_bus = 1
        device_address = 0x68

        # Check for Offset file
        try:
            [x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset] = np.loadtxt('/home/pi/droidracer/MPUClass/imuOffsets.csv', delimiter=',', dtype = "int")
            logger.info('Initialising offsets to file.')  
        except:
            [x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset] = np.array([0,0,0,0,0,0]) # Placeholder, run cal routine
            logger.info('No Offset file found. Initialising to Zero')
             
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
        self.TimeK = time.time()
        self.rawAccel = self.get_acceleration()
        self.rawOmega = self.get_rotation()
        
    def readDMP(self):
        self.TimeKminus1 = self.TimeK
        self.TimeK = time.time()
        FIFO_count = self.get_FIFO_count()
        mpu_int_status = self.get_int_status()

        # If overflow is detected by status or fifo count we want to reset
        if (FIFO_count == 1024) or (mpu_int_status & 0x10):
            self.reset_FIFO()
            print('overflow!')
        # Check if fifo data is ready
        elif (mpu_int_status & 0x02):
            # Wait until packet_size number of bytes are ready for reading, default
            # is 42 bytes
            while FIFO_count < self.FIFO_packet_size:
                FIFO_count = self.get_FIFO_count()
            
            self.FIFO_buffer = self.get_FIFO_bytes(self.FIFO_packet_size)
            self.reset_FIFO()
            Accel = self.DMP_get_acceleration(self.FIFO_buffer)
            self.dmpAccel = [Accel.x,Accel.y,Accel.z]
            self.dmpQuaternion = self.DMP_get_quaternion(self.FIFO_buffer)
            self.dmpGrav = self.DMP_get_gravity(self.dmpQuaternion)
            Omega = self.DMP_get_euler_roll_pitch_yaw(self.dmpQuaternion, self.dmpGrav)
            self.dmpEulerOmega = [Omega.x, Omega.y, Omega.z]
            
            
            
        
        
if __name__ == "__main__":
    di = droidInertial()

    print("Class Initialised")
    counter = 0
    t0 = time.time()
    di.readIMUraw()
    accelRaw = di.rawAccel
    while counter < 100:
        di.readIMUraw()
        accelRaw.append(di.rawAccel)
        counter += 1
    print('100 Raw values in: %f' % (di.TimeK - t0))
    print('Average read time: %f' % ((di.TimeK - t0) / 100))
    
    counter = 0
    t0 = time.time()
    di.readDMP()
    accelDmp = di.dmpAccel
    while counter < 100:
        di.readDMP()
        accelDmp.append(di.dmpAccel)
        counter += 1
    print('100 DMP values in: %f' % (di.TimeK - t0))
    print('Average read time: %f' % ((di.TimeK - t0) / 100))
    
    

    
        