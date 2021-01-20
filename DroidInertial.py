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
        # Set gravuity at ~9.80ms^-2
        self.gravity = 9.80665
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
        
        logger.debug(('Interrupt status: %s ' % hex(self.get_int_status())))
        logger.debug(('FIFO Packet size: %s ' % hex(self.DMP_get_FIFO_packet_size())))
        logger.debug(('FIFO count: %s ' % hex(self.get_FIFO_count())))
        
        
        
    def update_position(self):
        self.x = []
        
        
if __name__ == "__main__":
    di = droidInertial()

    print("Class Initialised")
    accel_reading = di.get_acceleration()
    x_accel_reading = accel_reading[0]
    y_accel_reading = accel_reading[1]
    z_accel_reading = accel_reading[2]
    print(x_accel_reading, y_accel_reading, z_accel_reading)

    
        