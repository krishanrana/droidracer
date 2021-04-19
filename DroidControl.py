
import logging
import signal
import sys
import time
import numpy as np
import os
import serial
import struct
#from fractions import Fraction

""" This class contains various methods for high level control of omni droid robot
"""

# logging.basicConfig(level=logging.DEBUG,
#                       format='[%(levelname)s] (%(threadName)-9s) %(message)s',)
shutdown_flag = False


class droidControl:

    def __init__(self, serial_port='/dev/ttyUSB0',baud=38400):

        # Velocity controller
        self.speed = 0
        self.vector = 0
        self.omega = 0

        # Open serial comms to Arduino
        self.ser = serial.Serial(serial_port, baud)
        print('Class initialised')


    def setSpeed(self, speed, vector, omega):
        # Store the speeds just in case
        self.speed = speed
        self.vector = vector
        self.omega = omega

        # Send over serial to Arduino
        speed = str(speed)
        vector = str(vector)
        omega = str(omega)
        data_out = speed + "," + vector + "," + omega + "\n"
        self.ser.write(data_out.encode("ascii"))

    def close(self):
        global shutdown_flag
        shutdown_flag = True

        # Close serial port
        self.ser.close()
        print('Releasing resources')

'''
Captures the ctrl+c keyboard command to close the services and free 
resources gracefully.
Allows the sockets and camera to be immediately reopened on next run without
waiting for the OS to close them on us.
'''
def signal_handler(signal, frame):
    global shutdown_flag
    shutdown_flag = True
    dc.close()
    print('Closing gracefully. Bye Bye!')


if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    global dc
    runTime = 10
    dc = droidControl()
    t0 = time.time()        
    while time.time()-t0 < runTime:
        dc.setSpeed(0, 0, 1)
        byteString = dc.ser.read(78)
        print(byteString)
        #time.sleep(0.017)
        
    print('out of loop')
    dc.setSpeed(0, 0, 0)
    dc.close()

    print('All done')
        
