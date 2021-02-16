from __future__ import division

import logging
import signal
import sys
import time
import numpy as np
import os
import serial
import struct
import matplotlib.pyplot as plt
import PySimpleGUI as psg
from fractions import Fraction


logging.basicConfig(level=logging.DEBUG,
                      format='[%(levelname)s] (%(threadName)-9s) %(message)s',)
shutdown_flag = False


class TunePID:

    def __init__(self, serial_port='/dev/ttyUSB0',baud=9600):

        # Initialise output filename and directory
        self.path = []
        self.fileName = []
        
        # Initialise input parameters
        self.testType = 0
        self.testMag = 0
        self.testPeriod = 0
        self.Kprop = 0
        self.Kint = 0
        self.Kder = 0

        # Initialise output variables
        self.dataIn = [[0,0,0,0]]
        self.outSetSpeed = 0
        self.outMotorSpeed = 0
        self.outPWM = 0
        self.outTime = 0

        self.testCompleted = False

        # Open serial comms to Arduino
        try:
            self.ser = serial.Serial(serial_port, baud)
        except:
            logging.error('Serial port not found')
            

        self.buf = bytearray()
        
        print('Class initialised')


    def readSerial(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.ser.in_waiting))
            data = self.ser.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)
    
    def writeSerial(self):

        # Send over serial to Arduino

        dataOut = (
            str(self.testType) + "," +
            str(self.testMag) + "," +
            str(self.testPeriod) + "," +
            str(self.Kprop) + "," +
            str(self.Kint) + "," +
            str(self.Kder) + "\n")

        self.ser.write(dataOut.encode("ascii"))

    def getOutputMsg(self):
        # Read buffer
        msgIn = self.readSerial()
        # Process message (tab seperated)
        if msgIn is not None:
            msgOut = msgIn.split("\t")
            self.outSetSpeed = msgOut[1]
            self.outMotorSpeed = msgOut[2]
            self.outPWM = msgOut[3]
            self.outTime = msgOut[4]
        # Write parameters to output list  
        self.dataIn.append(msgOut)

    def saveOutput(self):
        
        # Check user to save
        fileOut = []
        # Create file name (PIDtest_ss/mm/hr/d/m/y)

    def plotOutput(self):
        
        # update graph in loop
        fileOut = []
        

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
    tp.close()
    print('Closing gracefully. Bye Bye!')


if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    global tp
    tp = TunePID()
    t0 = time.time()
            
    while (True):
        # Get user input (Waveform parameters, PID gains)
        tp.testType = 0 # 0 is Step, 1 is Ramp, 2 is Sinusoid
        tp.testMag = 0 # Magnitude of input
        tp.testPeriod = 0 # Period of test in seconds
        tp.Kprop = 0 # Proportional gain
        tp.Kint = 0 # Integral Gain
        tp.Kder = 0 # Derivative gain

        # Send test parameters to arduino
        tp.writeSerial()
        while (tp.testCompleted is False):
            # Get measurements
            tp.getOutputMsg()
            # Plot measurements
            tp.plotOutput()
            
        # Save file
        tp.saveOutput()

        
        
        
    print('out of loop')
    tp.close()

    print('All done')
        
