from __future__ import division

import logging
import signal
import sys
import time
import numpy as np
import os
import serial

import matplotlib.pyplot as plt
#import PySimpleGUI as psg


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
        
        logging.debug('Initialisation successful')


    def readSerial(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.ser.inWaiting())) # in_waiting for python3
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
        print(dataOut.encode("ascii"))

    def getOutputMsg(self):
        # Read buffer
        #msgIn = self.readSerial()
        #self.ser.flush()
        msgIn = self.ser.readline()
        
        
        # Process message (tab seperated)
        try:
            msgOut = msgIn.split("\t")
            self.outSetSpeed = msgOut[0]
            self.outMotorSpeed = msgOut[1]
            self.outPWM = msgOut[2]
            self.outTime = msgOut[3]
            self.testCompleted = msgOut[4]
            # Write parameters to output list  
            self.dataIn.append([msgOut[0:4]])
            print(msgOut)
        except:
            logging.warning('Message lost')

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
            
    #while (True):
    # Get user input (Waveform parameters, PID gains)
    tp.testType = float(1) # 0 is Step, 1 is Ramp, 2 is Sinusoid
    tp.testMag = float(2) # Magnitude of input
    tp.testPeriod = float(4) # Period of test in seconds
    tp.Kprop = float(1) # Proportional gain
    tp.Kint = float(1) # Integral Gain
    tp.Kder = float(1) # Derivative gain

    # Send test parameters to arduino
    tp.writeSerial()
    while (time.time() - t0 < 2):
        # Get measurements
        tp.getOutputMsg()
        print(". \n")
        # Plot measurements
        #tp.plotOutput()
        
    # Save file
    #tp.saveOutput()

        
        
        
    print('out of loop')
    tp.close()

    print('All done')
        
