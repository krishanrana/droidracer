from threading import Thread
import logging
import signal
import sys
import time
import numpy as np
import os
import copy
import serial
import struct

import matplotlib.pyplot as plt
#import PySimpleGUI as psg


logging.basicConfig(level=logging.DEBUG,
                      format='[%(levelname)s] (%(threadName)-9s) %(message)s',)
shutdown_flag = False


class TunePID:

    def __init__(self, serial_port='/dev/ttyUSB0',baud=38400,inVarNum=4,inVarType = 'f'):

        # Initialise output filename and directory
        self.path = []
        self.fileName = []
        
        # Flags for thread operation
        self.isReceiving = False
        self.isRun = True
        self.thread = None
        
        
        # Initialise input parameters
        
        self.testType = 0
        self.testMag = 0
        self.testPeriod = 0
        self.Kprop = 0
        self.Kint = 0
        self.Kder = 0

        # Initialise data collection variables
        self.inVarNum = inVarNum
        self.inVarType = inVarType
        if (self.inVarType == 'f'):
            self.inVarBytes = 4
        elif (self.inVarType == 'i'):
            self.inVarBytes = 2          
        else:
            logging.error("Invalid Input data type, must be 'f' or 'i'")
            sys.exit()
          
        self.inRawData = bytearray(self.inVarNum * self.inVarBytes)
        self.inData = [0]* self.inVarNum
        # Here are the variables
#         self.outSetSpeed = 0
#         self.outMotorSpeed = 0
#         self.outPWM = 0
#         self.outTime = 0
#         self.testCompleted = False
        
        # Data container for save
        self.csvData = [[.0,0,0,0]];

        # Open serial comms to Arduino
        try:
            self.ser = serial.Serial(serial_port, baud)
        except:
            logging.error('Serial port not found')
            
        self.initialTimer = 0
        self.bufIn = bytearray()
        
        logging.debug('Initialisation successful')

    def writeSerial(self):

        # Send byte string over serial to Arduino
        self.ser.reset_output_buffer()
        dataOut = [
            self.testType,
            self.testMag,
            self.testPeriod, 
            self.Kprop, 
            self.Kint,
            self.Kder]
        dataByte = struct.pack('f'*len(dataOut),*dataOut)
        #dataByte = struct.pack(outVarType*len(dataOut),*dataOut)
        self.ser.write(dataByte)
        


    def updateSerialData(self):
        # Method reads serial stream given data format expected (floats or ints)
        # Set data time logging
        currentTimer = time.perf_counter()
        dataTime = ((currentTimer - self.initialTimer))     # the first reading will be erroneous
        self.previousTimer = currentTimer
        privateData = copy.deepcopy(self.inRawData[:])    # Synchronize all data to the same sample time
        for i in range(self.inVarNum):
            # Unpack message, numPlots = number of variables, dataNumBytes = datatype
            data = privateData[(i*self.inVarBytes):(self.inVarBytes + i*self.inVarBytes)]
#             value,  = struct.unpack(self.dataType, data)
            self.inData[i] = struct.unpack(self.inVarType, data)[0] # Unpack always returns tuple
        # Store values
        self.csvData.append([self.inData[0], self.inData[1], self.inData[2],dataTime])
        logging.debug('Data updated')
    
    def startReadThread(self):
        if self.thread == None:
            self.thread = Thread(target=self.readSerialThread)
            #self.thread.daemon = True
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)
    
    def readSerialThread(self):    # retrieve data
        time.sleep(1)  # give some buffer time for retrieving data
        self.ser.reset_input_buffer()
        while (self.isRun):
            self.ser.readinto(self.inRawData)
            self.isReceiving = True    

# Function for reading char strings from arduino (similar to serial.readline()
#     def readSerial(self):
#         i = self.bufIn.find(b"\n")
#         if i >= 0:
#             r = self.buf[:i+1]
#             self.buf = self.buf[i+1:]
#             return r
#         while True:
#             i = max(1, min(2048, self.ser.inWaiting())) # in_waiting for python3
#             data = self.ser.read(i)
#             i = data.find(b"\n")
#             if i >= 0:
#                 r = self.buf + data[:i+1]
#                 self.buf[0:] = data[i+1:]
#                 return r
#             else:
#                 self.buf.extend(data)
    
#     def getOutputMsg(self):
#         # Read buffer
#         #msgIn = self.readSerial()
#         #self.ser.flush()
#         msgIn = self.ser.readline()
#         
#         
#         # Process message (tab seperated)
#         try:
#             msgOut = msgIn.split("\t")
#             self.outSetSpeed = msgOut[0]
#             self.outMotorSpeed = msgOut[1]
#             self.outPWM = msgOut[2]
#             self.outTime = msgOut[3]
#             self.testCompleted = msgOut[4]
#             # Write parameters to output list  
#             self.dataIn.append([msgOut[0:4]])
#             print(msgOut)
#         except:
#             logging.warning('Message lost')

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
    tp = TunePID('/dev/ttyUSB0',38400,3,'f')
    tp.startReadThread()
    t0 = time.time()
            
    #while (True):
    # Get user input (Waveform parameters, PID gains)
    tp.testType = 0 # 0 is Step, 1 is Ramp, 2 is Sinusoid
    tp.testMag = 1.0 # Magnitude of input
    tp.testPeriod = 2.5 # Period of test in seconds
    tp.Kprop = 20 # Proportional gain
    tp.Kint = 5 # Integral Gain
    tp.Kder = 0.3 # Derivative gain

    # Send test parameters to arduino
    tp.writeSerial()
    
    for _ in range(10):
        # Get measurements
        tp.updateSerialData()
        for i in range(tp.inVarNum):
            print(tp.inData[i])
        print(time.time() - t0)
        # Plot measurements
        #tp.plotOutput()
        
    # Save file
    #tp.saveOutput()

        
        
        
    print('out of loop')
    tp.close()

    print('All done')
        
