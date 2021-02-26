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

    def __init__(self, serial_port='/dev/ttyUSB0',baud=38400,inVarNum=5,outVarNum=6,VarType = 'f'):

        # Initialise output filename and directory
        self.path = []
        self.fileName = []
        
        # Flags for thread operation
        self.isReceiving = False
        self.isRun = True
        self.thread = None

        # Setup variable conversion parameters
        self.VarType = VarType

        if (self.VarType == 'f'):
            self.VarBytes = 4
            logging.debug("Python expecting 4-byte FLOAT type input")
        elif (self.VarType == 'i'):
            self.VarBytes = 2 
            logging.debug("Python expecting 2-byte INT type input")         
        else:
            logging.error("Invalid Input data type, must be 'f' or 'i'")
            sys.exit()
        
        # Initialise OUTPUT test parameters 
        self.outVarNum = outVarNum
        self.dataOut = [0]*self.outVarNum

        self.testType = 0
        self.testMag = 0
        self.testPeriod = 0
        self.Kprop = 0
        self.Kint = 0
        self.Kder = 0
 
        # Initialise INPUT data collection variables
        self.inVarNum = inVarNum  
        self.inRawData = bytearray(self.inVarNum * self.VarBytes)
        self.inData = [0] * self.inVarNum
        
        # Data container for save
        self.saveData = [[0] * self.inVarNum]
        self.initialTimer = 0

        # Open serial comms to Arduino
        try:
            self.ser = serial.Serial(serial_port, baud)
            logging.debug('Serial port connected')
        except:
            logging.error('Serial port not found')
            sys.exit()
        
        
        logging.debug('Initialisation successful')

    def writeSerial(self):
        # Method compiles and converts message to byte string using defined data format.
        # Sends message over serial
        self.ser.reset_output_buffer()
        dataOut = [
            tp.testType,
            tp.testMag,
            tp.testPeriod,
            tp.Kprop,
            tp.Kint, 
            tp.Kder]   
        dataByte = struct.pack(self.VarType *len(dataOut),*dataOut)
        self.ser.write(dataByte)

    def getSerialData(self):
        # Method reads serial stream given data format expected (floats or ints)
        # Set data time logging
        currentTimer = time.perf_counter()
        dataTime = ((currentTimer - self.initialTimer))
        self.previousTimer = currentTimer
        privateData = copy.deepcopy(self.inRawData[:]) # Synchronize all data to the same sample time
        for i in range(self.inVarNum):
            # Unpack message, inVarNum = number of variables, VarBytes = datatype
            data = privateData[(i*self.VarBytes):(self.VarBytes + i*self.VarBytes)]
            self.inData[i] = struct.unpack(self.VarType, data)[0] # Unpack always returns tuple
        # Store values: setPoint, motorSpeed, PWM, timeStamp, completeFlag
        self.TestCompleted = bool(round(self.inData[-1]))
        # todo: Change to allow variable data size inVarNum. Try append([*self.inData])
        self.saveData.append([self.inData[0], self.inData[1], self.inData[2],self.inData[3],self.inData[4]])
        logging.debug('Data updated:%f', dataTime)
    
    def startReadThread(self):
        if self.thread == None:
            self.thread = Thread(target=self.readSerialThread)
            self.thread.daemon = True
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



    # def saveOutput(self):
        
    #     # Check user to save
    #     fileOut = []
    #     # Create file name (PIDtest_ss/mm/hr/d/m/y)

    # def plotOutput(self):
        
    #     # update graph in loop
    #     fileOut = []
        

    def close(self):
        global shutdown_flag
        shutdown_flag = True

        # Close serial port
        self.ser.close()
        print('Releasing resources')

'''
Captures the ctrl+c keyboard command to close the services and free 
resources gracefully.
'''
def signal_handler(signal, frame):
    global shutdown_flag
    shutdown_flag = True
    tp.close()
    print('Closing gracefully. Bye Bye!')


if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    global tp
    tp = TunePID('/dev/ttyUSB0',38400,5,6,'f')
    tp.startReadThread()
    t0 = time.time()
            
    #Use GUI loop for online changes
    # Get user input (Waveform parameters, PID gains)
    tp.testType = 0 # 0 is Step, 1 is Ramp, 2 is Sinusoid
    tp.testMag = 1.0 # Magnitude of input
    tp.testPeriod = 2.5 # Period of test in seconds
    tp.Kprop = 20 # Proportional gain
    tp.Kint = 5 # Integral Gain
    tp.Kder = 0.3 # Derivative gain

    # Send test parameters to arduino
    tp.writeSerial()
    tp.initialTimer = 0
    
    while tp.TestCompleted is False:
        # Get measurements until test is completed
        tp.getSerialData()
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
        
