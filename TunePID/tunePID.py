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
# Log file location
logfile = 'debug.log'

# Define your own logger name
logger = logging.getLogger("tunePIDlog")
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

# logging.basicConfig(level=logging.DEBUG,
#                       format='[%(levelname)s] (%(threadName)-9s) %(message)s',)
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
        
        # Loop control states
        self.TestRunning = False
        self.remoteWaiting = True

        # Setup variable conversion parameters
        self.VarType = VarType

        if (self.VarType == 'f'):
            self.VarBytes = 4
            logger.debug("Python expecting 4-byte FLOAT type input")
        elif (self.VarType == 'i'):
            self.VarBytes = 2 
            logger.debug("Python expecting 2-byte INT type input")         
        else:
            logger.error("Invalid Input data type, must be 'f' or 'i'")
            sys.exit()
        
        # Initialise OUTPUT test parameters 
        self.outVarNum = outVarNum
        self.dataOut = [0]*self.outVarNum

        self.testType = 0
        self.testMag = 0.0
        self.testPeriod = 0.0
        self.Kprop = 0.0
        self.Kint = 0.0
        self.Kder = 0.0
 
        # Initialise INPUT data collection variables
        self.inVarNum = inVarNum  
        self.inRawData = bytearray(self.inVarNum * self.VarBytes)
        self.inData = [0] * self.inVarNum
        
        # Data container for save
        self.saveData = [[0] * self.inVarNum]
        self.initialTimer = 0.0

        # Open serial comms to Arduino
        try:
            self.ser = serial.Serial(serial_port, baud)
            logger.debug('Serial port connected')
        except:
            logger.error('Serial port not found')
            sys.exit()
        
        
        logger.debug('Initialisation successful')

    def writeSerial(self):
        # Method compiles and converts message to byte string using defined data format.
        # Sends message over serial
        self.ser.reset_output_buffer()
        dataOut = [
            self.testType,
            self.testMag,
            self.testPeriod,
            self.Kprop,
            self.Kint, 
            self.Kder]   
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
        
        # Decode signal
        remoteSignal = round(self.inData[-1])
        if remoteSignal == 0:    
            self.TestRunning = False
            self.remoteWaiting = False
            logger.debug("Signal = 0: Remote completed test")
        elif remoteSignal == 1:    
            self.TestRunning = True
            self.remoteWaiting = False
            logger.debug("Signal = 1: Remote running test")
        elif remoteSignal == 5:    
            self.TestRunning = True
            self.remoteWaiting = False
            logger.debug("Signal = 5: Remote has received instructions, starting test")
        elif remoteSignal == 100:    
            self.TestRunning = False
            self.remoteWaiting = True
            logger.debug("Signal = 100: Remote waiting for instructions")
            
        # todo: Change to allow variable data size inVarNum. Try append([*self.inData])
        self.saveData.append([self.inData[0], self.inData[1], self.inData[2],self.inData[3],self.inData[4]])
        logger.debug('Data updated:%f', dataTime)
    
    def startReadThread(self):
        if self.thread == None:
            self.thread = Thread(target=self.readSerialThread,daemon = True)
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
            time.sleep(0.000001)


    def saveOutput(self):
        saveDataNP = np.array(self.saveData)
        self.saveDataNP = saveDataNP[saveDataNP[:,3]>0]
        np.savetxt('PIDTest.csv',self.saveDataNP,delimiter=',')
        logger.debug('Results saved to file')          

    def plotOutput(self):
        # Plot values: setPoint, motorSpeed, PWM, timeStamp, signal
        fig, ax = plt.subplots(figsize=(10,6))                    
        ax.plot(self.saveDataNP[:,3],self.saveDataNP[:,0], 'r',label='Setpoint')
        ax.plot(self.saveDataNP[:,3],self.saveDataNP[:,1], 'b',label='Motor speed')
        ax.set_ylabel('Velocity - m/s')
        ax.set_ylim(-(2*self.testMag),(2*self.testMag))
        ax2 = ax.twinx()
        ax2.plot(self.saveDataNP[:,3],self.saveDataNP[:,2], 'g',label='PWM')
        ax2.set_ylabel('Control u - PWM')
        ax2.set_ylim(-255,255)
        ax.set_title('PID Test')
        #ax.legend()
        fig.legend(loc="upper right", bbox_to_anchor=(1,1), bbox_transform=ax.transAxes)
        plt.show()
        

    def close(self):
        global shutdown_flag
        shutdown_flag = True
        # Shutdown thread
        self.isrun = False
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
    sys.exit()


if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    global tp
    tp = TunePID('/dev/ttyUSB0',38400,5,6,'f')
    tp.startReadThread()
    t0 = time.time()
            
    #Use GUI loop for online changes
    # Get user input (Waveform parameters, PID gains)
    tp.testType = 2.0 # 0 is Step, 1 is Ramp, 2 is Sinusoid
    tp.testMag = 0.2 # Magnitude of input
    tp.testPeriod = 2.0 # Period of waveform in seconds
    tp.Kprop = 1.5 # Proportional gain
    tp.Kint = 40 # Integral Gain
    tp.Kder = 0.001 # Derivative gain
    
    # Check if remote needs instructions
    while tp.remoteWaiting == True:
        # Send test parameters to arduino
        tp.getSerialData()
        tp.writeSerial()
        logger.debug("Data sent to remote")
        time.sleep(0.2)
        
    tp.initialTimer = 0
    
    while tp.TestRunning is True:
        # Get measurements until test is completed
        tp.getSerialData()
        for i in range(tp.inVarNum):
            print("%.3f" % tp.inData[i])
        time.sleep(0.015)

    # Save file
    tp.saveOutput()
    
    # Plot measurements
    tp.plotOutput()

    print('out of loop')
    tp.close()

    print('All done')
        
