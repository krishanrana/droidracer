""" This class contains various methods for high level control of omni droid robot
"""

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
logger = logging.getLogger("droidControlLog")
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


shutdown_flag = False


class droidControl:

    def __init__(self, serial_port='/dev/ttyUSB0',baud=38400,inVarNum=10,outVarNum=7,VarType = 'f'):

        # Initialise output filename and directory
        self.path = []
        self.fileName = []
        
        # Flags for thread operation
        self.isReceiving = False
        self.isRun = True
        self.thread = None
        
        # Loop control states
        self.droidMoving = False
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
        
        # Motor velociries and control parameters
        self.droidradius = 0.15
        
        # Control inputs
        self.MaxLinearVelocity = 0.5 # Maximum linear speed m/s
        self.MaxAngularVelocity = np.pi/2 # Maximum angular velocity rad/s
        
        self.droidHeading = np.pi/2 # direction of travel (radians relative to droid frame)
        self.worldRotation = 0 # # rotation of droid relative to initial pose
        self.LinearSpeed = 0 # m/s       
        self.AngularSpeed = 0.0 # radian/s
    
        self.velM1 = 0.0
        self.velM2 = 0.0
        self.velM3 = 0.0
        self.Kprop = 0.0
        self.Kint = 0.0
        self.Kder = 0.0
        
        self.runCommand = 0.0
 
        # Initialise INPUT data collection variables
        self.inVarNum = inVarNum  
        self.inRawData = bytearray(self.inVarNum * self.VarBytes)
        self.inData = [0] * self.inVarNum
        
        # Data container for save
        # TODO: make this automatic, currently adding timestamp and setpoint
        self.saveData = [[0] * (self.inVarNum + 1)]
        self.initialTimer = 0.0

        # Open serial comms to Arduino
        try:
            self.ser = serial.Serial(serial_port, baud)
            logger.debug('Serial port connected')
        except:
            logger.error('Serial port not found')
            sys.exit()
        
        
        logger.debug('Initialisation successful')
        
#------Bi-directional communication methods-----------

    def writeSerial(self):
        # Method compiles and converts message to byte string using defined data format.
        # Sends message over serial
        self.ser.reset_output_buffer()
        dataOut = [
            self.runCommand,
            np.round(self.velM1,2),
            np.round(self.velM2,2),
            np.round(self.velM3,2),
            self.Kprop,
            self.Kint, 
            self.Kder]   
        dataByte = struct.pack(self.VarType *len(dataOut),*dataOut)
        self.ser.write(dataByte)
        logger.debug('Data written to serial')
        print(self.runCommand)

    def getSerialData(self):
        # Method reads serial stream given data format expected (floats or ints)
        # Set data time logging
        currentTimer = time.time()
        dataTime = ((currentTimer - self.initialTimer))
        privateData = copy.deepcopy(self.inRawData[:]) # Synchronize all data to the same sample time
        
        byteSignal = privateData[0:4]       
        intSignal = struct.unpack('i', byteSignal)[0]
        self.inData[0] = intSignal
        for i in range(1,self.inVarNum):
            # Unpack message, inVarNum = number of variables, VarBytes = datatype
            data = privateData[(i*self.VarBytes):(self.VarBytes + i*self.VarBytes)]
            self.inData[i] = struct.unpack(self.VarType, data)[0] # Unpack always returns tuple
        
        # Decode signal from remote, update states
        # Unpack 4 Byte packet and read bits
        encoded32 = np.uint32(self.inData[0])
        self.remoteSignal = [1 if c=='1' else 0 for c in bin(intSignal)[2:].zfill(4)]
        
        print(self.remoteSignal)
        if self.remoteSignal[-1] == 0:
            self.remoteWaiting = True
            logger.debug("NO_COMMS")
        if self.remoteSignal[-1] == 1:
            self.remoteWaiting = False
        if self.remoteSignal[-2] == 0:
            self.droidMoving = False
            logger.debug("PARKED")
        if self.remoteSignal[-2] == 1:
            self.droidRunning = True
            logger.debug("RUNNING")
        if self.remoteSignal[-3] == 0:
            self.droidMoving = False
        if self.remoteSignal[-3] == 1:
            self.droidRunning = True
            logger.debug("TIMEOUT")
            
        # todo: Change to allow variable data size inVarNum. Try append([*self.inData])
        self.saveData.append([self.inData])
#         self.saveData.append([self.inData[0], self.velM1, self.inData[1], self.velM2, self.inData[2],self.inData[3],self.velM3,self.inData[4],self.inData[5],self.inData[6], dataTime])
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
            
#------High level control methods-----------
            
            
    def calcMotorVels(self, linearVel, theta, angularVel):
        # 3 wheel omniwheel kinematics
        # Transforms from velocity/heading/angular velocity to motor speeds
        self.velM1 = -((linearVel * (-0.5 * np.cos(theta) - np.sqrt(3) / 2 * np.sin(theta))) + (2 * angularVel * self.droidradius));
        self.velM2 = -((linearVel * (-0.5 * np.cos(theta) + np.sqrt(3) / 2 * np.sin(theta))) + (2 * angularVel * self.droidradius));
        self.velM3 = -(linearVel * np.cos(theta) + (2 * angularVel * self.droidradius));
    
    def estimateDroidMotion(self):
        # Inverse kinematic model: INPUT - wheel velocities, OUTPUT velocity, ang vel
        pass
#------Tests--------------------------------
    
    def testVectorDrive(self, Distance, Heading):
        Heading = Heading %(2*np.pi)
        
        # Calculate time to drive
        if Distance <= 0:
            runTime = 0
            logger.debug("No distance, aborting")
            return;
        elif self.LinearSpeed <= 0:
            runTime = 0
            logger.debug("No linear velocity, aborting")
            return;
        else:
            runTime = abs(Distance / self.LinearSpeed)
            logger.debug("Estimated runtime = %0.2f " % runTime)
            if runTime > 10:
                logger.debug("Runtime too long, aborting")
                return;

        # Calculate 3 motor velocities from input
        dc.calcMotorVels(self.LinearSpeed, Heading, 0)
        logger.debug("M1: %f" % dc.velM1)
        logger.debug("M2: %f" % dc.velM2)
        logger.debug("M3: %f" % dc.velM3)
        # Initialise timer
        dc.initialTimer = time.time()
        
        dc.runCommand = 1.0
        driveTime = 0.0
        # Test drive to target - simple    
        while driveTime <= runTime:
            dc.getSerialData()
            dc.writeSerial()
            # Naive open-loop odometry 
            driveTime = time.time() - dc.initialTimer
            displacement = dc.LinearVelocity * driveTime
    #         for i in range(dc.inVarNum):
    #             print("%.3f" % dc.inData[i])
            logger.debug('Current displacement: %0.3f' % displacement)
            time.sleep(0.025)
            
        dc.runCommand = 0.0
        dc.writeSerial()
        time.sleep(0.030)
        while self.droidMoving == True:
            dc.writeSerial()
            dc.getSerialData()
            time.sleep(0.025)
        logger.debug('Droid Parked')
        
    def testRotationDrive(self, Rotation):
        
        direction = Rotation/abs(Rotation)
        Rotation = Rotation %(2*np.pi)
        # Calculate time to drive
        if Rotation == 0:
            runTime = 0
            logger.debug("No rotation, aborting")
            return;
        elif self.AngularSpeed ==  0:
            runTime = 0
            logger.debug("No angular velocity, aborting")
            return;
        else:
            runTime = abs(Rotation / (self.AngularSpeed))
            logger.debug("Estimated runtime = %0.2f " % runTime)
            if runTime > 10:
                logger.debug("Runtime too long, aborting")
                return;
        # Calculate velocity including sign to reach destination
        AngularVelocity = self.AngularSpeed * direction
        # Calculate 3 motor velocities from input
        dc.calcMotorVels(0, 0, AngularVelocity)
        logger.debug("M1: %f" % dc.velM1)
        logger.debug("M2: %f" % dc.velM2)
        logger.debug("M3: %f" % dc.velM3)
        # Initialise timer
        dc.initialTimer = time.time()
        
        dc.runCommand = 1.0
        driveTime = 0.0
        # Test drive to target - simple    
        while driveTime <= runTime:
            dc.getSerialData()
            dc.writeSerial()
            # Naive open-loop odometry 
            driveTime = time.time() - dc.initialTimer
            # Replace with estimate from odometry
            rotation = self.AngularSpeed * driveTime
    #         for i in range(dc.inVarNum):
    #             print("%.3f" % dc.inData[i])
            logger.debug('Current rotation: %0.3f' % rotation)
            time.sleep(0.025)
            
        dc.runCommand = 0.0
        dc.writeSerial()
        time.sleep(0.030)
        while self.droidMoving == True:
            dc.writeSerial()
            dc.getSerialData()
            time.sleep(0.025)
        logger.debug('Droid Parked')
        
#------Data visualisation methods-----------
    
    def saveOutput(self):
        saveDataNP = np.array(self.saveData)
        self.saveDataNP = saveDataNP[saveDataNP[:,9]>0]
        np.savetxt('drive.csv',self.saveDataNP,delimiter=',')
        logger.debug('Results saved to file')          

    def plotOutput(self):
        # Plot values: setPoint, motorSpeed, PWM, timeStamp, signal
        fig, ax = plt.subplots(1,3, figsize=(12,9))                    
        ax[0,0].plot(self.saveDataNP[:,9],self.saveDataNP[:,0], 'r',label='M1 Setpoint')
        ax[0,0].plot(self.saveDataNP[:,9],self.saveDataNP[:,1], 'b',label='M1 Motor speed')
        ax[0,0].set_ylabel('Velocity - m/s')
        ax[0,0].set_ylim(-2,2)
        ax2 = ax.twinx()
        ax2[0,0].plot(self.saveDataNP[:,9],self.saveDataNP[:,2], 'g',label='M1 PWM')
        ax2[0,0].set_ylabel('Control u - PWM')
        ax2[0,0].set_ylim(-255,255)
        ax[0,0].set_title('M1')
        
        ax[0,1].plot(self.saveDataNP[:,9],self.saveDataNP[:,3], 'r',label='M2 Setpoint')
        ax[0,1].plot(self.saveDataNP[:,9],self.saveDataNP[:,4], 'b',label='M2 Motor speed')
        ax[0,1].set_ylabel('Velocity - m/s')
        ax[0,1].set_ylim(-2,2)
#         ax2 = ax.twinx()
        ax2[0,1].plot(self.saveDataNP[:,9],self.saveDataNP[:,5], 'g',label='M2 PWM')
        ax2[0,1].set_ylabel('Control u - PWM')
        ax2[0,1].set_ylim(-255,255)
        ax[0,1].set_title('M2')
    
        ax[0,2].plot(self.saveDataNP[:,9],self.saveDataNP[:,7], 'r',label='M1 Setpoint')
        ax[0,2].plot(self.saveDataNP[:,9],self.saveDataNP[:,8], 'b',label='M2 Motor speed')
        ax[0,2].set_ylabel('Velocity - m/s')
        ax[0,2].set_ylim(-2,2)
#         ax2 = ax.twinx()
        ax2[0,2].plot(self.saveDataNP[:,9],self.saveDataNP[:,9], 'g',label='M3 PWM')
        ax2[0,2].set_ylabel('Control u - PWM')
        ax2[0,2].set_ylim(-255,255)
        ax[0,2].set_title('M3')

        fig.legend(loc="upper right", bbox_to_anchor=(1,1), bbox_transform=ax.transAxes)
        plt.show()
        

    
#------Shutdown methods-----------

    def close(self):
        global shutdown_flag
        shutdown_flag = True
        # Shutdown thread
        self.isrun = False
        time.sleep(2)
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
    dc.close()
    print('Closing gracefully. Bye Bye!')
    sys.exit()


if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    global dc
    dc = droidControl('/dev/ttyUSB0',38400,10,7,'f')
    dc.startReadThread()
    t0 = time.time()
            
    #Use GUI loop for online changes
    # Get user input 
    dc.LinearSpeed = 0.2 # m.s^-1
    dc.AngularSpeed = np.pi/2 # degrees.s^-1
    
    Heading = np.pi/2 # radians in robot frame
    Distance = 0.1 # metres
    Rotation = -0.5 * np.pi # radian in world frame
    
    dc.Kprop = 1.5 # Proportional gain
    dc.Kint = 40 # Integral gain
    dc.Kder = 0.001 # Derivative gain
    dc.runCommand = 0.0
    
#     dc.testVectorDrive(Distance, Heading)
    
    dc.testRotationDrive(Rotation)
    
    
    



#     # Save file
#     dc.saveOutput()
#     
#     # Plot measurements
#     dc.plotOutput()

    print('out of loop')
    dc.close()

    print('All done')
        

