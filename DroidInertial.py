# -*- coding: utf-8 -*-
"""
Created on Mon Jan 18 11:29:54 2021

@author: doug palmer

NOTE: Some code from MPU6050.py and iC2 library by Jeff Rowland via Geir Istad
"""
# Import packages
from threading import Thread
import numpy as np
import sys
from MPUClass.MPU6050 import MPU6050 # Rewrite class using native numpy
import logging
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
#import PySimpleGUI as psg
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as spr
import logging

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
    
    def __init__(self,biasType = 'External',dataType = 'raw'):
        
        # Flags for thread operation
        self.dmpReceiving = False
        self.dmpIsRun = True
        self.dmpThread = None
        
        self.imuReceiving = False
        self.imuIsRun = True
        self.imuThread = None
        
        self.newData = False
        
        self.TimeKminus1 = time.time()        
        self.TimeK = time.time()
        #self.gravity = np.array([0,0,9.80665])
        self.gravity = 9.80665
        

        # User set internal or external bias correction
        self.biasType = biasType
        self.accelBias = np.array([0,0,0])
        self.gyroBias = np.array([0,0,0])
        self.dataType = dataType
        self.FIFO_buffer = [0]*42
        # DMP states
        self.dmpAccel = np.array([[0,0,0]])
        self.dmpEulerTheta = np.array([[0,0,0]])
        
        # Raw readings from IMU
        self.rawAccel = np.array([0,0,0])
        self.rawAccelMinus1 = np.array([0,0,0])
        
        self.rawOmega = np.array([0,0,0])
        self.rawOmegaMinus1 = np.array([0,0,0])
        
        # Processed output 
        self.accel = np.array([0,0,0])
        self.accelMinus1 = np.array([0,0,0])
        self.velocity = np.array([0,0,0])
        self.velocityMinus1 = np.array([0,0,0])
        self.displacement = np.array([0,0,0])
        self.displacementMinus1 = np.array([0,0,0])
        
        self.omega = np.array([0,0,0])
        self.omegaMinus1 = np.array([0,0,0])   
        self.theta = np.array([0,0,0])
        self.thetaMinus1 = np.array([0,0,0])
             
        # Storage
        self.imuOut = []
        
        i2c_bus = 1
        device_address = 0x68

        # Check for IMU bias file
        self.loadCalData(loadBiasData =True)

        # Initialise device using inheritance
        MPU6050.__init__(self, i2c_bus, device_address)
        
        # Initialise DMP
        self.dmp_initialize()
        # Start DMP
        self.set_DMP_enabled(True)
        self.FIFO_packet_size = self.DMP_get_FIFO_packet_size()
                
        logger.debug(('Interrupt status: %s ' % hex(self.get_int_status())))
        logger.debug('FIFO Packet size: %s ' % self.FIFO_packet_size)
        logger.debug(('FIFO count: %s ' % hex(self.get_FIFO_count())))
        
        if self.biasType == 'Internal':
            self.set_x_accel_offset(self.accelBias[0])
            self.set_y_accel_offset(self.accelBias[1])
            self.set_z_accel_offset(self.accelBias[2])
            self.set_x_gyro_offset(self.gyroBias[0])
            self.set_y_gyro_offset(self.gyroBias[1])
            self.set_z_gyro_offset(self.gyroBias[2])
            logger.debug('IMU biases compensated INTERNALLY')
        else:
            logger.debug('IMU biases compensated EXTERNALLY')
            
        if self.dataType == 'dmp':   
            self.startDMPThread()
            logger.debug('Data from DMP')
        else:
            self.startIMUThread()
            logger.debug('Data from IMU')
              
    def startIMUThread(self):
        if self.imuThread == None:
            self.imuThread = Thread(target=self.readIMUthread, daemon = True)
            #self.thread.daemon = True
            self.imuThread.start()
            # Block till we start receiving values
            while self.imuReceiving != True:
                time.sleep(0.1)
                
    def readIMUthread(self):
        time.sleep(0.5)  # give some buffer time for retrieving data
        logger.debug('readIMUThread starting..')
        while (self.imuIsRun):
            self.TimeKminus1 = self.TimeK
            self.TimeK = time.time()
            # : Store as numpy arrays as we will be using matrix operations for Kalman filter
            rawAccel = np.array([self.get_acceleration()])
            rawOmega = np.array([self.get_rotation()])
            # Manually remove bias if needed
            if self.biasType != 'Internal':
                rawAccel = rawAccel - (self.accelBias * 8.95)
                rawOmega = rawOmega - (self.gyroBias * 8.95)
            # Scale and Convert to ms-2
            self.rawAccel = rawAccel / 2**14  * self.gravity
            self.rawOmega = rawOmega / 131.0
            self.imuReceiving = True
            self.newData = True
            time.sleep(0.000001)
               
    def startDMPThread(self):
        if self.dmpThread == None:
            self.dmpThread = Thread(target=self.readDMPThread,daemon = True)
            #self.thread.daemon = True
            self.dmpThread.start()
            # Block till we start receiving values
            while self.dmpReceiving != True:
                time.sleep(0.1)
            
    
    def readDMPThread(self):    # retrieve data
        time.sleep(0.5)  # give some buffer time for retrieving data
        self.reset_FIFO()
        logger.debug('readDMPThread starting..')
        while (self.dmpIsRun):
            FIFO_count = self.get_FIFO_count()
            mpu_int_status = self.get_int_status()
        
            while FIFO_count < self.FIFO_packet_size:
                FIFO_count = self.get_FIFO_count()
                time.sleep(0.000001)
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
                    time.sleep(0.000001)
                
                self.TimeKminus1 = self.TimeK
                self.TimeK = time.time()
                self.FIFO_buffer = self.get_FIFO_bytes(self.FIFO_packet_size)
                self.reset_FIFO()
                self.dmpReceiving = True
                self.newData = True
                time.sleep(0.000001)
            
    def processInertial(self):
        self.inertialDt = self.TimeK - self.TimeKminus1
        
        if self.dataType == 'dmp':
            # Preserves data and timestamp for data regardless of polling frequency
            if self.newData == True:
                self.dmpAccelMinus1 = self.dmpAccel
                self.dmpEulerThetaMinus1 = self.dmpEulerTheta
                self.omegaMinus1 - self.omega
                rawDMPAccel = self.DMP_get_acceleration_int16(self.FIFO_buffer)          
                self.dmpQuaternion = self.DMP_get_quaternion(self.FIFO_buffer)
                self.dmpRawAccel = np.array([[rawDMPAccel.x,rawDMPAccel.y,rawDMPAccel.z]])  
                self.dmpGrav = self.DMP_get_gravity(self.dmpQuaternion)
                linAccel = self.DMP_get_linear_accel(rawDMPAccel, self.dmpGrav)
                self.dmpAccel = (np.array([[linAccel.x / 8192.0,linAccel.y / 8192.0,linAccel.z / 8192.0]])) * self.gravity         
                theta = self.DMP_get_euler_roll_pitch_yaw(self.dmpQuaternion, self.dmpGrav)
                self.dmpEulerTheta = np.array([theta.x, theta.y, theta.z])
                self.newData = False;
            # Update droid state
            self.accel = self.dmpAccel
            self.accelMinus1 = self.dmpAccelMinus1
            self.theta = self.dmpEulerTheta
            self.thetaMinus1 = self.dmpEulerThetaMinus1
            # Calculate rough velocity through differentiation, improve with KF
            self.omega = (self.theta - self.thetaMinus1) / self.inertialDt
            
        else:
            if self.newData == True:
                self.rawAccelMinus1 = self.rawAccel
                self.rawOmegaMinus1 = self.rawOmega
                self.thetaMinus1 = self.theta
                self.newData = False;
            
            self.accel = self.rawAccel
            self.accelMinus1 = self.rawAccelMinus1
            self.omega = self.rawOmega
            self.omegaMinus1 = self.rawOmegaMinus1
            # Calculate rough pose through integration, improve with KF and magnetometer
            self.theta = self.thetaMinus1 + ((self.omega + self.omegaMinus1)/2) * self.inertialDt
        
        # Integrate acceleration
        self.velocityMinus1 = self.velocity
        self.velocity =  (self.accel + self.accelMinus1)/2 * self.inertialDt + self.velocityMinus1
        self.displacementMinus1 = self.displacement
        self.displacement =  (self.velocity + self.velocityMinus1)/2 * self.inertialDt + self.displacementMinus1


    def storeIMUdata(self):
        temp = self.rawAccel.tolist() + self.rawOmega.tolist() +  [self.TimeK]
        self.imuOut.append(temp)
        
    # TODO: Wrap calibration functions in seperate calibration class
    def recordStaticIMU(self, sampleLength = 1000, saveData = True):       
        # Ensure that existing biases are removed before calibration
        self.set_x_accel_offset(0)
        self.set_y_accel_offset(0)
        self.set_z_accel_offset(0)         
        
        poses = {'pose 1':(0,'all'),
                 'pose 2':(1,'1'),
                 'pose 3':(2,'1'),
                 'pose 4':(1,'2'),
                 'pose 5':(2,'2'),
                 'pose 6':(1,'3'),
                 'pose 7':(2,'3'),
                 'pose 8':(1,'1 & 2'),
                 'pose 9':(2,'1 & 2'),
                 'pose 10':('1 & 2','1 & 2'),
                 'pose 11':('2 & 1','1 & 2'),
                 'pose 12':('cushion','all'),
                 'pose 13':('cushion','all'),
                 'pose 14':('cushion','all'),
                 'pose 15':('cushion','all'),}
        
        self.meanAccel=[]
        self.varAccel=[]
        self.accelData = np.array([[0,0,0]])
        
        # Get data at various poses
        for pose, blocks in poses.items():
            print('\n','Set up robot in',pose, 'with',blocks[0], 'blocks under wheel ',blocks[1],'\n')
            input('Press any key to continue, ctrl C to quit')
            # Get initial reading
            self.processInertial()
            self.poseAccel = self.accel
            t00 = time.time()
            while len(self.poseAccel) < sampleLength:
                # Get raw IMU data
                self.processInertial()
                self.poseAccel = np.concatenate((self.poseAccel, self.accel),axis=0)
                print('.', end="", flush=True)
                time.sleep(0.005)
                
            
            calTime = time.time()-t00
            meanPoseAccel = np.mean(self.poseAccel,axis=0)
            varPoseAccel = np.var(self.poseAccel,axis=0)
            self.accelData = np.concatenate((self.accelData, self.poseAccel),axis=0)
            self.meanAccel.append(meanPoseAccel)
            self.varAccel.append(varPoseAccel)
            
            print(len(self.poseAccel),' samples collected in %0.2f seconds'%  calTime)
            print('\n')
            print('Mean accel (x,y,z) = ', meanPoseAccel)
            print('Variance accel (x,y,z) = ', varPoseAccel,'\n'*3)
            # Optimse for biases
        if saveData:
            np.savetxt('staticData.csv',self.accelData[1:],delimiter=',')
            logger.info('Saving static data to file: %s entries' % len(self.accelData))
            
    def loadCalData(self, loadBiasData = False, loadStaticData=False):
        if loadStaticData is True:       
            try:
                self.accelData = np.loadtxt('/home/pi/droidracer/staticData.csv', delimiter=',', dtype = "float")
                logger.info('staticData.csv loaded')
            except:
                logger.info('staticData.csv file not found')
                userIO = input('\n Press y to perform static calibration or any other key to Quit \n')
                if userIO == 'y':
                    self.recordStaticIMU()
                else:
                    logger.info('Quitting script')
                    exit()
        else:
            pass
        
        if loadBiasData is True:       
            try:
                IMUbiases = np.loadtxt('/home/pi/droidracer/imuBiases.csv', delimiter=',', dtype = "int")
                logger.info('Initialising IMU biases to file.')
                # Use existing IMU bias            
                self.accelBias = np.array(IMUbiases[0:3])
                self.gyroBias = np.array(IMUbiases[3:6])
            
            except:
                self.accelBias = np.array([0,0,0])
                self.gyroBias = np.array([0,0,0])
                logger.warning('No Bias file found. Static Calibration required')

        else:
            pass
        # Check for IMU Offset file (Not yet implemented)
#                     try:
#                 self.imuOffset = np.loadtxt('/home/pi/droidracer/MPUClass/imuOffsets.csv', delimiter=',', dtype = "int")
#                 logger.info('Initialising IMU offsets to file.')
#             except:
#                 self.imuOffset = np.array([0,0,0]) # Placeholder, run cal routine
#                 logger.warning('No Offset file found. Dynamic calibration required')
  
    def calStatic(self, getStaticData = False, optType = 'norm', saveData = True):

        
        if getStaticData is True:
            logger.info('Get ready to acquire static calibration data from robot!')
            self.recordStaticIMU()
        else:
            self.loadCalData(loadStaticData = True)
                       
        if optType is 'norm':
            b0 = np.array([0,0,0])
            obsA = self.accelData
            grav = -self.gravity
            self.biasEstimate = least_squares(self.costFunction, b0, method='trf',ftol=1e-13, verbose=2, args=(grav, obsA))
            
        elif optType is 'rotation':
            b0 = np.array([0,0,0])
            obsA = self.accelData
            grav = np.array([[0,0,-self.gravity]]* 900)
            # Make call to non-lin least squares for parameter estimation
            self.biasEstimate = least_squares(self.costFunction2, b0, method='trf',ftol=1e-13, verbose=2, args=(grav, obsA))
        

        logger.info('Bias estimation complete: ')
        self.accelBias = self.biasEstimate.x * 16384.0  / self.gravity
        logger.debug(self.accelBias)
        
        if saveData:
            # TODO: This is hacky, consider gyro info seperately
            saveFile = [self.accelBias[0],self.accelBias[1],self.accelBias[2],0,0,0]
            np.savetxt('imuBiases.csv',saveFile,delimiter=',')
            logger.info('Saving static accelerometer biases to file')
            
    def costFunction(self, bias, grav, obs):
                
        Ax = obs[:,0]
        Ay = obs[:,1]
        Az = obs[:,2]
        Bx = bias[0]
        By = bias[1]
        Bz = bias[2]
        # Find residual : E = norm(g) - norm(A-B)
        cost = self.gravity - np.sqrt((Ax - Bx)**2 + (Ay - By)**2 + (Az-Bz)**2)
        return cost
    
    def costFunction2(self, bias, grav, obs):
        #THIS needs much work, align vector attempts to find true rotation. This requires adjustment to bias
        sampleLength = 1000
        A = np.array([[0,0,0]])
        B = np.array([[0,0,0]])
        
        for poseIdx in range(9):        
            # Calculate closest rotation matrix using current estimate of bias
            start = (poseIdx)*sampleLength
            end = start + 1000
            tempObs = obs[start:end,:]
            C = spr.align_vectors(tempObs - bias, grav)
            # Concatenate to variable for each pose
            tempBias = obs[start:end,:] - C[0].apply(grav)
            A = np.concatenate((A,tempObs),axis=0)
            B = np.concatenate((B,tempBias),axis=0)
            
            # Get value of bias estimate that allows for proper rotation
        
        Ax = A[1:-1,0]
        Ay = A[1:-1,1]
        Az = A[1:-1,2]
        Bx = B[1:-1,0]
        By = B[1:-1,1]
        Bz = B[1:-1,2]
        # Find residual : E = norm(g) - norm(A-B)
        cost = self.gravity - np.sqrt((Ax - Bx)**2 + (Ay - By)**2 + (Az-Bz)**2)
        return cost
    
        
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
                self.processInertial()
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
        print("Test read time...")
        counter = 0
        t0 = time.time()
        self.processInertial()
        accel = np.array([[0,0,0]])
        accel = np.concatenate((accel, self.accel),axis=0)
        while counter < 100:
            self.processInertial()
            accel = np.concatenate((accel, self.accel),axis=0)
            counter += 1
            time.sleep(0.01)
            
        print('100 values in: %f' % (self.TimeK - t0))
        print('Average read time: %f' % ((self.TimeK - t0) / 100))
        self.plotDataSet(accel)

        
    def plotDataSet(self,data):
        
        fig, ax = plt.subplots(3,1,figsize=(12,9))

        ax[0].plot(data[:,0],'r',label='Accel - x (m/s^2)')
        ax[0].set_ylabel('Accel - (m/s^2)')
        ax[1].plot(data[:,1],'g',label='Accel - y (m/s^2)')
        ax[2].plot(data[:,2],'b',label='Accel - z (m/s^2)')
#         fig.legend(loc="upper right", bbox_to_anchor=(1,1), bbox_transform=ax.transAxes)

        plt.show()
       
    def testLinProp(self):
        print("Test linear propagation")
     
        # Warm up IMU
        print('Warming up...')
        for idx in range(60): 
            print('temp: {0}'.format(self.get_temp()))
            time.sleep(0.1)
        
        self.processInertial()
        time.sleep(1)
        # Initialise storage containers   
        accel = np.array([[0,0,0]])
        disp = np.array([[0,0,0]])       
        vel = np.array([[0,0,0]])
        disp = np.concatenate((disp, self.displacement),axis=0)
        vel = np.concatenate((vel, self.velocity),axis=0)
        accel = np.concatenate((accel, self.accel),axis=0)
        
        t0 = time.time()
        counter = 0
        while counter < 1000:
            self.processInertial()
            disp = np.concatenate((disp, self.displacement),axis=0)
            vel = np.concatenate((vel, self.velocity),axis=0)
            accel = np.concatenate((accel, self.accel),axis=0)
            counter +=1
            time.sleep(0.02)
            
        tend = time.time()
        
        print('Time: %f' % (tend-t0))
        print('xAc bias: %0.4f' % np.median(accel[:,0]))
        print('yAc bias: %0.4f' % np.median(accel[:,1]))
        print('zAc bias: %0.4f' % np.median(accel[:,2]))
        
        fig, ax = plt.subplots(3,3,figsize=(12,9))
        
        ax[0,0].plot(disp[:,0], 'r',label='Displacement - x (m)')
        ax[0,0].set_ylabel('Displacement - m')
        ax[0,1].plot(disp[:,1], 'r',label='Displacement - y (m)')
        ax[0,2].plot(disp[:,2], 'r',label='Displacement - z (m)')
        
        ax[1,0].plot(vel[:,0],'g',label='Velocity - x (m/s)')
        ax[1,0].set_ylabel('Velocity - (m/s)')
        ax[1,1].plot(vel[:,1],'g',label='Velocity - y (m/s)')
        ax[1,2].plot(vel[:,2],'g',label='Velocity - z (m/s)')
        
        ax[2,0].plot(accel[:,0],'b',label='Accel - x (m/s^2)')
        ax[2,0].set_ylabel('Accel - (m/s^2)')
        ax[2,1].plot(accel[:,1],'b',label='Accel - y (m/s^2)')
        ax[2,2].plot(accel[:,2],'b',label='Accel - z (m/s^2)')
        
        plt.show()
        
    def estPose(self,obs,bias,sampleLength=1000):
        
        # THIS NEEDS MUXCH WORK!!!!!!!!!
        # Calculate robot pose using gravity vector from dataset, Add magnwtometer later  
        Ax = obs[:,0]
        Ay = obs[:,1]
        Az = obs[:,2]
        B = bias*self.gravity /2**14
        grav = np.array([[0,0,-self.gravity]]*sampleLength)
        rotOut=[] 
#         A = np.array([[0,0,0]])
        for poseIdx in range(9):        
            # Calculate closest rotation matrix using current estimate of bias
            startIdx = (poseIdx)*sampleLength
            endIdx = startIdx + sampleLength
            tempObs = obs[startIdx:endIdx,:]
            rotObj = spr.align_vectors(tempObs - B, grav)
            # Concatenate to variable for each pose
#             rotation = C[0].as_euler('zyx',degrees=True)
#             A = np.concatenate((A,rotation),axis=0)
            rotOut.append(rotObj[0])    
        return rotOut
    
    def plotPose(self,pose, poseNo):
        fig = plt.figure(figsize=(9,9))
        ax = fig.gca(projection='3d')
        
        #Set origin of world frame
        x0 = [0,0,0]
        y0 = [0,0,0]
        z0 = [0,0,0]
        
        # Create world frame arrows      
        u0 = [1,0,0]
        v0 = [0,1,0]
        w0 = [0,0,1]
        
        colours = ['r','g','b']
        
        plotScale = 10
        ax.quiver(x0,y0,z0,u0,v0,w0,color=[[0.7,0.7,0.7]]*3,length = plotScale,arrow_length_ratio=0.1)
        ax.text(plotScale,0,0.1,r'$X_w$')
        ax.text(0,plotScale,0,r'$Y_w$')
        ax.text(0,0,plotScale,r'$Z_w$')
        
        # Create gravity vector
        ax.quiver(0,0,0,0,0,-9.8,color='r',arrow_length_ratio=0.2)
        ax.text(0,0,-10,'Gravity',color ='r')
        
        # Create rotated coordinate system
        uR, vR, wR = pose.apply([u0,v0,w0]) * 8
        ax.quiver(x0,y0,z0,uR,vR,wR,color=colours,length = 1,arrow_length_ratio=0.1)
        ax.text(uR[0],uR[1],uR[2],r'$X_r$',color ='r')
        ax.text(vR[0],vR[1],vR[2],r'$Y_r$',color ='g')
        ax.text(wR[0],wR[1],wR[2],r'$Z_r$',color ='b')
        
        ax.set_xlim3d(-10,10)
        ax.set_ylim3d(-10,10)
        ax.set_zlim3d(-10,10)
        
        ax.set_xlabel(r'$A_x - ms^{-2}$')
        ax.set_ylabel(r'$A_y - ms^{-2}$')
        ax.set_zlabel(r'$A_z - ms^{-2}$')
        
        Euler = pose.as_euler('zyx',degrees=True)        
        ax.set_title(r'Pose: {0} $R_x$: {1:.2f} $R_y$: {2:.2f} $R_z$: {3:.2f}'.format(poseNo, Euler[0], Euler[1], Euler[2]))

    def checkRotation(self,R):
        R = pose.as_matrix()
        deter = np.round(np.linalg.det(R),decimals=4)
        if deter == 1:
            isRot = 'rotation'
        elif deter == -1:
            isRot = 'reflection'
        else:
            isRot = '??'
        return isRot, deter
    #------Shutdown methods-----------

    def close(self):
        global shutdown_flag
        shutdown_flag = True
        # Shutdown thread
        self.dmpIsRun = False
        self.imuIsRun = False
        time.sleep(1) 
        logger.debug('DroidInertial releasing resources')

if __name__ == "__main__":
    di = droidInertial(biasType = 'Internal',dataType = 'raw')
#     di.testReadSpeed()
#     di.calStatic(getStaticData = False)
#     di.plotDataSet(di.accelData)
    di.testLinProp()
#     di.loadCalData(loadStaticData = True)
#     rotations = di.estPose(di.accelData, di.accelBias)
#     for poseNo,pose in enumerate(rotations,start=1):
#         di.plotPose(pose,poseNo)
# #         isRot, deter = di.checkRotation(pose)
# #         print('Pose {0} is a {1}, determinant ={2:.2f}'.format(poseNo,isRot, deter))
#         
    di.close()
    plt.show()
        
        
        
         