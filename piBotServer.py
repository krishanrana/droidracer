'''
PiBotServer class

    This PiBotServer class is run on the raspberry pi.

'''

import threading
import socket
import logging
import signal
import sys
import time
import numpy as np
import os
import serial

import piVideoStream
from socket_helpers import *
from constants import *
import auto


logging.basicConfig(level=logging.INFO,
                      format='[%(levelname)s] (%(threadName)-9s) %(message)s',)
shutdown_flag = False


class PiBotServer:
    '''
    #####################################################################################
        CONSTRUCTOR
    #####################################################################################
    '''
    def __init__(
                    self,
                    cam_resolution=(DEFAULT_CAM_W,DEFAULT_CAM_H),
                    port_cmds_a=PORT_CMDS_A,
                    port_cmds_b=PORT_CMDS_B,
                    port_camera=PORT_CAMERA,
                    serial_port='/dev/ttyUSB0',
                    baud=9600,
                ):

        self.speed = 0
        self.vector = 0
        self.omega = 0
        '''
        Robot states:
            0 = 'manual'
            1 = 'auto'
        '''
        self.state = 0
        # Used for sending debug images
        size = (int(cam_resolution[1]*CAM_SCALING),int(cam_resolution[0]*CAM_SCALING),3)
        self.debug_frame = np.empty(size, 'uint8')
        self.debug = False  
        
        # Ports
        self.port_cmds_a = port_cmds_a
        self.port_cmds_b = port_cmds_b
        self.port_camera = port_camera

        # Open serial comms to Arduino
        self.ser = serial.Serial(serial_port, baud)
        
        # Raspberry pi camera
        self.cam = 0
        self.cam_resolution = cam_resolution
        self.__camera_flag = False

        # Keep track of open ports to close when server is closed
        self.open_socks = []

        

        logging.debug("PiBotServer instance created")


    '''
    #####################################################################################
        SERVICES
    #####################################################################################
    '''

    def StartServers(self):
        # Starts the listener servers each in a fresh thread
        cmdsA_thr = threading.Thread(name="CmdsAListener", target=self.__Listen, args=(self.port_cmds_a,), daemon=False)
        cmdsB_thr =threading.Thread(name="CmdsBListener", target=self.__Listen, args=(self.port_cmds_b,), daemon=False)
        cam_thr = threading.Thread(name="CameraListener", target=self.__Listen, args=(self.port_camera,), daemon=False)
        cmdsA_thr.start()
        cmdsB_thr.start()
        cam_thr.start()

    def __Listen(self, port, num_of_connects=1):
        # Start a socket and bind to the listening port.
        # 0.0.0.0 is localhost but on all network interfaces.
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allows immediate reconnection in the event of program forcequitting
        s.bind(('0.0.0.0', port))
        logging.info("Server started on %s. Listning on port %d...", socket.gethostname(), port)

        s.listen(num_of_connects)   # Now wait for client connection.

        self.open_socks.append(s)   # Add listeners to sockets list. Need to close the listeners when shutting down the server.

        while (not shutdown_flag):
            conn, addr = s.accept() # Establish connection with client.
            logging.debug("Got connection from %s:%d", addr[0], addr[1])
            thr = threading.Thread(name=addr, target=self.__HandleClient, args=(conn, addr, port))
            thr.start()


    def __HandleClient(self, conn, addr, port):
        '''
        To be run in a new thread when a client connects.
        Choose what to do with the client data here.
        '''
        if port == self.port_camera:
            self.__SendCameraStream(conn)
        elif port == self.port_cmds_a:
            self.__ProcessCmdA(conn, addr)
        elif port == self.port_cmds_b:
            self.__ProcessCmdB(conn, addr)
        else:
            logging.debug("Unrecognised port: %d", port)


    def __ProcessCmdA(self, conn, addr):
        data = RecvMsg(conn)    # Receive our data
        conn.close()            # Close the connection
        logging.debug('Received data: %s', data)

        # Separate opCode from the payload. Payload hadling will
        # differ depending on the operation.
        cmds = data.decode().split(',')
        opCode = int(cmds[0])

        '''
        opCodes for commands A are:
            0   Mode selection ('auto','manual')
            1   Control raw motor powers            # NOT IMPLEMENTED
            2   Control speed/heading/omega

        '''
        if opCode == 0:
            if cmds[1].lower() == 'auto':
                logging.info("ENTERING AUTONOMOUS MODE")
                self.enterAutoMode()
            elif cmds[1].lower() == 'manual':
                logging.info("ENTERING MANUAL MODE")
                self.enterManualMode()
            else:
                logging.error("Unknown mode '%s'", cmds[1])
                

        elif opCode == 1:
            motor1 = int(cmds[1])
            motor2 = int(cmds[2])
            motor3 = int(cmds[3])
            self.setPower(motor1, motor2, motor3)


        elif opCode == 2:
            speed = int(cmds[1])
            vector = int(cmds[2])
            omega = int(cmds[3])
            self.setSpeed(speed, vector, omega)

        else:
            logging.warning("Ignoring unknown opCode: %d", opCode)

    def __ProcessCmdB(self, conn, addr):
        data = RecvMsg(conn)    # Receive our data
        logging.debug('Received data: %s', data)

        # Seperate opCode from the payload. Payload hadling will
        # differ depending on the operation.
        cmds = data.decode().split(',')
        opCode = int(cmds[0])

        '''
        opCodes for commands B are:
            0   Get the motor speeds

        '''
        if opCode == 0:
            logging.error("getSpeed not implemented yet")

        else:
            logging.warning("Ignoring unknown opCode: %d", opCode)
    
        conn.close()            # Close the connection


    def StartCameraStream(self):
        # Start the capture stream
        self.cam = piVideoStream.PiVideoStream(
                                            resolution=self.cam_resolution,
                                            vflip=True,
                                            hflip=True
                                            )
        self.cam.start()                    # Start the capture stream
        self.__camera_flag = True


    def __SendCameraStream(self, conn):
        while (self.__camera_flag): # Keep streaming
            if self.cam.frame_available:
                self.cam.frame_available = False
                if self.debug:
                    self.frame = self.debug_frame
                else:
                    self.frame = self.cam.read()
                SendNumpy(conn, self.frame, jpeg=True) # Send an image

        self.cam.stop()
        conn.close()


    def close(self):
        global shutdown_flag
        shutdown_flag = True
        self.__camera_flag = False

        # Close serial port
        self.ser.close()

        # Shutdown the listener sockets
        for s in self.open_socks:
            s.close()
       
        self.cam.stop()


    '''
    #####################################################################################
        STATE MACHINE MODES
    #####################################################################################
    '''
    def enterManualMode(self):
        self.state = 0
        auto.stop(self)

    def enterAutoMode(self):
        self.state = 1
        auto.begin(self) # Pass the droid object so auto can grab frames and setSpeeds 

    '''
    #####################################################################################
        SETTERS
    #####################################################################################
    '''
    def setPower(self, motor1, motor2, motor3):
        logging.error("Damn, sending raw power commands is not implemented yet")


    def setSpeed(self, speed, vector, omega):
        speed = str(speed)
        vector = str(vector)
        omega = str(omega)
        data_out = speed + "," + vector + "," + omega + "\n"
        self.ser.write(data_out.encode("ascii"))

    '''
    #####################################################################################
        GETTERS
    #####################################################################################
    '''
    def getSpeed(self):
        logging.error("Damn, receiving speed values is not implemented yet")
        
        return self.speed
    

'''
Captures the cntl+C keyboard command to close the services and free 
resources gracefully.
Allows the sockets and camera to be immediately reopened on next run without
waiting for the OS to close them on us.
'''
def signal_handler(signal, frame):
    global shutdown_flag
    shutdown_flag = True
    pb.close()
    print('Closing gracefully. Bye Bye!')


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    global pb
    pb = PiBotServer()
    pb.StartServers()
    pb.StartCameraStream()
    while not shutdown_flag:
        time.sleep(1)
