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

import piVideoStream
from socket_helpers import *

PORT_CMDS_A = 5000
PORT_CMDS_B = 5001
PORT_CAMERA = 5002

logging.basicConfig(level=logging.DEBUG,
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
                    ticks_per_rev=(200,200),
                    port_cmds_a=PORT_CMDS_A,
                    port_cmds_b=PORT_CMDS_B,
                    port_camera=PORT_CAMERA,
                ):

        self.speed = np.array([0,0,0])
        
        
        # Ports
        self.port_cmds_a = port_cmds_a
        self.port_cmds_b = port_cmds_b
        self.port_camera = port_camera
        
        # Raspberry pi camera
        self.cam = 0
        self.cam_resolution = (640,480)
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
            logging.info("Got connection from %s:%d", addr[0], addr[1])
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

        # Seperate opCode from the payload. Payload hadling will
        # differ depending on the operation.
        cmds = data.decode().split(',')
        opCode = int(cmds[0])

        '''
        opCodes for commands A are:
            0   Control motor power

        '''
        if opCode == 0:
            motor1 = int(cmds[1])
            motor2 = int(cmds[2])
            motor3 = int(cmds[3])
            self.setPower(motor1, motor2, motor3)

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
            speeds = self.getSpeed()
            msg = "%d,%d,%d" % (speeds[0],speeds[1],speeds[2])
            msg = msg.encode()
            SendMsg(conn, msg)

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
                self.frame = self.cam.read() # Might want to preprocess here on the raspberry pi?
                SendNumpy(conn, self.frame, jpeg=False) # Send an image

        self.cam.stop()
        conn.close()


    def close(self):
        global shutdown_flag
        shutdown_flag = True
        self.__camera_flag = False

        # Shutdown the listener sockets
        for s in self.open_socks:
            s.close()
       
        self.cam.stop()

    '''
    #####################################################################################
        SETTERS
    #####################################################################################
    '''
    def setPower(self, motor1, motor2, motor3):
        logging.error("Damn, havent got any motors programmed yet")

    '''
    #####################################################################################
        GETTERS
    #####################################################################################
    '''
    def getSpeed(self):
        logging.error("Don't got no speeds to give. Just sending random numbers")
        self.speed[0] = 12
        self.speed[1] = 45
        self.speed[2] = -87
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
