'''
PiBotClient class

    This PiBotClient class is used to connect to a remote 
    piBotServer instance running on the raspberry pi.
'''

import threading
import socket
import logging
import signal
import sys
import time
import numpy as np

from socket_helpers import *

PORT_CMDS_A = 5000
PORT_CMDS_B = 5001
PORT_CAMERA = 5002
DEFAULT_HOST = '0.0.0.0'


logging.basicConfig(level=logging.DEBUG,
                      format='[%(levelname)s] (%(threadName)-9s) %(message)s',)


class PiBotClient:

    '''
    #####################################################################################
        CONSTRUCTOR
    #####################################################################################
    '''
    def __init__(   
                    self, 
                    host=DEFAULT_HOST, 
                    cam_resolution=(2592,1944),
                    port_cmds_a=PORT_CMDS_A,
                    port_cmds_b=PORT_CMDS_B,
                    port_camera=PORT_CAMERA,
                ):

        '''
        #####################################################################################
            NETWORKING STUFF
            Port usage:
                self.port_cmds_a:   One-way communication. Not for returning data.
                self.port_cmds_b:   Requests for data from the server. Blocks until it gets a response.
                self.port_camera:   Used for getting camera feed only. Not adjusting camera settings.
        #####################################################################################
        '''
        self.host = host
        self.port_cmds_a = port_cmds_a
        self.port_cmds_b = port_cmds_b
        self.port_camera = port_camera


        # Camera stuff
        self.__cam_stream_enable = False
        self.cam_resolution = cam_resolution
        self.frame = np.empty((cam_resolution[0],cam_resolution[1],3), 'uint8')
        self.frame_available = False

        logging.debug("Created instance of PiBotClient")

    '''
    #####################################################################################
        SERVICES
    #####################################################################################
    '''
    def StartCamStream(self):
        threading.Thread(name="CameraStreamReceiver", target=self.__StartCamStream, daemon=False).start()
        
    def __StartCamStream(self):
        '''
        Continuously receives images from the server 
        and saves the current frame.
        '''
        logging.debug("Starting camera stream")
        self.__cam_stream_enable = True
        s = socket.socket()
        s.connect((self.host, self.port_camera))

        while(self.__cam_stream_enable):
            self.frame = RecvNumpy(s, jpeg=False)
            self.frame_available = True
            
        s.close()

    def StopCamStream(self):
        logging.debug("Stopping camera stream")
        self.__cam_stream_enable = False


    '''
    #####################################################################################
        SETTERS
        Use to send values to the server.
    #####################################################################################
    '''

    def setPower(self, motor1, motor2, motor3):
        opCode = 0
        msg = "%d,%d,%d" % (motor1,motor2,motor3)   # Prepare the comma-separated values
        self.__sendCmdA(opCode, msg)                # Use the helper function to send the cmd
        

    '''
    #####################################################################################
        GETTERS
    #####################################################################################
    '''
    def getSpeed(self):
        opCode = 0
        data = self.__sendCmdB(opCode)  # Doesn't need a msg. Server will know by port and opCode what to do
        return data


    '''
    #####################################################################################
        HELPER FUNCTIONS - SHOULDN'T BE USED OUTSIDE THE CLASS
    #####################################################################################
    '''
    def __sendCmdA(self, opCode, msg):
        '''
        Helper function to send a command to the server.
        '''
        s = socket.socket()                         # Load a new socket
        s.connect((self.host, self.port_cmds_a))    # Attempt connection
        msg = "%d,%s" % (opCode,msg)                # Append opCode to front of the msg
        msg = msg.encode()                          # Convert to bytestring
        SendMsg(s, msg)                             # Send
        s.close()                                   # Done

    def __sendCmdB(self, opCode, msg=""):
        '''
        Helper function to send a command to the server.
        '''
        s = socket.socket()                         # Load a new socket
        s.connect((self.host, self.port_cmds_b))    # Attempt connection
        msg = "%d,%s" % (opCode,msg)                # Append opCode to front of the msg
        msg = msg.encode()                          # Convert to bytestring
        SendMsg(s, msg)                             # Send
        data = RecvMsg(s)                           # Wait for server to send stuff back
        s.close()                                   # Done
        return data.decode().split(',')             # Return the info as an array of strings



'''
Captures the cntl+C keyboard command to close the services and free 
resources gracefully.
Allows the sockets immediately reopened on next run without
waiting for the OS to close them on us.
'''
def signal_handler(signal, frame):
    global shutdown_sig
    shutdown_sig = True
    print('Closed gracefully. Bye Bye!')


'''
Test the class or connection.
'''
if __name__ == '__main__':
    import cv2
    shutdown_sig = False
    signal.signal(signal.SIGINT, signal_handler)

    pb = PiBotClient(host=DEFAULT_HOST)
    pb.StartCamStream()

    while not shutdown_sig:
        if pb.frame_available:
            pb.frame_available = False
            cv2.imshow('This is my window', pb.frame)
            cv2.waitKey(100)
    input()