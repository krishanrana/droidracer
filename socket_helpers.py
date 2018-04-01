#!/usr/bin/python3
'''
Author: Sam Armstrong
Last updated: 22/2/2018

Desciption:
    Some handy functions to make life with sockets in python 
    a lot easier. No need to worry about the recv() function 
    disconnecting before the job is through.

    Just give the SendMsg(sock,msg) a socket and bytestring 
    and receive it with RecvMsg(sock).

    Image send and receiving has been implemented with jpeg (requires cv2)
    and just plain old np arrays.

'''

import socket
import cv2
import struct
import logging
import numpy as np


'''
#####################################################################################
    Send and receive bytestrings
#####################################################################################
'''
def SendMsg(sock, msg):
    '''
    Sends an entire byte string.
    To be used in conjunction with RecvMsg()
    '''
    # Prefix each message with a 4-byte length (network byte order)
    msg = struct.pack('>I', len(msg)) + msg
    sock.sendall(msg)

def RecvMsg(sock):
    '''
    Receives an ENTIRE message in bytestring.
    To be used in conjunction with SendMsg()
    '''
    # Read message length and unpack it into an integer
    raw_msglen = RecvAll(sock, 4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]
    # Read the message data
    return RecvAll(sock, msglen)

'''
#####################################################################################
    Send and receive images
#####################################################################################
'''
def SendNumpy(sock, nparray, jpeg=False):
    '''
    Prepares and sends an numpy array over sockets.
    @params:
        nparray:    Numpy array of an image.
        jpeg:       Enables jpeg compression before sending.
                    Must be decoded on the receiving end.
    '''
    if jpeg:
        buf = cv2.imencode('.jpg', nparray)[1].tostring()
        SendMsg(sock, buf)
    else:
        # Get the shape of the array to send in header
        shape = nparray.shape
        dimensions = len(shape)
        #Send the number of dimensions, then the sizes.
        sock.sendall(struct.pack('>I', dimensions))
        # Convert to bytes
        for each in shape:
            sock.sendall(struct.pack('>I', each))
        # Send the actual array
        SendMsg(sock, nparray.tostring())

def RecvNumpy(sock, jpeg=False):
    '''
    Receives an numpy array over sockets.
    @params:
        sock:       The socket to listen on
        jpeg:       Enables jpeg decompression.
                    To be enabled if the transmission was compressed.
    '''
    if jpeg:
        buf = np.fromstring(RecvMsg(sock), np.uint8)
        nparray = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    else:
        # Receive header info to rebuild numpy array correctly
        num = struct.unpack('>I', RecvAll(sock,4))[0] # Number of dimensions
        shape = []
        for i in range(num):
            shape.append(struct.unpack('>I', RecvAll(sock,4))[0]) # Each dimension
        shape = tuple(shape)
        # Receive the array
        buf = RecvMsg(sock)
        logging.debug('Image received. Size: %d %d' % (shape[0], shape[1]))
        nparray = np.fromstring(buf, dtype='uint8').reshape(shape)

    return nparray


'''
#####################################################################################
    Keeps looping until all data is received. 
    Don't use this function outside, use the better ones 
    that send heading info instead.
#####################################################################################
'''
def RecvAll(sock, n):
    '''
    Helper function to recv n bytes or return None if EOF is hit.
    '''
    data = bytes()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data


if __name__ == '__main__':
    print("No tests defined.")