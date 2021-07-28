# Pygame init Code from The Raspberry Blonde

import pygame, sys
import time
import numpy as np
from DroidControl import droidControl 

class pyJoystick():
    def __init__(self, windowSize = (200,200),windowPos = [0,32]):
        self.dirX = 0
        self.dirY = 0
        self.rotL = 0
        self.rotR = 0
        self.stop = 0
        
        joystick_count = 0
        # setup the pygame window
        pygame.init()
        self.window = pygame.display.set_mode(windowSize, windowPos[0], windowPos[1])
        timeout = 0
        # Find joysticks
        while joystick_count == 0 and timeout < 10:
            joystick_count = pygame.joystick.get_count()
            print ("There are " + str(joystick_count) + " joystick/s: " + str(timeout)+ " tries.")
            timeout += 1
            time.sleep(1)
        if joystick_count == 0:
            # if no joysticks, quit program safely
            print ("Error, I did not find any joysticks")
            pygame.quit()
            sys.exit()
        else:
            # initialise joystick
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

        self.axes = self.joystick.get_numaxes()
        self.buttons = self.joystick.get_numbuttons()
        self.hats = self.joystick.get_numhats()


    def getAxis(self,number):
        # when nothing is moved on an axis, the VALUE IS NOT EXACTLY ZERO
        # so this is used not "if joystick value not zero"
        if self.joystick.get_axis(number) < -0.1 or self.joystick.get_axis(number) > 0.1:
          # value between 1.0 and -1.0
            print ("Axis value is %s" %(self.joystick.get_axis(number)))
            print ("Axis ID is %s" %(number))
        else:
            print(self.joystick.get_axis(number))
     
    def getButton(self,number):
        # returns 1 or 0 - pressed or not
        if self.joystick.get_button(number):
          # just prints id of button
          print ("Button ID is %s" %(number))

    def getHat(self,number):
        if self.joystick.get_hat(number) != (0,0):
          # returns tuple with values either 1, 0 or -1
            print ("Hat value is %s, %s" %(self.joystick.get_hat(number)[0],self.joystick.get_hat(number)[1]))
            print ("Hat ID is %s" %(number))
    
    def getButtonMapping(self):

        while True:
            for event in pygame.event.get():
              # loop through events, if window shut down, quit program
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            if self.axes != 0:
                for i in range(self.axes):
                    self.getAxis(i)
            if self.buttons != 0:
                for i in range(self.buttons):
                    self.getButton(i)
            if self.hats != 0:
                for i in range(self.hats):
                    self.getHat(i)
            time.sleep(0.1)
            
    def getJoystickInput(self):
        
        for event in pygame.event.get():
              # loop through events, if window shut down, quit program
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        if self.axes != 0:
            dirX = np.round(self.joystick.get_axis(0),decimals=3)
            if (dirX < -0.1) or (dirX > 0.1):
                self.dirX = dirX
            else: self.dirX = 0
            
            dirY = np.round(self.joystick.get_axis(1),decimals=3)
            if (dirY < -0.1) or (dirY > 0.1):
                self.dirY = dirY
            else:
                self.dirY = 0
            self.rotL = np.round((self.joystick.get_axis(2) + 1.0)/2,decimals=3)
            self.rotR = np.round((self.joystick.get_axis(5) + 1.0)/2,decimals=3)
            
        if self.buttons != 0: 
            self.stop = self.joystick.get_button(1)
        
    def initTelemetry(self):
        return 0

    
    def updateTelemetry(self):
        # Get current droid position and rotation from droidControl
        
        # From Controller

        # From Encoder

        # From IMU

        # From Fusion Estimate

        # Set sprite positions

        # Update window
        return 0


class DroidSprite(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.image = pygame.Surface((32, 32))
        self.image.fill(WHITE)
        self.rect = self.image.get_rect()  # Get rect of some size as 'image'.
        self.position = [0, 0]
        self.rotation = 0

    def update(self):
        self.rect.move_ip(*self.velocity)
     
            
            
if __name__ == "__main__":
    f710 = pyJoystick()
    
    dc = droidControl()
    dc.LinearSpeed = 1
    dc.AngularSpeed = 3.1415
    dc.runCommand = True
    dc.Kprop = 1.5 # Proportional gain
    dc.Kint = 40 # Integral gain
    dc.Kder = 0.001 # Derivative gain
#     f710.getButtonMapping()
    
    while f710.stop == 0:
#         t0 = time.time()
        f710.getJoystickInput()
        print([f710.dirX, -f710.dirY, f710.rotL, f710.rotR])
#         print(time.time() - t0)
        dc.processCommands(f710.dirX, f710.dirY, f710.rotL, f710.rotR)
        dc.driveDroid()
        time.sleep(0.05)
        
    dc.runCommand = False
    dc.writeSerial()
    dc.close()
    time.sleep(2)
    sys.exit()
    
    