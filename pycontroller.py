# Pygame init Code from The Raspberry Blonde

import pygame, sys
import time


class pyJoystick():
    def __init__(self, windowSize = (200,200),windowPos = [0,32]):
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

        print ("There is " + str(self.axes) + " axes")
        print ("There is " + str(self.buttons) + " button/s")
        print ("There is " + str(self.hats) + " hat/s")

    def getAxis(self,number):
        # when nothing is moved on an axis, the VALUE IS NOT EXACTLY ZERO
        # so this is used not "if joystick value not zero"
        if self.joystick.get_axis(number) < -0.1 or self.joystick.get_axis(number) > 0.1:
          # value between 1.0 and -1.0
            print ("Axis value is %s" %(self.joystick.get_axis(number)))
            print ("Axis ID is %s" %(number))
     
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
    
    def startEventLoop(self):

        while True:
            for event in pygame.event.get():
              # loop through events, if window shut down, quit program
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            if self.axes != 0:
                for i in range(self.axes):
                    self.getAxis(i)
        #         print("axis {0}: value {1}".format(i,getAxis(i)))
            if self.buttons != 0:
                for i in range(self.buttons):
                    self.getButton(i)
        #         print("Button {0}: value {1}".format(i,getButton(i))) 
            if self.hats != 0:
                for i in range(self.hats):
                    self.getHat(i)
        #         print("Hat {0}: value {1}".format(i,getHat(i)))
            time.sleep(0.1)

if __name__ == "__main__":
    f710 = pyJoystick()
    f710.startEventLoop()
    
    