import numpy as np

# Velocity in m/s robot frame
# Angular velocity in rad/s

class Navigation():
    def __init__(   self,
                    maxVelocity=0.2,
                    Komega=0.2,
                    maxOmega = 0.1,
                    Kp=1):
        self.maxVelocity = maxVelocity
        self.Komega = Komega
        self.maxOmega = maxOmega
        self.Kp = Kp

    def processNav(self, heading, leftOffset, rightOffset, obstacle, obDist):
        # Determine angular velocity based on camera direction (90 deg to robot frame)
        theta = (heading - np.pi/2)

        # Cap theta to FOV; remove when using potential fields
        LIM = 30*np.pi/180
        if theta > LIM:
            theta = LIM
        elif theta < -LIM:
            theta = -LIM

        print("Theta: "+str(theta * 180/np.pi))
        omega =  self.maxOmega * theta * self.Komega
        print("Omega: "+str(omega))
        robotSpeed = self.maxVelocity * (1.0 - (0.8 * np.absolute(omega)/self.maxOmega))
        
        #self.robotHeading = self.Kp *potentialField.getRobotHeading(self,things) 
        # Determine vehicle heading vector in radians from x right = 0
        #trackOffset = leftOffset - rightOffset
        
        # Create a vehicle direction vector
        #vector = (theta * Kh) + (trackOffset * Kt)
        
        robotHeading = (np.pi/2 * self.Kp)
        
        return robotSpeed, robotHeading, omega