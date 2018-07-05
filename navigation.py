import numpy as np

class Navigation():
    def __init__(   self,
                    speed=0.01,
                    Komega=0.2,
                    Kh=1):
        self.speed = speed
        self.Komega = Komega
        self.Kh = Kh

    def processNav(self, heading, leftOffset, rightOffset, obstacle, obDist):
        
        # Determine angular velocity based on camera direction
        theta = (heading - np.pi/2)
        omega =  theta * self.Komega
        
        speed = self.speed
            
        # Determine vehicle heading vector in radians from x right = 0
        #trackOffset = leftOffset - rightOffset
        
        # Create a vehicle direction vector
        #vector = (theta * Kh) + (trackOffset * Kt)
        
        vector = (np.pi/2 * self.Kh)
        
        return speed, vector, omega