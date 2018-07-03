

class Navigation():
    def __init__(   self,
                    speed=1,
                    Komega=1,
                    Kh=1):
        self.speed = speed
        self.Komega = Komega
        self.Kh = Kh

    def processNav(self, heading, leftOffset, rightOffset, obstacle, obDist):
        
        # Determine angular velocity based on camera direction
        theta = (1.5708 - heading)
        omega =  theta * self.Komega
        
        speed = self.speed
            
        # Determine vehicle heading vector in radians from x right = 0
        #trackOffset = leftOffset - rightOffset
        
        # Create a vehicle direction vector
        #vector = (theta * Kh) + (trackOffset * Kt)
        
        vector = (theta * self.Kh)
        
        return speed, vector, omega