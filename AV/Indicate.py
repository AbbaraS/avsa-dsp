import sys
sys.path.insert(0, '/home/pi/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')
import time
from Led import *

class Indicate:
    def __init__(self):
        self.led=Led()
        
    '''
    RGB colours: 
    Red: 255, 0, 0
    Orange: 255, 50, 0
    
    ''' 
    
    
    '''
    Vehicle LED locations:
    
    Right       Left
          Back
    0x02        0x04
    0x01        0x08
    0x80        0x10
    0x40        0x20
          Front
    '''
    
    def indicate_left(self):
        '''
        Indicates using two left LED lights in Orange 
        '''
        self.led.colorWipe(self.led.strip, Color(0, 0, 0))
        self.led.ledIndex(0x04, 255, 50, 0)      
        self.led.ledIndex(0x20, 255, 50, 0)         
        time.sleep(0.1)
        self.led.colorWipe(self.led.strip, Color(0,0,0))
        
    def indicate_right(self):
        '''
        Indicates using two right LED lights in Orange 
        '''
        self.led.colorWipe(self.led.strip, Color(0, 0, 0))
        self.led.ledIndex(0x02, 255, 50, 0)      
        self.led.ledIndex(0x40, 255, 50, 0)   
        time.sleep(0.1)
        self.led.colorWipe(self.led.strip, Color(0,0,0)) 
        
    def indicate_stop(self):
        '''
        Indicates using 2 front and 2 back LED lights in Red  
        '''
        self.led.colorWipe(self.led.strip, Color(0, 0, 0))
        self.led.ledIndex(0x02, 255, 0, 0)      
        self.led.ledIndex(0x04, 255, 0, 0)      
        self.led.ledIndex(0x40, 255, 0, 0)      
        self.led.ledIndex(0x20, 255, 0, 0)      
        time.sleep(0.1)
        self.led.colorWipe(self.led.strip, Color(0,0,0)) 
        

        