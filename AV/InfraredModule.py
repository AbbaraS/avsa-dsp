import RPi.GPIO as GPIO

class InfraredModule:
    '''
    Class Name: InfraredModule
    Description: It sets the GPIO pins for the IR sensors and returns their values
    '''
    def __init__(self):
        self.IR01 = 14  # Left
        self.IR02 = 15  # Middle
        self.IR03 = 23  # Right
    
        # GPIO setup 
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IR01, self.IR02, self.IR03], GPIO.IN)
    
    def read_IR(self): 
        LMR = 0x00 # binary value for each IR sensor
        if GPIO.input(self.IR01): LMR |= 4  # Left
        if GPIO.input(self.IR02): LMR |= 2  # Middle
        if GPIO.input(self.IR03): LMR |= 1  # Right
        return LMR   