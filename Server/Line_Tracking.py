import time
from Motor import *
import RPi.GPIO as GPIO
class Line_Tracking:
    def __init__(self):
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IR01,GPIO.IN)
        GPIO.setup(self.IR02,GPIO.IN)
        GPIO.setup(self.IR03,GPIO.IN)
    
    def run(self):
        while True:
            self.LMR=0x00 #binary value 000
            
            if GPIO.input(self.IR01)==True:
                self.LMR=(self.LMR | 4)
                #print(f"IR01 - {self.LMR} - left sensor")
            
            if GPIO.input(self.IR02)==True:
                self.LMR=(self.LMR | 2)
                #print(f"IR02 - {self.LMR} - middle sensor")
            
            if GPIO.input(self.IR03)==True:
                self.LMR=(self.LMR | 1)
                #print(f"IR03 - {self.LMR} - right sensor")
                
                
            if self.LMR==2: #2 is 010 - middle sensor detected the line
                print("forward - 2")
                PWM.setMotorModel(800,800,800,800)
            elif self.LMR==4: #4 is 100 - left sensor detected the line, so turn left
                print("left - 4")
                PWM.setMotorModel(-1500,-1500,2500,2500)
            elif self.LMR==6: #6 is 110 - left and middle sensors detected the line, so take a sharp left turn
                print("left - 6")
                PWM.setMotorModel(-2000,-2000,4000,4000)
            elif self.LMR==1: #1 is 001 - right sensor detected the line, so turn right
                print("right - 1")
                PWM.setMotorModel(2500,2500,-1500,-1500) 
            elif self.LMR==3: #3 is 011 - right and middle sensors detected the line, so take a sharp right turn
                print("right - 3")
                PWM.setMotorModel(4000,4000,-2000,-2000)
            elif self.LMR==7: #7 is 111 - all sensors detected the line, so stop
                #pass
                print("stop - 7")
                PWM.setMotorModel(0,0,0,0)
            #print("end while")
            
infrared=Line_Tracking()
# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')
    try:
        infrared.run()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program  will be  executed.
        PWM.setMotorModel(0,0,0,0)
