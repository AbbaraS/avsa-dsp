import sys
sys.path.insert(0, '/home/pi/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')
from servo import *

class Servos:
    self.pwm_S=Servo()
    
    def move_servo_0(angle):
        self.pwm_S.setServoPwm('0', angle)
    
    def move_servo_1(angle):
        self.pwm_S.setServoPwm('1', angle)
    