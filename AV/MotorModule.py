import sys
sys.path.insert(0, '/home/pi/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')
from Motor import *  
from log import log
from Battery import *
from InfraredModule import *

class MotorModule:
    def __init__(self):
        self.motor = Motor()
        self.battery = Battery()
        self.motor_time_proportion = 2
        self.battery_compensation = 7.5/self.battery.battery_voltage
        self.W = 1700//2  # Angular velocity
        self.IR = InfraredModule()
        
        
    def set_motor_model(self, LU, LL, RU, RL):
        self.motor.setMotorModel(LU, LL, RU, RL)
    
    def rotate(self, target_angle, interrupt=False):
        '''
        Rotate the robot, up to a maximum of target_angle
        If target_angle > 0, rotate left, otherwise rotate right
        If interrupt = True, the rotation will stop early if the line is found
        '''
        log(f"start rotation")
        angle_rotated = 0
        degrees_per_iteration = 5
        time_to_rotate = 2 * degrees_per_iteration * self.motor_time_proportion * self.battery_compensation / 1000
        rotation_direction = 1 if target_angle  > 0 else -1 
        angle = rotation_direction * degrees_per_iteration
        
        VY = int(self.W * math.cos(math.radians(angle)))  # Y velocity
        VX = -int(self.W * math.sin(math.radians(angle)))  # X velocity
        
        FL = 0-(VY + VX) - self.W  # Front left wheel velocity
        BL = 0-(VY - VX) - self.W  # Back left wheel velocity
        
        FR = VY - VX + self.W  # Front right wheel velocity
        BR = VY + VX + self.W  # Back right wheel velocity
        
        if target_angle < 0:  # Invert if we're turning right
            FL = -FL
            BL = -BL
            FR = -FR
            BR = -BR
        
        while abs(angle_rotated) < abs(target_angle):
            self.set_motor_model(FL, BL, FR, BR)
            time.sleep(time_to_rotate)
            angle_rotated += rotation_direction * degrees_per_iteration
            if interrupt and abs(angle_rotated) > 25:
                LMR = self.IR.read_IR()
                if LMR != 0x00:
                    self.stop()
                    self.state.avoiding_obstacle = False
                    log("line found. stopping rotation. ")
                    # If the robot has rotated ~180 degrees then we assume it reached the end of the path, so quit the program
                    if 200 >= abs(angle_rotated) >= 150:
                        log(f"exiting at rotated angle {angle_rotated}")
                        GPIO.cleanup()
                        exit()
                    break

        if angle_rotated == target_angle:
            self.stop()
            log(f"line not found.")
        log(f"angle rotated {angle_rotated}. ")