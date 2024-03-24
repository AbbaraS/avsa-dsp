import sys
sys.path.insert(0, '/home/pi/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')
import traceback
import RPi.GPIO as GPIO
import time
from log import log
from servo import *
from Motor import *  
from battery import *
import math
import datetime
from Led import *

SPD = 800  # Speed of the robot, used in the various movement functions


class AVState:
    '''
    Class Name: AVState
    Description: Contains variables relating to the state of the AV
    '''
    def __init__(self):
        self.avoiding_obstacle = False
        self.making_decision = False
        self.line_lost_count = 0
        self.last_turn_dir = 1  # 1 = left, -1 = right
    

class RunAV:
    '''
    Class Name: RunAV
    Description: Contains the AV navigation logic
    '''
    def __init__(self):
        # AV State
        self.state = AVState()

        # Infrared Sensors
        self.IR01 = 14  # Left
        self.IR02 = 15  # Middle
        self.IR03 = 23  # Right
        
        # Ultrasonic Sensor
        self.trigger_pin = 27
        self.echo_pin = 22
        self.MAX_DISTANCE = 300  # cm
        self.MIN_DISTANCE = 20 # cm
        self.timeOut = self.MAX_DISTANCE*60
        
        self.led=Led()
        
        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IR01, self.IR02, self.IR03], GPIO.IN)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        
    ### CONTROL ###
    
    ### Indication ###
    
    def indicate_left(self):
        self.led.colorWipe(self.led.strip, Color(0, 0, 0))
        self.led.ledIndex(0x01,255,0,0)      #Red
        #led.ledIndex(0x02,255,0,0)      #Red
        #led.ledIndex(0x04,255,0,0)      #Red
        #led.ledIndex(0x08,255,0,0)      #Red
        time.sleep(0.1)
        self.led.colorWipe(self.led.strip, Color(0,0,0))
        
    def indicate_right(self):
        self.led.colorWipe(self.led.strip, Color(0, 0, 0))
        #self.led.ledIndex(0x10,255,0,0)    #Red
        self.led.ledIndex(0x20,255,0,0)    #Red
        #led.ledIndex(0x40,255,0,0)    #Red
        #self.led.ledIndex(0x80,255,0,0)    #Red
        time.sleep(0.1)
        self.led.colorWipe(self.led.strip, Color(0,0,0))
    
    ### Movement ###
    
    def forward(self):
        '''
        Moves the robot forward
        '''
        log("forward")
        PWM.setMotorModel(SPD,SPD,SPD,SPD)
        if self.state.making_decision:
            self.state.making_decision = False
        
    def left(self):
        '''
        Moves the robot left
        '''
        #print("left")
        log("left")
        PWM.setMotorModel(-int(SPD*0.75),-int(SPD*0.75),int(SPD*1.25),int(SPD*1.25))
    
    def sharp_left(self):
        '''
        Moves the robot sharply left
        '''
        #print("sharp left")
        log("sharp left")
        PWM.setMotorModel(-SPD, -SPD, SPD*2, SPD*2)
    
    def right(self):
        '''
        Moves the robot right
        '''
        #print("right")
        log("right")
        PWM.setMotorModel(int(SPD*1.25),int(SPD*1.25), -int(SPD*0.75),-int(SPD*0.75))
    
    def sharp_right(self):
        '''
        Moves the robot sharply right
        '''
        #print("sharp right")
        log("sharp right")
        PWM.setMotorModel(SPD*2, SPD*2, -SPD, -SPD)
    
    def stop(self):
        '''
        Stops the robot
        '''
        #print("stop")
        log("stop")
        PWM.setMotorModel(0, 0, 0, 0)
    
    ### Rotation ###
    
    def rotate(self, target_angle, interrupt=False):
        '''
        Rotate the robot, up to a maximum of target_angle
        If target_angle > 0, rotate left, otherwise rotate right
        If interrupt = True, the rotation will stop early if the line is found
        '''
        log(f"start rotation")
        angle_rotated = 0
        bat_compensation = 7.5/get_battery_voltage()
        degrees_per_iteration = 5
        motor_time_proportion = 2 # Can change this
        time_to_rotate = 2 * degrees_per_iteration * motor_time_proportion * bat_compensation / 1000
        W=1700//2  # Angular velocity
        rotation_direction = 1 if target_angle  > 0 else -1 
        angle = rotation_direction * degrees_per_iteration
        VY = int(W * math.cos(math.radians(angle)))  # Y velocity
        VX = -int(W * math.sin(math.radians(angle)))  # X velocity
        
        FL = 0-(VY + VX) - W  # Front left wheel velocity
        BL = 0-(VY - VX) - W  # Back left wheel velocity
        
        FR = VY - VX + W  # Front right wheel velocity
        BR = VY + VX + W  # Back right wheel velocity
        
        if target_angle < 0:  # Invert if we're turning right
            FL = -FL
            BL = -BL
            FR = -FR
            BR = -BR
        
        while abs(angle_rotated) < abs(target_angle):
            PWM.setMotorModel(FL, BL, FR, BR)
            time.sleep(time_to_rotate)
            angle_rotated += rotation_direction * degrees_per_iteration
            if interrupt and abs(angle_rotated) > 25:
                LMR = self.read_IR_sensors()
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
    
    def rotate_left(self):
        log(f"rotate left")
        self.state.last_turn_dir = 1
        self.indicate_left()
        self.rotate(90, interrupt=True)
    
    def rotate_right(self):
        log(f"rotate right")
        self.state.last_turn_dir = -1
        self.indicate_right()
        self.rotate(-90, interrupt=True)
    
    def rotate_full(self):
        self.rotate(360 * self.state.last_turn_dir, interrupt=True)
    
    #### PERCEPTION ###
    
    def read_IR_sensors(self):  # For Infrared
        LMR = 0x00
        if GPIO.input(self.IR01): LMR |= 4  # Left
        if GPIO.input(self.IR02): LMR |= 2  # Middle
        if GPIO.input(self.IR03): LMR |= 1  # Right
        #log(f"LMR={LMR}")
        return LMR    
    
    def pulseIn(self, pin, level, timeOut):  # For Ultrasonic
        t0 = time.time()
        while(GPIO.input(pin) != level):
            if((time.time() - t0) > timeOut*0.000001):
                return 0
        t0 = time.time()
        while(GPIO.input(pin) == level):
            if((time.time() - t0) > timeOut*0.000001):
                return 0
        pulseTime = (time.time() - t0)*1000000
        return pulseTime

    def get_distance(self):  # For Ultrasonic
        distance_cm=[0,0,0,0,0]
        for i in range(5):
            GPIO.output(self.trigger_pin,GPIO.HIGH)      # make trigger_pin output 10us HIGH level 
            time.sleep(0.00001)     # 10us
            GPIO.output(self.trigger_pin,GPIO.LOW) # make trigger_pin output LOW level 
            pingTime = self.pulseIn(self.echo_pin,GPIO.HIGH,self.timeOut)   # read plus time of echo_pin
            dis = pingTime * 340.0 / 2.0 / 10000.0     # calculate distance with sound speed 340m/s
            if dis == 0.0:
                dis = 2000000
            distance_cm[i] = dis
        distance_cm=sorted(distance_cm)
        return  int(distance_cm[0])
        
    def check_us_LMR(self):
        for i in range(45,166,60):
            self.pwm_S.setServoPwm('0',i)
            time.sleep(0.5)
            if i==45:
                L = self.get_distance()
                log(f"L={L}")
            elif i==105:
                M = self.get_distance()
                log(f"M={M}")
            else:
                R = self.get_distance()
                log(f"R={R}")
        
        self.pwm_S.setServoPwm('0', 105)
        return L, M, R
    
    ### PLANNING ###
    
    def choose_direction(self, LMR):
        '''
        Choose the next direction for the robot to move based on the value of LMR
        (Left, Middle, Right from the IR sensors)
        '''
        self.state.avoiding_obstacle = False
        
        if LMR==2: #2 is 010 - middle sensor detected the line
            return self.forward
        elif LMR==4: #4 is 100 - left sensor detected the line, so turn left
            return self.left
        elif LMR==6: #6 is 110 - left and middle sensors detected the line, so take a sharp left turn
            return self.sharp_left
        elif LMR==1: #1 is 001 - right sensor detected the line, so turn right
            return self.right
        elif LMR==3: #3 is 011 - right and middle sensors detected the line, so take a sharp right turn
            return self.sharp_right
        elif LMR==7 and not self.state.making_decision: #7 is 111 - all sensors detected the line, so desicion point
            self.state.making_decision = True
            return self.make_T_junction_decision()
            
        elif LMR == 0: # The robot's not detecting the line
            self.state.line_lost_count += 1
            if self.state.line_lost_count >= 5: # The robot has definitely gone off the line
                self.state.line_lost_count = 0
                log("line not detected. rotating to find line. ")
                return self.rotate_full
            return None
        else:
            return None
    
    def get_best_direction(self):
        M = self.get_distance()
        if M > self.MIN_DISTANCE:
            self.avoiding_obstacle = False
            return None
        else:
            self.avoiding_obstacle = True
            log("obstacle ahead...")
            self.stop()
            L, M, R = self.check_us_LMR()    
            time.sleep(0.05)
            if M > self.MIN_DISTANCE:
                self.avoiding_obstacle = False
                return None
            elif L > R: 
                return self.rotate_left
            elif R > L:
                return self.rotate_right
            else: 
                return None
    
    def make_T_junction_decision(self):
        log("making t junction decision")
        self.stop()
        L, M, R = self.check_us_LMR()    
        time.sleep(0.01)
        if L > R: 
            return self.rotate_left
        elif R > L:
            return self.rotate_right
        else: 
            log("none")
            return None
    
    ### run ###
    
    def run(self):
        self.pwm_S=Servo()
        self.pwm_S.setServoPwm('0', 105)
        self.pwm_S.setServoPwm('1', 110)

        self.battery_percentage = get_battery_percentage()

        while True:
            
            move = self.get_best_direction()
            if move is not None:
                move()
            else:
                LMR = self.read_IR_sensors()
                move = self.choose_direction(LMR)
                if move is not None:
                    self.state.line_lost_count = 0
                    move()

                    
            time.sleep(0.01)
        
        GPIO.cleanup()
           

runAV=RunAV()

# Main program logic follows:
if __name__ == '__main__':
    #print ('Program is starting ... ')
    log('\n_______________________________________________')
    log('Program is starting ... ')
    try:
        runAV.run()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program  will be  executed.
        log("KeyboardInterrupt")
        log(f"avoiding obstacle: {runAV.avoiding_obstacle}, making decision: {runAV.making_decision} , line lost count: {runAV.line_lost_count}")
        GPIO.cleanup()
        runAV.pwm_S.setServoPwm('0', 105)
        runAV.pwm_S.setServoPwm('1', 110)
        PWM.setMotorModel(0,0,0,0)
    except Exception as e:
        print(traceback.format_exc())
        GPIO.cleanup()
        log(traceback.format_exc())
        PWM.setMotorModel(0,0,0,0)
        runAV.pwm_S.setServoPwm('0', 105)
        runAV.pwm_S.setServoPwm('1', 110)
        exit()
        
        