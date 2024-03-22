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

class RunAV:
    def __init__(self):
        
        # 
        self.avoiding_obstacle = False
        
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
        
        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IR01, self.IR02, self.IR03], GPIO.IN)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        
        
    
    ### infrared ###
    
    def read_IR_sensors(self):  # For Infrared
        LMR = 0x00
        if GPIO.input(self.IR01): LMR |= 4  # Left
        if GPIO.input(self.IR02): LMR |= 2  # Middle
        if GPIO.input(self.IR03): LMR |= 1  # Right
        #log(f"LMR={LMR}")
        return LMR
    
    ### movement ###
    
    def forward(self):
        #print("forward")
        log("forward")
        PWM.setMotorModel(800,800,800,800)
        
    def left(self):
        #print("left")
        log("left")
        PWM.setMotorModel(-750,-750,1250,1250)
    
    def sharp_left(self):
        #print("sharp left")
        log("sharp left")
        PWM.setMotorModel(-1000, -1000, 2000, 2000)
    
    def right(self):
        #print("right")
        log("right")
        PWM.setMotorModel(1250,1250, -750,-750)
    
    def sharp_right(self):
        #print("sharp right")
        log("sharp right")
        PWM.setMotorModel(2000, 2000, -1000, -1000)
    
    def stop(self):
        #print("stop")
        log("stop")
        PWM.setMotorModel(0, 0, 0, 0)
    
    def choose_direction(self, LMR):
        #log("choosing direction ")
        self.avoiding_obstacle = False
        
        if LMR==2: #2 is 010 - middle sensor detected the line
            self.forward()
        elif LMR==4: #4 is 100 - left sensor detected the line, so turn left
            self.left()
        elif LMR==6: #6 is 110 - left and middle sensors detected the line, so take a sharp left turn
            self.sharp_left()
        elif LMR==1: #1 is 001 - right sensor detected the line, so turn right
            self.right()
        elif LMR==3: #3 is 011 - right and middle sensors detected the line, so take a sharp right turn
            self.sharp_right()
        elif LMR==7: #7 is 111 - all sensors detected the line, so stop
            self.stop()
        #else:
            
    def rotate(self, target_angle, interrupt=False):
        log(f"start rotation")
        
        angle_rotated = 0
        
        bat_compensation = 7.5/get_battery_voltage()
        #log(f"bat_compensation={bat_compensation}")
        
        degrees_per_iteration = 5
        
        motor_time_proportion = 2
        
        time_to_rotate = 2 * degrees_per_iteration * motor_time_proportion * bat_compensation / 1000
        #log(f"time_to_rotate={time_to_rotate}")        
        
        W=1700//2
        #log(f"W={W}")
        
        rotation_direction = 1 if target_angle  > 0 else -1 
        #log(f"rotation_direction={rotation_direction}")
        
        angle = rotation_direction * degrees_per_iteration
        #log(f"angle={angle}")
        
        VY = int(W * math.cos(math.radians(angle)))
        VX = -int(W * math.sin(math.radians(angle)))
        #log(f"VY={VY}, VX={VX}")
        
        FL = 0-(VY + VX) - W
        BL = 0-(VY - VX) - W
        
        FR = VY - VX + W
        BR = VY + VX + W
        
        if target_angle < 0:
            FL = -FL
            BL = -BL
            FR = -FR
            BR = -BR
        
        #log(f"FL={FL}, BL={BL}, FR={FR}, BR={BR}")        

        #start_time = datetime.datetime.now()
        #loop_n=0
        
        while abs(angle_rotated) < abs(target_angle):
            #loop_n +=1

            PWM.setMotorModel(FL, BL, FR, BR)

            time.sleep(time_to_rotate)
            
            angle_rotated += rotation_direction * degrees_per_iteration
            
            if interrupt and abs(angle_rotated) > 20:
                LMR = self.read_IR_sensors()
                if LMR != 0x00:
                    self.stop()
                    self.avoiding_obstacle = False
                    log("Line found. Stopping rotation. ")
                    break

        #stop_time = datetime.datetime.now()
        #avg_time = (stop_time-start_time)/loop_n
        #log(f'({stop_time} - {start_time}) / {loop_n} = {avg_time}')
        if angle_rotated == target_angle:
            self.stop()
            log(f"line not found.")
            
        log(f"angle rotated {angle_rotated}. ")
    
    def rotate_left(self):
        log(f"rotate left")
        self.rotate(90, interrupt=True)
    
    def rotate_right(self):
        log(f"rotate right")
        self.rotate(-90, interrupt=True)
    
    
    ### ultrasonic ###
    
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
        #print(f"distance_cm={distance_cm}")
        return  int(distance_cm[0])
        
    def check_us_LMR(self):
        for i in range(45,166,60):
            #print(i)
            self.pwm_S.setServoPwm('0',i)
            time.sleep(0.2)
            if i==45:
                L = self.get_distance()
                #print(f"L={L}")
                log(f"L={L}")
            elif i==105:
                M = self.get_distance()
                #print(f"M={M}")
                log(f"M={M}")
            else:
                R = self.get_distance()
                #print(f"R={R}")
                log(f"R={R}")
        
        self.pwm_S.setServoPwm('0', 105)
        return L, M, R
    
    def get_us_best_direction(self):
        M = self.get_distance()
        log(f"get_dis M={M}")
        if M > self.MIN_DISTANCE:
            self.avoiding_obstacle = False
            return None
        else:
            self.avoiding_obstacle = True
            log("obstacle ahead...")
            self.stop()
            #print(f"dir M={M}")
            #log(f"dir M={M}")
            L, M, R = self.check_us_LMR()    
            time.sleep(0.05)
            if M > self.MIN_DISTANCE:
                return None
            elif L > R: 
                return self.rotate_left
            elif R > L:
                return self.rotate_right
            else: 
                return None
    
    ### run ###
    
    def run(self):
        self.pwm_S=Servo()
        self.pwm_S.setServoPwm('0', 105)
        self.pwm_S.setServoPwm('1', 110)

        self.battery_percentage = get_battery_percentage()

        while True:
            
            move = self.get_us_best_direction()
            if move is not None:
                move()
                #log("moving..")
            #elif not self.avoiding_obstacle:
            else:
                LMR = self.read_IR_sensors()
                self.choose_direction(LMR)
            #else:
            #    log("else in run")
            #    self.forward()
            time.sleep(0.05)
        
        GPIO.cleanup()
        

            
            
runAV=RunAV()
# Main program logic follows:
if __name__ == '__main__':
    #print ('Program is starting ... ')
    log('_______________________________________________')
    log('Program is starting ... ')
    try:
        runAV.run()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program  will be  executed.
        log("KeyboardInterrupt")
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