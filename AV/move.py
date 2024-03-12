import sys
sys.path.insert(0, '/home/pi/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')
import traceback
import RPi.GPIO as GPIO
import time
from log import log
from servo import *
from Motor import *  


class RunAV:
    def __init__(self):
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
        return LMR
    
    ### movement ###
    
    def forward(self):
        print("forward")
        PWM.setMotorModel(800,800,800,800)
        
    def left(self):
        print("left")
        PWM.setMotorModel(-750,-750,1250,1250)
    
    def sharp_left(self):
        print("sharp left")
        PWM.setMotorModel(-1000, -1000, 2000, 2000)
    
    def right(self):
        print("right")
        PWM.setMotorModel(1250,1250, -750,-750)
    
    def sharp_right(self):
        print("sharp right")
        PWM.setMotorModel(2000, 2000, -1000, -1000)
    
    def stop(self):
        print("stop")
        PWM.setMotorModel(0, 0, 0, 0)
    
    def choose_direction(self, LMR):
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
        for i in range(30,151,60):
            #print(i)
            self.pwm_S.setServoPwm('0',i)
            time.sleep(0.2)
            if i==30:
                L = self.get_distance()
                print(f"L={L}")
            elif i==90:
                M = self.get_distance()
                print(f"M={M}")
            else:
                R = self.get_distance()
                print(f"R={R}")
        
        self.pwm_S.setServoPwm('0', 90)
        return L, M, R
    
    def get_us_best_direction(self):
        M = self.get_distance()
        
        if M > self.MIN_DISTANCE:
            return None
        else:
            self.stop()
            print(f"dir M={M}")
            L, M, R = self.check_us_LMR()    
            time.sleep(0.1)
            if M > self.MIN_DISTANCE:
                return None
            elif L > R: 
                return self.left
            elif R > L:
                return self.right
            else: 
                print("hello? :3")
                return None
    
    ### run ###
    def run(self):
        self.pwm_S=Servo()
        self.pwm_S.setServoPwm('0', 90)
        while True:
            
            move = self.get_us_best_direction()
            if move is not None:
                move()
            else:
                LMR = self.read_IR_sensors()
                self.choose_direction(LMR)
                
            time.sleep(0.1)
        

            
            
runAV=RunAV()
# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')
    try:
        runAV.run()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program  will be  executed.
        runAV.pwm_S.setServoPwm('0', 90)
        PWM.setMotorModel(0,0,0,0)
    except Exception as e:
        print(traceback.format_exc())
        PWM.setMotorModel(0,0,0,0)
        runAV.pwm_S.setServoPwm('0', 90)
        exit()