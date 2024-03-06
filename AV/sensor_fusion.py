import RPi.GPIO as GPIO
import time
from log import log

class SensorInput:
    def __init__(self):
        # Infrared Sensors
        self.IR01 = 14  # Left
        self.IR02 = 15  # Middle
        self.IR03 = 23  # Right
        
        # Ultrasonic Sensor
        self.trigger_pin = 27
        self.echo_pin = 22
        self.MAX_DISTANCE = 300  # cm
        self.timeOut = self.MAX_DISTANCE*60
        
        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IR01, self.IR02, self.IR03], GPIO.IN)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        
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
            distance_cm[i] = pingTime * 340.0 / 2.0 / 10000.0     # calculate distance with sound speed 340m/s
        distance_cm=sorted(distance_cm)
        return  int(distance_cm[2])
    
    def read_IR_sensors(self):  # For Infrared
        LMR = 0x00
        if GPIO.input(self.IR01): LMR |= 4  # Left
        if GPIO.input(self.IR02): LMR |= 2  # Middle
        if GPIO.input(self.IR03): LMR |= 1  # Right
        return LMR

    def sensor_fusion(self):
            #GPIO.cleanup()
        #while True:
            distance = self.get_distance()
            LMR = self.read_IR_sensors()
            action = ''
            direction = ''
            # Basic Sensor Fusion Algorithm
            if distance < 30:  # If obstacle is closer than 30cm, stop
                print("Obstacle detected, stopping.")

                action = 'decide'
                direction = 'Stop'
                log(f'sensor fusion, ultrasound, {distance}, {action}, {direction}')
            elif LMR in [2, 4, 1]:  # Line detected
                print("Following line")

                action = 'move'
                if LMR == 2:
                    direction = 'forward'
                elif LMR == 4:
                    direction = 'left'
                elif LMR == 1:
                    direction = 'right'
                    
            elif LMR in [6, 3]: # 2 sensors detected line - decision needs to happen
                print("two paths available")
                action = 'decide'
                direction = 'Stop'
            elif LMR == 7:  # 3 sensors detected the line - decision needs to happen
                print("three paths available")
                action = 'decide'
                direction = 'Stop'
            elif LMR == 0:
                print('lost line')
                action = 'decide'
                direction = 'Stop'
            else:
                print("Default action")
                # Define a default action
                action = 'decide'
                direction = 'Stop'
            
            log(f'sensor fusion, ir, {LMR}, {action}, {direction}') 
            #time.sleep(0.1)  # Small delay to prevent overloading
            
            return action, direction
