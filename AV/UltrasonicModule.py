import RPi.GPIO as GPIO
import time


class UltrasonicModule:
    self.trigger_pin = 27 #setting the trigger pin GPIO number
    self.echo_pin = 22 #setting the echo pin GPIO number
    self.MAX_DISTANCE = 300  # cm
    self.MIN_DISTANCE = 20 # cm
    self.timeOut = self.MAX_DISTANCE*60
    
    # GPIO setup
    GPIO.setup(self.trigger_pin, GPIO.OUT)
    GPIO.setup(self.echo_pin, GPIO.IN)
    
    
    def pulseIn(self, pin, level, timeOut): 
        ''' 
        sends the ultrasonic wave and returns the pulse time 
        '''
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
    
    
    def get_distance(self):  
        '''
        triggers the ultrasonic wave and reads its pulse time to calculate the distance from the retuned echo 
        '''
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