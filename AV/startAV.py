import sys
sys.path.insert(0, '/home/pi/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')
from log import log
from sensor_fusion import SensorInput
from battery import read_battery
import datetime
from Motor import *         
import time
import RPi.GPIO as GPIO


PWM=Motor() 
now=datetime.datetime.now()

def task1():
    log("task 1")
    moveAV = True
    si = SensorInput()
    PWM=Motor() 
    try:
        while moveAV:
            BV, BP = read_battery()
            action, direction = si.sensor_fusion()
            
            if action == 'move':
                moveAV = True
                if direction == 'forward':
                    PWM.setMotorModel(800,800,800,800)
                elif direction == 'right':
                    PWM.setMotorModel(800,800,-800,-800)
                elif direction == 'left':
                    PWM.setMotorModel(-800,-800,800,800)
                    
            elif action == 'decide':
                moveAV = False
                PWM.setMotorModel(0,0,0,0)
            
            log("sleeping")  
            time.sleep(2.2)
            PWM.setMotorModel(0,0,0,0)
            #break
            
    except KeyboardInterrupt: 
        read_battery()
        GPIO.cleanup()
        PWM.setMotorModel(0,0,0,0)   
        
def stop():
    GPIO.cleanup()
    PWM.setMotorModel(0,0,0,0)
    
# Main program logic follows:
if __name__ == '__main__':

    print ('Program is starting ... ')
    import sys
    if len(sys.argv)<2:
        print ("Parameter error: Please assign the device")
        exit() 
    if sys.argv[1] == 'task1':
        task1()
    elif sys.argv[1] == 'stop':
        stop()