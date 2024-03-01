import sys
sys.path.insert(0, '/home/pi/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')
from log import log
from Ultrasonic import *
from battery import read_battery
import datetime

now=datetime.datetime.now()

def task1():
    log("task 1")
    moveAV = True
    us=Ultrasonic() 
    now=datetime.datetime.now()
    t2=now.strftime("%H-%M-%s")
    start_s = int(now.strftime("%s"))
    try:
        while moveAV:
            #read battery every 10 seconds
            now=datetime.datetime.now()
            s = int(now.strftime("%s"))
            interval = start_s
            if s == interval:
                BV, BP = read_battery()
                interval = interval + 10 #read battery every 10 s
            
            distance_cm = ultrasonic.get_distance()
            print(distance_cm)
            if distance_cm > 30:
                PWM.setMotorModel(1000,1000,1000,1000)
            else:
                PWM.setMotorModel(0,0,0,0)
                moveAV = False
        read_battery()
    except KeyboardInterrupt: 
        read_battery()
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