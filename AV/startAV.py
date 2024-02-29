import sys
sys.path.insert(0, '/home/pi/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')

from Ultrasonic import *
from ADC import Adc

def read_battery():
    adc = Adc()
    Left_IDR=adc.recvADC(0) 
    Right_IDR=adc.recvADC(1)
    Power=adc.recvADC(2)*3
    print(f"left={Left_IDR} right={Right_IDR} power={Power}")

def task1():
    print("task 1")
    
    read_battery()
    
    ultrasonic=Ultrasonic() 
    
    distance_cm = ultrasonic.get_distance()
    try:
        while True:
            distance_cm = ultrasonic.get_distance()
            print(distance_cm)
            if distance_cm > 30:
                PWM.setMotorModel(2000,2000,2000,2000)
            else:
                PWM.setMotorModel(0,0,0,0)
    except KeyboardInterrupt: 
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