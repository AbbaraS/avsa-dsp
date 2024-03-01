import sys
sys.path.insert(0, '/home/pi/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')


from Line_Tracking import Line_Tracking

def infrared_reading():
    GPIO.input(self.IR01)