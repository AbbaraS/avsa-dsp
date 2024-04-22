from ADC import Adc
from log import log
import datetime

class Battery:
    def __init__(self):
        self.full_charge_voltage = 8.4
        self.min_voltage = 6.0
        self.adc = Adc()
        self.battery_voltage = self.get_battery_voltage()
    

    def get_battery_percentage():
        ''' 
        gets the remaining battery percentage
        '''

        percentage_left = ((self.battery_voltage - self.min_voltage) / (self.full_charge_voltage - self.min_voltage)) * 100

        percentage_left = max(0, min(100, percentage_left))

        #print(f'battery percentage: {percentage_left}%')

        log(f'battery percentage: {percentage_left}%')    

        return percentage_left


    def get_battery_voltage():
        ''' 
        gets the available battery voltage
        '''
        
        battery_voltage = self.adc.recvADC(2) * 3

        #log(f'battery voltage: {get_battery_voltage}')    

        return battery_voltage


    
    