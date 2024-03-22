from ADC import Adc
from log import log
import datetime

def get_battery_percentage():
    
    adc = Adc()
    
    full_charge_voltage = 8.4
    
    min_voltage = 6.0
    
    battery_voltage = adc.recvADC(2) * 3 
    
    percentage_left = ((battery_voltage - min_voltage) / (full_charge_voltage - min_voltage)) * 100
    
    percentage_left = max(0, min(100, percentage_left))
    
    #print(f'battery percentage: {percentage_left}%')
    
    log(f'battery percentage: {percentage_left}%')    
    
    return percentage_left


def get_battery_voltage():
    
    adc = Adc()
    
    battery_voltage = adc.recvADC(2) * 3 
    
    #log(f'battery voltage: {battery_voltage}')    
    
    return battery_voltage


    
    