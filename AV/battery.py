from ADC import Adc
from log import log
import datetime

def read_battery():
    
    adc = Adc()
    
    #adc_value = adc.recvADC(2) #reading channel 2 for battery voltage
    
    full_charge_voltage = 8.4
    
    min_voltage = 6.0
    
    battery_voltage = adc.recvADC(2) * 3 
    
    percentage_left = ((battery_voltage - min_voltage) / (full_charge_voltage - min_voltage)) * 100
    
    percentage_left = max(0, min(100, percentage_left))
    
    log(f'battery, {battery_voltage}, {percentage_left}')    
    
    return battery_voltage, percentage_left


#def read_battery_every_s(s):
    
    