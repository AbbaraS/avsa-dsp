import datetime
import os


now=datetime.datetime.now()
d=now.strftime("%Y-%m-%d")
t=now.strftime("%H-%M")
t2=now.strftime("%H-%M-%s")
path=f"/home/pi/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Data/{d}/{t}"

if not os.path.exists(path):
    os.makedirs(path)

#log_file = f"{path}\{t}.txt"

# u - ultrasoic
# p - photoresistor
# i - infrared


def log(log_txt, m=None):
    write = True
    if m == 'u':
        file = 'ultrasonic'
    elif m == 'p':
        file = 'photoresistor'
    elif m == 'i':
        file = 'infrared'
    elif m == 'b':
        file = 'battery'
    else:
        file = 'general'
        with open(f'{path}/{file}', "a") as file:
            file.write(f'{log_txt}')
        write = False
    
    if write:
        with open(f'{path}/{file}', "a") as file:
            file.write(f'{t2}, {log_txt}')