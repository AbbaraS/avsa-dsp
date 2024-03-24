import datetime
import os


now=datetime.datetime.now()
d=now.strftime("%Y-%m-%d")
t=now.strftime("%H-%M-%S")
#t2=now.strftime("%H-%M-%s")
path=f"/home/pi/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Data/{d}"

if not os.path.exists(path):
    os.makedirs(path)

log_file = f"{path}/{t}.txt"

# u - ultrasoic
# p - photoresistor
# i - infrared


def log(log_txt):
    now=datetime.datetime.now()
    now_time=now.strftime("%H:%M:%S")
    with open(f'{log_file}', "a") as file:
        file.write(f'{now_time}, {log_txt}\n')

    
