from Ultrasonic import *
ultrasonic=Ultrasonic()  

def ultrasonic():
    distance_cm = ultrasonic.get_distance()
    try:
        
        while True:
            distance_cm = ultrasonic.get_distance()   #Get the value
            print ("Obstacle distance is "+str(data)+"CM")
            time.sleep(1)
    except KeyboardInterrupt:
        print ("\nEnd of program")
