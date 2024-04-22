import sys
sys.path.insert(0, '/home/pi/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')
import datetime
from cv2 import IMREAD_GRAYSCALE, imread, threshold, THRESH_BINARY_INV, findContours, RETR_EXTERNAL,CHAIN_APPROX_SIMPLE, contourArea 
from picamera2 import Picamera2
from ServoModule import *

class CameraModule:
    def __init__(self):
        self.servo=ServoModule()
    
    def capture_point(self):
        # get the date and time
        now=datetime.datetime.now()
        d=now.strftime("%Y-%m-%d")
        t=now.strftime("%H-%M-%S")
        
        # move camera down vertically
        self.servo.move_servo_1(80)
        
        # set image path
        image_path = f"/home/pi/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Data/{d}/{t}.jpg"
        
        # capture image
        camera = Picamera2()
        camera.start_and_capture_file(image_path)
        
        # move camera to normal vertical position 
        self.servo.move_servo_1(110)
        
        
        return image_path
    
    def process_image(self):
        # get image path
        image_path = self.capture_point()
        
        # load image in grayscale
        img = imread(image_path, cv2.IMREAD_GRAYSCALE)
        _ , binary_image = threshold(img, 200, 255, THRESH_BINARY_INV)
        
        # find contours in the binary image
        contours, _ = findContours(binary_image, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)
        
        # find number of paths
        paths = [cnt for cnt in contours if contourArea(cnt) > 100]
        
        num_paths = len(paths)
        
        log(f"paths found: {num_paths} - image path: {image_path}")
        return num_paths
    