



class Vehicle:
    '''
    Class Name: Vehicle
    '''
    def __init__(self):
        # AV State
        self.state = AVState()
        self.options=[]
        self.spd = 700  # Speed of the robot, used in the various movement functions

    
    def choose_direction(self):
        '''
        Choose the next direction for the robot to move based on the value of LMR
        (Left, Middle, Right from the IR sensors)
        '''
        self.state.avoiding_obstacle = False
        LMR = self.read_IR_sensors()
        
        if LMR != 7:
            self.state.making_decision = False
            self.options = []
        
        if LMR==2: #2 is 010 - middle sensor detected the line
            return self.forward
        elif LMR==4: #4 is 100 - left sensor detected the line, so turn left
            return self.left
        elif LMR==6: #6 is 110 - left and middle sensors detected the line, so take a sharp left turn
            return self.sharp_left
        elif LMR==1: #1 is 001 - right sensor detected the line, so turn right
            return self.right
        elif LMR==3: #3 is 011 - right and middle sensors detected the line, so take a sharp right turn
            return self.sharp_right
        elif LMR==7 and not self.state.making_decision: #7 is 111 - all sensors detected the line, so desicion point
            self.stop()
            time.sleep(0.5)
            img_path = self.capture_point()
            paths_ahead = self.process_image(img_path)
            self.state.making_decision = True
            return self.make_junction_decision()
            
        elif LMR == 0: # The robot's not detecting the line
            self.state.line_lost_count += 1
            if self.state.line_lost_count >= 5: # The robot has definitely gone off the line
                self.state.line_lost_count = 0
                log("line not detected. rotating to find line. ")
                return self.rotate_full
            return None
        else:
            return None
        
        
    def get_best_direction(self):
        self.pwm_S.setServoPwm('0',105)
        time.sleep(0.01)
        M = self.get_distance()
        log(f"gbd_M = {M}")
        if M > self.MIN_DISTANCE:
            self.avoiding_obstacle = False
            return None
        else:
            self.avoiding_obstacle = True
            log("obstacle ahead...")
            self.stop()
            time.sleep(1)
            L, M, R = self.check_us_LMR()    
            time.sleep(0.01)
            if M > self.MIN_DISTANCE:
                self.avoiding_obstacle = False
                return None
            elif L > R: 
                return self.rotate_left
            elif R > L:
                return self.rotate_right
            else: 
                return None
    
    def make_T_junction_decision(self):
        log("making t junction decision")
        self.stop()
        L, M, R = self.check_us_LMR()    
        time.sleep(0.01)
        if L > R: 
            return self.rotate_left
        elif R > L:
            return self.rotate_right
        else: 
            log("none")
            return None
        
        
    def make_junction_decision(self):
        #number of options = 3
        log("making junction decision")
        self.stop()
        L, M, R = self.check_us_LMR()
        
        #eliminate worst option
        
        worst = ""
        if L < M:
            worst = "L"
            self.options = ["M", "R"]
            if L > R:
                worst = "R"
                self.options = ["L", "M"]
        else:
            worst = "M"
            self.options = ["L", "R"]
            if M > R:
                worst = "R"
                self.options = ["L", "M"]

        log(f"eliminating worst option : {worst}")
        #number of options = 2, choose best of two options
        #number of options = 1

        if worst == "M":
            if L > R: 
                self.options=["R"]
                return self.rotate_left
            elif R > L:
                self.options=["L"]
                return self.rotate_right
        elif worst == "L":
            if M > R:
                self.options = ["R"]
                self.state.last_turn_dir = -1
                return self.choose_direction()
            elif R > M:
                self.options = ["M"]
                return self.rotate_right
        elif worst == "R":
            if L > M:
                self.options = ["M"]
                return self.rotate_left
            elif M > L:
                self.options = ["L"]
                self.state.last_turn_dir = 1
                self.forward()
                return self.choose_direction()
            
    def take_remaining_path(self):
        log("making remaining decision")
        self.state.making_decision=False
        if self.options[0] == "M":
            self.options = []
            return self.forward
        elif self.options[0] == "R":
            self.options = []
            return self.rotate_right
        elif self.options[0] == "L":
            self.options = []
            return self.rotate_left
    
    def run(self):
        self.pwm_S=Servo()
        self.pwm_S.setServoPwm('0', 105)
        self.pwm_S.setServoPwm('1', 110)

        self.battery_percentage = get_battery_percentage()
        
        self.spd = int(self.spd + (self.spd * (1 - (self.battery_percentage / 100))))
        
        log(f"speed: {self.spd}")

        while True:
            move = self.get_best_direction()
            if move is not None and not self.state.making_decision:
                move()
            else:
                if self.state.making_decision == True and len(self.options) == 1:
                    move = self.make_remaining_option()
                else:
                    move = self.choose_direction()
                    if len(self.options) == 1:
                        self.state.making_decision = True
                    elif len(self.options) == 0:
                        self.state.making_decision = False
                
                if move is not None:
                    self.state.line_lost_count = 0
                    move()

            if self.state.making_decision:
                log("sleeping .005")         
                time.sleep(0.005)
                self.stop()
            else:
                time.sleep(0.01)
        
        GPIO.cleanup()
        
        