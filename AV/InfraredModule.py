


class InfraredModule:
    self.IR01 = 14  # Left
    self.IR02 = 15  # Middle
    self.IR03 = 23  # Right
    
    # GPIO setup 
    GPIO.setup([self.IR01, self.IR02, self.IR03], GPIO.IN)
    
    def read_IR_sensors(self):  # For Infrared
        LMR = 0x00
        if GPIO.input(self.IR01): LMR |= 4  # Left
        if GPIO.input(self.IR02): LMR |= 2  # Middle
        if GPIO.input(self.IR03): LMR |= 1  # Right
        #log(f"LMR={LMR}")
        return LMR   