class VehicleState:
    '''
    Class Name: VehicleState
    Description: Contains variables relating to the state of the AV
    '''
    def __init__(self):
        self.avoiding_obstacle = False
        self.making_decision = False
        self.line_lost_count = 0
        self.last_turn_dir = 1  # 1 = left, -1 = right
       