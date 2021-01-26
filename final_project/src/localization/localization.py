

class Localization:
    def __init__(self):
        self.robot_position = None
        self.ball_position_local = None
        self.ball_position_global = None

    
    def update(self, data):
        pass 

    def get_robot_position(self):
        return self.robot_position

    def get_ball_position(self):
        return self.ball_position_global
