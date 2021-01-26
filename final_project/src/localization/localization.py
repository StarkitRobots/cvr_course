

class Localization:
    def __init__(self):
        self.robot_position = None
        self.ball_pos_self = None
        self.ball_pos_world = None

    
    def update(self, data):
        pass 

    def get_robot_position(self):
        return self.robot_position

    def get_ball_position(self):
        return self.ball_pos_world
