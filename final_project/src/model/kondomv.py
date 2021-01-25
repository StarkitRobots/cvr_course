import sys
import json
from .model import Model
from ..odometry import Odometry
from .servo_dict import ServoDict

class KondoMV(Model):
    def __init__(self, imu=None):
        super().__init__()
        with open("model/kal.json") as f:
            self.servos = ServoDict(json.loads(f.read()))
        
        self.robot_height = 0.42

        self.sizes = {
        "a5": 0.0215,  # m distance from symmetry axis to servo 5
        "b5": 0.0185,  # м расстояние от оси сервы 5 до оси сервы 6 по горизонтали
        "c5": 0,     # м расстояние от оси сервы 6 до нуля Z по вертикали
        "a6": 0.042,    # м расстояние от оси сервы 6 до оси сервы 7
        "a7": 0.0655,  # м расстояние от оси сервы 7 до оси сервы 8
        "a8": 0.0638,  # м расстояние от оси сервы 8 до оси сервы 9
        "a9": 0.0355,  # м расстояние от оси сервы 9 до оси сервы 10
        "a10": 0.0254,  # м расстояние от оси сервы 10 до центра стопы по горизонтали
        "b10": 0.0164,  # м расстояние от оси сервы 10 до низа стопы
        "c10": 0.012   # м расстояние от оси сервы 6 до оси сервы 10 по горизонтали
        }

        self.odometry = Odometry(imu)

if __name__ == "__main__":
    kondo = KondoMV()
    print(kondo.servos['torso'])
