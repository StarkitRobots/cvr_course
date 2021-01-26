import sys
import time
import math
import json
import os

from src.vision import Vision, Sensor
from src.localization import Localization
from src.model import Model


class Server:
    def __init__(self):
        with open("calibration/camera_calibration.json") as f:
            calibration = json.load(f)
        self.camera = Sensor()
        self.vision = Vision()
        self.model = Model()
        robot_height = 0.42
        # setting model parameters
        self.model.set_parameters(calibration["cam_col"],
                                  robot_height,
                                  [0, 0, 0, 0, 0, 0],
                                  [0, 0])
        self.localization = Localization()
        with open("logs/data.json") as f:
            self.logs_data = json.load(f)

    def run(self):
        for sample in self.logs_data:
            self_data = {}
            self.model.update_camera_pan_tilt(
                sample['head_yaw'],
                sample['head_pitch'])

            # Taking picture.
            img = self.camera.get_frame(sample['img'])
            # Processing image
            camera_data = self.vision.get(img)

            for observation_type in camera_data:
                # self means in robots coords
                if observation_type not in self_data.keys():
                    self_data[observation_type] = []
                self_points = []

                for observation in camera_data[observation_type]:
                    cx = observation[5]
                    cy = observation[6]
                    w = observation[2]
                    h = observation[3]

                    self_points.append(self.model.pic2r(cx, cy + (w + h) / 4))
                self_data[observation_type] += self_points

            self.localization.update(self_data)
            print(f'ETO ball: {self.localization.ball_position_global}, \
                    ETO robot: {self.localization.robot_position}')
