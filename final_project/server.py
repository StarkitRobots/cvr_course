import sys
import time
import math
import json
import os

from src.vision import Vision, Sensor
from src.localization import Localization
from src.model import Model


class Server:
    def __init__(self, sequence_number):
        with open("calibration/calib.json") as f:
            calibration = json.load(f)
        self.camera = Sensor()
        self.vision = Vision()
        self.model = Model()
        robot_height = 0.67
        # setting model parameters
        self.model.set_parameters(calibration["camera_matrix"],
                                  robot_height,
                                  calibration["k_coef"],
                                  calibration["p_coef"])
        self.localization = Localization()
        with open(f"cvr_logs/seq_{sequence_number}.json") as f:
            self.logs_data = json.load(f)

    def run(self):
        for sample in self.logs_data:
            self_data = {}
            self.model.update_camera_pan_tilt(
                float(sample['head_yaw']),
                float(sample['head_pitch']))

            # Taking picture.
            img = self.camera.get_frame(os.path.join("cvr_logs", sample['img_path']))
            # Processing image
            camera_data = self.vision.get(img)
            for observation_type in camera_data:
                # self means in robots coords
                if observation_type not in self_data.keys():
                    self_data[observation_type] = []
                self_points = []
                for observation in camera_data[observation_type]:
                    cx, cy, w, h = observation
                    self_points.append(self.model.pic2r(cx, cy + (w + h) / 4))
                self_data[observation_type] += self_points

            self.localization.update(self_data)
            print(f'ETO ball: {self.localization.get_ball_position()}, \
                    ETO robot: {self.localization.get_robot_position()}')
