'''
@author KyeonghyunYoo

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
{
	"SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
	"SettingsVersion": 1.2,
	"SimMode": "Multirotor",
	"ClockSpeed": 1,
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "X": 0,
      "Y": 0,
      "Z": 0
    },
    "Drone2": {
      "VehicleType": "SimpleFlight",
      "X": 0,
      "Y": 2,
      "Z": 0
    },
    "Drone3": {
      "VehicleType": "SimpleFlight",
      "X": 0,
      "Y": -2,
      "Z": 0
    },
    "Drone4": {
      "VehicleType": "SimpleFlight",
      "X": 1,
      "Y": 1,
      "Z": 0
    },
    "Drone5": {
      "VehicleType": "SimpleFlight",
      "X": -1,
      "Y": 1,
      "Z": 0
    },
    "Drone6": {
      "VehicleType": "SimpleFlight",
      "X": 1,
      "Y": -1,
      "Z": 0
    },
    "Drone7": {
      "VehicleType": "SimpleFlight",
      "X": -1,
      "Y": -1,
      "Z": 0
    }

  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
'''
from airsim.utils import *
from airsim.types import *
import numpy as np
import time
import tempfile
import os
import airsim
import threading
import json

class commands(airsim.MultirotorClient):
    def __init__(self, settingFile):
        with open(settingFile, 'r', encoding='utf-8') as f:
            self.json_data = json.load(f)
        super().__init__()
        self.client = airsim.MultirotorClient()
        self.degree = 0.0
        self.Head = 0
        self.direction = [0.0, 0.0, 0.0, 0.0]
        self.unit = []
        self.time_out = 20
        for i in self.json_data['Vehicles']:
            self.client.enableApiControl(True, i)
            self.client.armDisarm(True, i)
            self.unit.append(i)


    # The commands are the actions of drone
    def take_off(self):
        for i in self.unit:
            t = threading.Thread(target=self.client.takeoffAsync, args=(self.time_out, i))
            t.start()

    def land(self):
        for i in self.unit:
            t = threading.Thread(target=self.client.landAsync, args=(self.time_out, i))
            t.start()

    def up(self, height = 2, name = ''):
        for i in self.unit:
            state = self.get_drone_position(name)
            x = state.position.x_val
            y = state.position.y_val
            z = state.position.z_val - height
            t = threading.Thread(target=self.client.moveToPositionAsync, args=(x, y, z, 1.0,3e+38,
                                                                               DrivetrainType.MaxDegreeOfFreedom,
                                                                               YawMode(), -1, 1, i))
            t.start()

    def down(self, height = 2):
        for i in self.unit:
            state = self.get_drone_position(i)
            x = state.position.x_val
            y = state.position.y_val
            z = state.position.z_val + height
            t = threading.Thread(target=self.client.moveToPositionAsync,
                                 args=(x, y, z, 1.0, 3e+38,DrivetrainType.MaxDegreeOfFreedom,
                                       YawMode(), -1, 1, i))
            t.start()

    def forward(self, dis=3):
        for i in self.unit:
            self.direction[int(self.Head)] = dis
            state = self.get_drone_position(i)
            x = state.position.x_val + self.direction[0] - self.direction[2]
            y = state.position.y_val + self.direction[1] - self.direction[3]
            z = state.position.z_val
            t = threading.Thread(target=self.client.moveToPositionAsync,
                                 args=(x, y, z, 1.0,3e+38, DrivetrainType.MaxDegreeOfFreedom,
                                       YawMode(), -1, 1, i))
            t.start()
            self.reset_direction()

    def left(self, dis = 3):
        for i in self.unit:
            dir = self.Head + 3

            if dir >= 4:
                dir = dir - 4

            self.direction[int(dir)] = dis
            state = self.get_drone_position(i)
            x = state.position.x_val + self.direction[0] - self.direction[2]
            y = state.position.y_val + self.direction[1] - self.direction[3]
            z = state.position.z_val
            t = threading.Thread(target=self.client.moveToPositionAsync,
                                 args=(x, y, z, 1.0,3e+38, DrivetrainType.MaxDegreeOfFreedom,
                                       YawMode(), -1, 1, i))
            t.start()
            self.reset_direction()

    def right(self, dis = 3):
        for i in self.unit:
            dir = self.Head + 1

            if dir >= 4:
                dir = dir - 4

            self.direction[int(dir)] = dis
            state = self.get_drone_position(i)
            x = state.position.x_val + self.direction[0] - self.direction[2]
            y = state.position.y_val + self.direction[1] - self.direction[3]
            z = state.position.z_val
            t = threading.Thread(target=self.client.moveToPositionAsync,
                                 args=(x, y, z, 1.0,3e+38, DrivetrainType.MaxDegreeOfFreedom,
                                       YawMode(), -1, 1, i))
            t.start()
            self.reset_direction()

    def back(self, dis=3):
        for i in self.unit:
            dir = self.Head + 2

            if dir >= 4:
                dir = dir - 4

            self.direction[int(dir)] = dis
            state = self.get_drone_position(i)
            x = state.position.x_val + self.direction[0] - self.direction[2]
            y = state.position.y_val + self.direction[1] - self.direction[3]
            z = state.position.z_val
            t = threading.Thread(target=self.client.moveToPositionAsync,
                                 args=(x, y, z, 1.0,3e+38, DrivetrainType.MaxDegreeOfFreedom,
                                       YawMode(), -1, 1, i))
            t.start()
            self.reset_direction()

    def reset_direction(self):
        self.direction = [0.0, 0.0, 0.0, 0.0]

    def turn_right(self, name=''):
        vx = 0
        vy = 0
        duration = 1
        self.degree = self.degree + 90
        for i in self.unit:
            t = threading.Thread(target=self.client.moveByVelocityZAsync,
                                 args=(vx,vy,self.get_drone_position(name).position.z_val,
                                       duration, airsim.DrivetrainType.MaxDegreeOfFreedom,
                                       airsim.YawMode(False, self.degree - 15), i))
            t.start()
        self.Head = self.degree // 90

    def turn_left(self, name=''):
        vx = 0
        vy = 0
        duration = 1
        self.degree = self.degree - 90
        for i in self.unit:
            t = threading.Thread(target=self.client.moveByVelocityZAsync,
                                 args=(vx, vy, self.get_drone_position(name).position.z_val, duration,
                                       airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, self.degree + 15),
                                       i))
            t.start()
        self.Head = self.degree // 90

    def repos(self):
        a = self.get_drone_position('Drone1').position
        for i in self.unit:
            if i == 'Drone1':
                continue
            else:
                x = self.json_data['Vehicles'][i]["X"]
                y = self.json_data['Vehicles'][i]["Y"]
                z = self.json_data['Vehicles'][i]["Z"]
                self.client.moveToPositionAsync(a.x_val, a.y_val, a.z_val,
                                               1.0,3e+38, DrivetrainType.MaxDegreeOfFreedom,
                                                YawMode(), -1, 1, i)

    def get_drone_position(self, name = ""):
        return self.client.simGetVehiclePose(vehicle_name = name)

    def killed(self):
        for i in self.unit:
            self.client.enableApiControl(False, i)
            self.client.armDisarm(False, i)
