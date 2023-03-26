"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


DOF control methods example
---------------------------
An example that demonstrates various DOF control methods:
- Load cartpole asset from an urdf
- Get/set DOF properties
- Set DOF position and velocity targets
- Get DOF positions
- Apply DOF efforts
"""

import math
from isaacgym import gymutil, gymtorch, gymapi
import time
import os
import torch
import yaml
# from pytorch3d.transforms import euler_angles_to_matrix, matrix_to_quaternion, quaternion_to_matrix, quaternion_apply, quaternion_invert
from tasks.glider_physics import LiftingLine
# from joystick import Joystick


class State_Check():
    def __init__(self):
        self.dt = 0.01
        self.device='cuda'
        self.num_envs=2
        with open("cfg/task/Dynasoar.yaml", "r") as cfg:
            try:
                self.cfg = yaml.safe_load(cfg)
            except yaml.YAMLError as exc:
                print(exc)

        self.glider_states = torch.zeros((self.num_envs, 13), dtype=torch.float, device=self.device)
        self.glider_states[:,6] = 1.0
        self.glider_states[:,7] = -20.0
        self.actions = torch.zeros((self.num_envs,6), dtype=torch.float, device=self.device)
        self.initial_glider_states = torch.zeros((self.num_envs, 13), dtype=torch.float, device=self.device)
        self.initial_glider_states[:,6] = 1.0
        self.debug_flags = self.cfg["env"]["debug_flags"]

        self.glider_params = {"wings": self.cfg["env"]["glider"]["wings"],
                              "station_pts": self.cfg["env"]["glider"]["station_pts"],
                              "cla": self.cfg["env"]["glider"]["cla"],
                              "rho": self.cfg["env"]["glider"]["rho"],
                              "eps": self.cfg["env"]["glider"]["eps"],
                              "envs" : (self.num_envs),
                              "TW1": torch.tensor(self.cfg["env"]["glider"]["offsets"]["W1"]),
                              "TW2": torch.tensor(self.cfg["env"]["glider"]["offsets"]["W2"]),
                              "TW3": torch.tensor(self.cfg["env"]["glider"]["offsets"]["W3"]),
                              "TW4": torch.tensor(self.cfg["env"]["glider"]["offsets"]["W4"]),
                              "HW1": torch.tensor(self.cfg["env"]["glider"]["headings"]["W1"]),
                              "HW2": torch.tensor(self.cfg["env"]["glider"]["headings"]["W2"]),
                              "HW3": torch.tensor(self.cfg["env"]["glider"]["headings"]["W3"]),
                              "HW4": torch.tensor(self.cfg["env"]["glider"]["headings"]["W4"]),
                              "C1":  torch.tensor(self.cfg["env"]["glider"]["chords"]["W1"]),
                              "C2":  torch.tensor(self.cfg["env"]["glider"]["chords"]["W2"]),
                              "C3":  torch.tensor(self.cfg["env"]["glider"]["chords"]["W3"]),
                              "C4":  torch.tensor(self.cfg["env"]["glider"]["chords"]["W4"]),
                              "wind_speed":  torch.tensor(self.cfg["env"]["glider"]["wind_speed"]),
                              "wind_thickness":  torch.tensor(self.cfg["env"]["glider"]["wind_thickness"]),
                              "thrust_force":  self.cfg["env"]["glider"]["thrust_force"],
                              "brake_force":  self.cfg["env"]["glider"]["brake_force"],
                              "mass":  self.cfg["env"]["glider"]["mass"],
                              "ixx":  self.cfg["env"]["glider"]["ixx"],
                              "iyy":  self.cfg["env"]["glider"]["iyy"],
                              "izz":  self.cfg["env"]["glider"]["izz"],
                              "ixz":  self.cfg["env"]["glider"]["ixz"],
                              "Cd0":  self.cfg["env"]["glider"]["Cd0"],
                              "front_area":  self.cfg["env"]["glider"]["front_area"],
                              "device":  (self.device),

                             }
        

        self.physics = LiftingLine(self.glider_params, self.dt, self.debug_flags)

sc = State_Check()
num_envs = 2

root_states, alpha, Vinf_body_m1, ground_speed = sc.physics.update(sc.glider_states, 
                                                                   sc.actions, 
                                                                   sc.initial_glider_states, 
                                                                   sc.debug_flags)

# print('W_global')
# print(sc.physics.W_global)

# print('Vinf_body_m1')
# print(Vinf_body_m1)

# print('Y_wn1')
# print(sc.physics.Y_wn1)

# print('Forces')
# print(sc.physics.CheckThisForce[:,0,:])

# print('Torques')
# print(sc.physics.CheckThisTorque[:,0,:])

# print('r_global')
# print(sc.physics.r_global_wnm3[0,:,0,:])

print('Force_sum_m3')
print(sc.physics.Force_sum_m3[0,:])

print('Torque_sum_m3')
print(sc.physics.Torque_sum_m3[0,:])