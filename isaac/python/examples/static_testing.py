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
from pytorch3d.transforms import euler_angles_to_matrix, matrix_to_quaternion, quaternion_to_matrix, quaternion_apply, quaternion_invert
from glider_physics import LiftingLine
from joystick import Joystick

class State_Check():
    def __init__(self):
        with open("Dynasoar.yaml", "r") as cfg:
            try:
                self.cfg = yaml.safe_load(cfg)
            except yaml.YAMLError as exc:
                print(exc)

        self.num_envs = 2
        self.create_sim()
        self.create_envs(self.num_envs, 1, 2)
        self.get_state_tensors()

        # optimization flags for pytorch JIT
        torch._C._jit_set_profiling_mode(False)
        torch._C._jit_set_profiling_executor(False)

        # Look at the first env
        cam_pos = gymapi.Vec3(2, 1, 1)
        cam_target = gymapi.Vec3(0, 0, 0)
        self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)

        self.vel_loc = torch.zeros((self.num_envs, 3), device=self.device)
        self.goal_pos = torch.zeros((self.num_envs, 3), device=self.device)

        # self.roll_range = torch.arange(-10, 10, 1)
        self.roll_range = [0]
        # self.pitch_range = torch.arange(-10, 10, 0.1)
        self.pitch_range = [0]
        # self.yaw_range = torch.arange(-10, 10, 1)
        self.yaw_range = [0]
        # self.wind_range = [20.0]
        self.wind_range = torch.arange(1, 30, 1)
        self.joy=Joystick()

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
                        "device":  (self.device)
                        }
        self.debug_flags = self.cfg["env"]["debug_flags"]
                                
        self.physics = LiftingLine(self.glider_params, self.dt, self.debug_flags)

        self.get_state_tensors()

        

    def create_sim(self):
        # initialize gym
        self.gym = gymapi.acquire_gym()

        # parse arguments
        args = gymutil.parse_arguments(description="Joint control Methods Example")

        # create a simulator
        sim_params = gymapi.SimParams()
        sim_params.substeps = 2
        sim_params.dt = 1.0 / 100.0
        self.dt = sim_params.dt

        sim_params.physx.solver_type = 1
        sim_params.physx.num_position_iterations = 4
        sim_params.physx.num_velocity_iterations = 1

        sim_params.physx.num_threads = args.num_threads
        sim_params.physx.use_gpu = args.use_gpu

        sim_params.use_gpu_pipeline = False
        if args.use_gpu_pipeline:
            print("WARNING: Forcing CPU pipeline.")
        self.device = 'cpu'

        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, 0.0)
        self.sim = self.gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)

        if self.sim is None:
            print("*** Failed to create sim")
            quit()

        # create viewer using the default camera properties
        self.viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())
        if self.viewer is None:
            raise ValueError('*** Failed to create viewer')

    def create_envs(self, num_envs, spacing, num_per_row):

        lower = gymapi.Vec3(-spacing, 0.0, -spacing)
        upper = gymapi.Vec3(spacing, 0.0, spacing)

        # add ground plane
        plane_params = gymapi.PlaneParams()
        plane_params.static_friction = 1.0
        plane_params.dynamic_friction = 1.0
        plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
        self.up_axis_idx = 2 #2 for z axis
        self.gym.add_ground(self.sim, plane_params)
        
        # Load asset with default control type of position for all joints
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = False
        asset_options.angular_damping = 0.0
        asset_options.max_angular_velocity = 10000
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS

        asset_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../assets')
        glider_asset_file = "urdf/Glider_S/urdf/Glider_S.urdf"
        glider_asset  = self.gym.load_asset(self.sim, asset_root, glider_asset_file, asset_options)

        # initial root pose for cartpole actors
        initial_pose = gymapi.Transform()
        initial_pose.p = gymapi.Vec3(0.0, 0.0, 1.0)
        initial_pose.r = gymapi.Quat(0, 0.0, 0.0, 1.0)

        # Create environmentself.tensebot_handles = []
        self.glider_handles = []
        self.envs = []
        for i in range(self.num_envs):
            # create env instance
            env_ptr = self.gym.create_env(
                self.sim, lower, upper, num_per_row
            )
            # Creates a tensegrity bot for an environment
            # Returns a list of handles for the support actors
            glider_handle = self.gym.create_actor(env_ptr, glider_asset, initial_pose, 'WalkBot', 0, 0)
            self.glider_handles.append(glider_handle)
            self.envs.append(env_ptr)

        self.num_actors = self.gym.get_actor_count(self.envs[0])
        self.num_bodies = self.gym.get_env_rigid_body_count(self.envs[0])


    def get_state_tensors(self):
        actor_root_state = self.gym.acquire_actor_root_state_tensor(self.sim)
        self.root_states = gymtorch.wrap_tensor(actor_root_state)
        self.initial_root_states = self.root_states.clone()
        self.pos = self.root_states.view(self.num_envs, self.num_actors, 13)[..., 0:3] #num_envs, num_actors, 13 (pos,ori,Lvel,Avel)
        self.ori = self.root_states.view(self.num_envs, self.num_actors, 13)[..., 3:7] #num_envs, num_actors, 13 (pos,ori,Lvel,Avel)
        self.linvel = self.root_states.view(self.num_envs, self.num_actors, 13)[..., 7:10] #num_envs, num_actors, 13 (pos,ori,Lvel,Avel)
        self.angvel = self.root_states.view(self.num_envs, self.num_actors, 13)[..., 10:13] #num_envs, num_actors, 13 (pos,ori,Lvel,Avel)
        self.gym.refresh_actor_root_state_tensor(self.sim)

    def simulation_loop(self):
        # while not self.gym.query_viewer_has_closed(self.viewer):
        self.gym.refresh_actor_root_state_tensor(self.sim)
        self.gym.refresh_dof_state_tensor(self.sim)
        # print(root_states)
        # print(dof_states)
        

        deg2rad = torch.pi/180
        #Scan Through Angles
        
        for W in self.wind_range:
            for R in self.roll_range:
                for P in self.pitch_range:
                    for Y in self.yaw_range:
                        # print(P)
                        #Create a quaternion from the roll pitch and yaw
                        euler_angles = torch.tensor((Y*deg2rad,P*deg2rad,R*deg2rad))
                        RotMat = euler_angles_to_matrix(euler_angles, 'ZYX')
                        Q = matrix_to_quaternion(RotMat)
                        # print(Q)
                        #Record Force and Torque
                        #Plot
                        self.glider_params['wind_speed'] = W
                        self.root_states[:, 0:2] = torch.zeros((2))
                        self.root_states[:, 2] = torch.tensor(1)
                        self.root_states[:,3:7] = torch.roll(Q, -1, 0)
                        self.root_states[:, 7:10] = torch.zeros((3))
                        self.root_states[:, 10:13] = torch.zeros((3))
                        self.gym.set_actor_root_state_tensor(self.sim, gymtorch.unwrap_tensor(self.root_states))
                        
                        actions = torch.zeros((self.num_envs,6))
                        self.physics.update(self.root_states, 
                                            actions, 
                                            self.initial_root_states, 
                                            self.debug_flags)

                        # step the physics
                        # self.gym.simulate(self.sim)
                        # self.gym.fetch_results(self.sim, True)

                        # update the viewer
                        self.gym.step_graphics(self.sim)
                        self.gym.draw_viewer(self.viewer, self.sim, True)
                        
                    
                        # Wait for dt to elapse in real time.
                        # This synchronizes the physics simulation with the rendering rate.
                        # self.gym.sync_frame_time(self.sim)
                    

        print('Done')

        self.gym.destroy_viewer(self.viewer)
        self.gym.destroy_sim(self.sim)


sc = State_Check()
sc.simulation_loop()