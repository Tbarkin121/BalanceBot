# Copyright (c) 2018-2022, NVIDIA Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np
import os
import torch

from isaacgym import gymtorch
from isaacgym import gymapi
from isaacgym.torch_utils import *
from isaacgym.terrain_utils import *
# from tasks.base.vec_task import VecTask
from tasks.base.vec_task_glider import VecTaskGlider

from typing import Tuple, Dict
from .glider_physics import LiftingLine
from .joystick import Joystick
import pygame
import time

from pytorch3d.transforms import quaternion_to_matrix, quaternion_apply, euler_angles_to_matrix, quaternion_invert
from .csv_logger import CSVLogger

class Dynasoar(VecTaskGlider):

    def __init__(self, cfg, rl_device, sim_device, graphics_device_id, headless, virtual_screen_capture, force_render):
        self.cfg = cfg

        # Starting State Ranges 
        self.roll_range = self.cfg["env"]["randomRanges"]["roll"]
        self.pitch_range = self.cfg["env"]["randomRanges"]["pitch"]
        self.yaw_range = self.cfg["env"]["randomRanges"]["yaw"]
        self.vinf_range = self.cfg["env"]["randomRanges"]["vinf"]
        self.height_range = self.cfg["env"]["randomRanges"]["height"]
        self.target_range = self.cfg["env"]["randomRanges"]["target_range"]
        self.target_radius = self.cfg["env"]["randomRanges"]["target_radius"]
        self.debug_flags = self.cfg["env"]["debug_flags"]
        self.obs_scales = self.cfg["env"]["obs_scales"]
        # normalization
        self.lin_vel_scale = self.cfg["env"]["learn"]["linearVelocityScale"]
        self.ang_vel_scale = self.cfg["env"]["learn"]["angularVelocityScale"]
        self.dof_pos_scale = self.cfg["env"]["learn"]["dofPositionScale"]
        self.dof_vel_scale = self.cfg["env"]["learn"]["dofVelocityScale"]
        self.action_scale_main = self.cfg["env"]["control"]["actionScale_main"]
        self.action_scale_tail = self.cfg["env"]["control"]["actionScale_tail"]

        # reward scales
        self.rew_scales = {}
        self.rew_scales["lin_vel_xy"] = self.cfg["env"]["learn"]["linearVelocityXYRewardScale"]
        self.rew_scales["ang_vel_z"] = self.cfg["env"]["learn"]["angularVelocityZRewardScale"]
        self.rew_scales["torque"] = self.cfg["env"]["learn"]["torqueRewardScale"]

        # randomization
        self.randomization_params = self.cfg["task"]["randomization_params"]
        self.randomize = self.cfg["task"]["randomize"]

        # plane params
        self.plane_static_friction = self.cfg["env"]["plane"]["staticFriction"]
        self.plane_dynamic_friction = self.cfg["env"]["plane"]["dynamicFriction"]
        self.plane_restitution = self.cfg["env"]["plane"]["restitution"]

        # base init state
        pos = self.cfg["env"]["baseInitState"]["pos"]
        rot = self.cfg["env"]["baseInitState"]["rot"]
        v_lin = self.cfg["env"]["baseInitState"]["vLinear"]
        v_ang = self.cfg["env"]["baseInitState"]["vAngular"]
        state = pos + rot + v_lin + v_ang

        self.base_init_state = state

        self.cfg["env"]["numObservations"] = 10
        self.cfg["env"]["numActions"] = 3

        super().__init__(config=self.cfg, rl_device=rl_device, sim_device=sim_device, graphics_device_id=graphics_device_id, headless=headless, virtual_screen_capture=virtual_screen_capture, force_render=force_render)

        # other
        self.dt = self.sim_params.dt
        self.max_episode_length_s = self.cfg["env"]["learn"]["episodeLength_s"]
        self.max_episode_length = int(self.max_episode_length_s / self.dt + 0.5)

        for key in self.rew_scales.keys():
            self.rew_scales[key] *= self.dt

        if self.viewer != None:
            p = self.cfg["env"]["viewer"]["pos"]
            lookat = self.cfg["env"]["viewer"]["lookat"]
            cam_pos = gymapi.Vec3(p[0], p[1], p[2])
            cam_target = gymapi.Vec3(lookat[0], lookat[1], lookat[2])
            self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)

        # get gym state tensors
        actor_root_state = self.gym.acquire_actor_root_state_tensor(self.sim)

        self.gym.refresh_actor_root_state_tensor(self.sim)

        # create some wrapper tensors for different slices
        self.root_states = gymtorch.wrap_tensor(actor_root_state)
        # *2 because we are loading 2 assets
        self.glider_states = self.root_states.view(self.num_envs*2, 13)[0::2, :]
        self.goal_states = self.root_states.view(self.num_envs*2, 13)[1::2, :]
        
        self.glider_states_prev = self.glider_states.clone()

        self.commands = torch.zeros(self.num_envs, 3, dtype=torch.float, device=self.device, requires_grad=False)
        self.commands_y = self.commands.view(self.num_envs, 3)[..., 1]
        self.commands_x = self.commands.view(self.num_envs, 3)[..., 0]
        self.commands_yaw = self.commands.view(self.num_envs, 3)[..., 2]
        
        # initialize some data used later on
        self.extras = {}
        self.initial_root_states = self.root_states.clone()
        self.initial_glider_states = self.glider_states.clone()
        self.initial_glider_states[:] = to_torch(self.base_init_state, device=self.device, requires_grad=False)
        
        self.gravity_vec = to_torch(get_axis_params(-1., self.up_axis_idx), device=self.device).repeat((self.num_envs, 1))
        self.actions = torch.zeros(self.num_envs, self.num_actions, dtype=torch.float, device=self.device, requires_grad=False)

        
        self.glider_params = {"wings": self.cfg["env"]["glider"]["wings"]["count"],
                              "station_pts": self.cfg["env"]["glider"]["station_pts"],
                              "cla": self.cfg["env"]["glider"]["cla"],
                              "rho": self.cfg["env"]["glider"]["rho"],
                              "eps": self.cfg["env"]["glider"]["eps"],
                              "envs" : (self.num_envs),
                              "actions" : (self.num_actions),
                              "observations" : (self.num_observations),
                              "TW1": torch.tensor(self.cfg["env"]["glider"]["wings"]["offsets"]["W1"]),
                              "TW2": torch.tensor(self.cfg["env"]["glider"]["wings"]["offsets"]["W2"]),
                              "TW3": torch.tensor(self.cfg["env"]["glider"]["wings"]["offsets"]["W3"]),
                              "TW4": torch.tensor(self.cfg["env"]["glider"]["wings"]["offsets"]["W4"]),
                              "HW1": torch.tensor(self.cfg["env"]["glider"]["wings"]["headings"]["W1"]),
                              "HW2": torch.tensor(self.cfg["env"]["glider"]["wings"]["headings"]["W2"]),
                              "HW3": torch.tensor(self.cfg["env"]["glider"]["wings"]["headings"]["W3"]),
                              "HW4": torch.tensor(self.cfg["env"]["glider"]["wings"]["headings"]["W4"]),
                              "C1":  torch.tensor(self.cfg["env"]["glider"]["wings"]["chords"]["W1"]),
                              "C2":  torch.tensor(self.cfg["env"]["glider"]["wings"]["chords"]["W2"]),
                              "C3":  torch.tensor(self.cfg["env"]["glider"]["wings"]["chords"]["W3"]),
                              "C4":  torch.tensor(self.cfg["env"]["glider"]["wings"]["chords"]["W4"]),
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
        self.target_pos = torch.zeros((self.num_envs, 2), device=self.device, dtype=torch.float)
        
        self.goal_reached = torch.ones(self.num_envs, device=self.device, dtype=torch.long)

        self.reset_idx(torch.arange(self.num_envs, device=self.device))
        self.reset_goals(torch.arange(self.num_envs, device=self.device))
        if(self.debug_flags['joystick']):
            self.joy = Joystick()

        self.camera_setup()
        self.input_delay = 10
        self.input_delay_counter = 0


    def create_sim(self):
        self.up_axis_idx = 2 # index of up axis: Y=1, Z=2
        self.sim = super().create_sim(self.device_id, self.graphics_device_id, self.physics_engine, self.sim_params)
        # self._create_ground_plane()
        self._create_custom_ground_plane()
        self._create_envs(self.num_envs, self.cfg["env"]['envSpacing'], int(np.sqrt(self.num_envs)))
        # If randomizing, apply once immediately on startup before the fist sim step
        if self.randomize:
            self.apply_randomizations(self.randomization_params)

    def _create_ground_plane(self):
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
        plane_params.static_friction = self.plane_static_friction
        plane_params.dynamic_friction = self.plane_dynamic_friction
        self.gym.add_ground(self.sim, plane_params)
        
    def _create_custom_ground_plane(self):
        terrain_width = 2000.
        terrain_length = 2000.
        horizontal_scale = 3.0  # [m]
        vertical_scale = 10.0  # [m]
        num_rows = int(terrain_width/horizontal_scale)
        num_cols = int(terrain_length/horizontal_scale)
        sub_terrain = SubTerrain(width=num_rows, length=num_cols, vertical_scale=vertical_scale, horizontal_scale=horizontal_scale)
        heightfield = discrete_obstacles_terrain(sub_terrain, max_height=vertical_scale, min_size=10., max_size=100., num_rects=10000).height_field_raw
        
        # add the terrain as a triangle mesh
        vertices, triangles = convert_heightfield_to_trimesh(heightfield, horizontal_scale=horizontal_scale, vertical_scale=vertical_scale, slope_threshold=1.5)
        vertices = vertices - vertical_scale
        tm_params = gymapi.TriangleMeshParams()
        tm_params.nb_vertices = vertices.shape[0]
        tm_params.nb_triangles = triangles.shape[0]
        tm_params.transform.p.x = -terrain_width/2.0
        tm_params.transform.p.y = -terrain_width/2.0
        self.gym.add_triangle_mesh(self.sim, vertices.flatten(), triangles.flatten(), tm_params)
    
    def _create_envs(self, num_envs, spacing, num_per_row):
        asset_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../../assets')
        glider_asset_file = "urdf/Glider_S/urdf/Glider_S.urdf"
        goal_asset_file = "urdf/GoalPin/urdf/GoalPin.urdf"
        asset_options = gymapi.AssetOptions()
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_NONE
        asset_options.collapse_fixed_joints = True
        asset_options.replace_cylinder_with_capsule = True
        asset_options.density = 0.001
        asset_options.angular_damping = 0.0
        asset_options.linear_damping = 0.0
        asset_options.armature = 0.0
        asset_options.thickness = 0.01
        asset_options.disable_gravity = False

        glider_asset = self.gym.load_asset(self.sim, asset_root, glider_asset_file, asset_options)
        goal_asset = self.gym.load_asset(self.sim, asset_root, goal_asset_file, asset_options)
        self.num_dof = self.gym.get_asset_dof_count(glider_asset)
        self.num_bodies = self.gym.get_asset_rigid_body_count(glider_asset)

        start_pose = gymapi.Transform()
        start_pose.p = gymapi.Vec3(*self.base_init_state[:3])

        body_names = self.gym.get_asset_rigid_body_names(glider_asset)
        self.dof_names = self.gym.get_asset_dof_names(glider_asset)
        self.base_index = 0

        env_lower = gymapi.Vec3(-spacing, -spacing, 0.0)
        env_upper = gymapi.Vec3(spacing, spacing, spacing)
        self.glider_handles = []
        self.goal_handles = []
        self.envs = []

        for i in range(self.num_envs):
            # create env instance
            env_ptr = self.gym.create_env(self.sim, env_lower, env_upper, num_per_row)
            glider_handle = self.gym.create_actor(env_ptr, glider_asset, start_pose, "glider", i, 1, 0)
            goal_handle = self.gym.create_actor(env_ptr, goal_asset, start_pose, "goal", i, 1, 0)
            
            rand_color = torch.rand((3), device=self.device)
            self.gym.set_rigid_body_color(env_ptr, glider_handle, 0, gymapi.MESH_VISUAL, gymapi.Vec3(rand_color[0],rand_color[1],rand_color[2]))
            self.gym.set_rigid_body_color(env_ptr, goal_handle, 0, gymapi.MESH_VISUAL, gymapi.Vec3(rand_color[0],rand_color[1],rand_color[2]))

            self.envs.append(env_ptr)
            self.glider_handles.append(glider_handle)
            self.goal_handles.append(goal_handle)

        self.base_index = self.gym.find_actor_rigid_body_handle(self.envs[0], self.glider_handles[0], "base")

    def pre_physics_step(self, actions):
        self.actions = actions.clone().to(self.device)


    def physics_step(self): 
        self.gym.refresh_actor_root_state_tensor(self.sim)
        # print('!!')
        # print(self.root_states)
        # print(self.glider_states)
        # print(self.goal_states)
        actions = self.actions

        if(self.debug_flags['joystick']):
            a = self.joy.get_axis()
            b = self.joy.get_button()
            d = self.joy.get_dpad()
            self.input_delay_counter += 1

            if(self.debug_flags['control_en']):
                a[2] = a[2] - 1 # The triggers are [0,2] instead of [-1,1]
                a[5] = a[5] - 1 
                # actions[self.env_2_watch,] = torch.tensor((a[3], a[1], a[2], a[5]), device=self.device)
                actions[self.env_2_watch,] = torch.tensor((a[3], a[1]), device=self.device)
            
            if(b[0] and self.input_delay_counter > self.input_delay):
                self.debug_flags['cam_follow'] = ~self.debug_flags['cam_follow']
                self.input_delay_counter = 0

            if(b[5] and self.input_delay_counter > self.input_delay):
                self.debug_flags['control_en'] = ~self.debug_flags['control_en']
                self.input_delay_counter = 0

        # Action Scaling
        # actions[:,0:2] = actions[:,0:2] * self.action_scale_tail #0,1
        # actions[:,3:5] = actions[:,3:5] * self.action_scale_main #3,4
        actions[:,1] = actions[:,1] * self.action_scale_tail #0,1
        actions[:,0] = actions[:,0] * self.action_scale_main #3,4
        # actions[:,2] = (actions[:,2]+1)/2.0 #Left Trigger [0,1] (Only Positive Inputs?)
        # actions[:,3] = (actions[:,3]+1)/2.0 #Right Trigger [0,1] (Only Positive Inputs?)
        # print(actions[self.env_2_watch,:])
        self.glider_states[...], self.alpha, self.obs_raw = self.physics.update(self.glider_states, 
                                                                            actions, 
                                                                            self.debug_flags)
        if(self.debug_flags['plots_enable']):
            self.draw_all_line_graphs()

        if(self.debug_flags['cam_follow']):
            if(self.debug_flags['joystick']):
                self.camera_follow(d)
            else:
                self.camera_follow([0,0])

        if(self.debug_flags['print']):
            #Contain printouts to debug function calls so they aren't scattered all over the place
            self.physics.debug()

        # self.print_all_tensors()
        # print(self.obs_buf[0],...)

    def post_physics_step(self):
        self.progress_buf += 1

        env_ids = self.reset_buf.nonzero(as_tuple=False).squeeze(-1)
        if len(env_ids) > 0:
            self.reset_idx(env_ids)

        env_ids = self.goal_reached.nonzero(as_tuple=False).squeeze(-1)
        if len(env_ids) > 0:
            self.reset_goals(env_ids)

        self.gym.set_actor_root_state_tensor(self.sim, gymtorch.unwrap_tensor(self.root_states))
        self.compute_observations()
        self.compute_reward(self.actions)
        # print(self.obs_buf[0,...])
        # print(self.rew_buf[0])

    def compute_reward(self, actions):
        alpha_fail = torch.abs(self.alpha) > 0.17
        alpha_reset = torch.where(alpha_fail, torch.ones_like(alpha_fail), torch.zeros_like(alpha_fail))
        alpha_reset = torch.sum(alpha_reset, dim=0)
        
        # ground_speed = self.obs_buf[:,1]
        # air_speed = self.obs_buf[:,2]
        # speed_reward = (ground_speed)**2 + (air_speed)**2

        # vec_2_target = self.target_pos - self.glider_states[:,0:2]
        # dist_2_target = torch.norm(vec_2_target, dim=1)
        # dist_reward = 1/(dist_2_target + 0.00001)
        # # Don't go above 1 dist reward...
        # reward_limit_idx = dist_reward>1.0
        # dist_reward[reward_limit_idx] = 1.0

        # print('!!!')
        # print(speed_reward)
        # print(dist_reward)
        self.rew_buf[:], self.reset_buf[:], self.goal_reached[:] = compute_glider_reward(
            # tensors
            self.glider_states,
            self.glider_states_prev,
            self.actions,
            self.progress_buf,
            self.alpha,
            self.obs_raw[:,2], #Air Speed
            self.obs_buf,
            self.target_pos,

            # Dict
            self.rew_scales,
            # other
            self.base_index,
            self.max_episode_length,
            self.num_envs,
            self.target_radius,
        )
        self.glider_states_prev = self.glider_states.clone()

    def compute_observations(self):
        # State Refresh happens in 
        base_quat = self.glider_states[:, 3:7]
        rot_mat = quaternion_to_matrix(torch.roll(base_quat, 1, 1))
        angles = self.physics.rotationMatrixToEulerAngles(rot_mat) # This is an ZYX format but returns X,Y,Z... 
        # angles = matrix_to_euler_angles(rot_mat, 'XYZ')
        ang_scale = 1/(torch.pi)

        roll = torch.unsqueeze(angles[:,0], dim=-1) * ang_scale
        pitch = torch.unsqueeze(angles[:,1], dim=-1) * ang_scale
        yaw = torch.unsqueeze(angles[:,2], dim=-1) * ang_scale

        pos = self.glider_states[:, 0:3] * torch.tensor([self.obs_scales['height']], device='cuda')
        # height = torch.unsqueeze(height, dim=-1)

        air_speed = self.obs_raw[:,2].reshape(self.num_envs,1)  * torch.tensor([self.obs_scales['air_speed']], device='cuda')

        ground_speed = self.obs_raw[:,1].reshape(self.num_envs,1)  * torch.tensor([self.obs_scales['ground_speed']], device='cuda')
        
        v_to_target_g = self.target_pos - self.glider_states[:, 0:2]
        v_to_target_norm = torch.unsqueeze(torch.norm(v_to_target_g, dim=1),dim=1)
        v_to_target_g = torch.unsqueeze(v_to_target_g/v_to_target_norm, dim=-1)
        v_to_target_g = torch.cat((v_to_target_g, torch.ones((self.num_envs,1,1),device=self.device)), dim=1)

        euler_angles = torch.cat((-yaw/ang_scale,
                                 torch.zeros_like(yaw),
                                 torch.zeros_like(yaw)), dim=-1)
                                 
        R = euler_angles_to_matrix(euler_angles, 'ZYX')
        vec_2_target_p = torch.matmul(R,v_to_target_g)

        battery_percent = self.physics.battery_states[:,0]/self.physics.batt_man.max_energy
        battery_percent = battery_percent.reshape(self.num_envs,1)

        self.obs_buf[:] = torch.cat((
                                     torch.unsqueeze(pos[:,2],dim=1), # Height
                                     ground_speed,
                                     air_speed,
                                     roll,
                                     pitch,
                                     yaw,
                                     battery_percent,
                                     vec_2_target_p[:,0:2, 0], #X,Y component of target heading vector in plane frame
                                     v_to_target_norm/1000,
                                     ), dim=-1)


        # self.obs_buf[:] = compute_glider_observations(  # tensors
        #                                                 self.root_states,
        #                                                 self.commands,
        #                                                 self.gravity_vec,
        #                                                 self.actions,
        #                                                 self.alpha,
        #                                                 self.Vinf_body_m1,
        #                                                 self.ground_speed,
        #                                                 # scales
        #                                                 self.lin_vel_scale,
        #                                                 self.ang_vel_scale,
        #                                                 self.dof_pos_scale,
        #                                                 self.dof_vel_scale
        # )


    def reset_idx(self, env_ids):
        # State Update moved to Post Physics Step
        # Randomization can happen only at reset time, since it can reset actor positions on GPU
        if self.randomize:
            self.apply_randomizations(self.randomization_params)

        deg_2_rad = torch.pi/180.0
        roll_init = torch_rand_float(self.roll_range[0], self.roll_range[1], (len(env_ids), 1), device=self.device).squeeze()
        pitch_init = torch_rand_float(self.pitch_range[0], self.pitch_range[1], (len(env_ids), 1), device=self.device).squeeze()
        yaw_init = torch_rand_float(self.yaw_range[0], self.yaw_range[1], (len(env_ids), 1), device=self.device).squeeze()
        height_init = torch_rand_float(self.height_range[0], self.height_range[1], (len(env_ids), 1), device=self.device).squeeze()
        vinf_init = torch_rand_float(self.vinf_range[0], self.vinf_range[1], (len(env_ids), 1), device=self.device).squeeze()
        
        if(roll_init.dim() == 0):
            roll_init = torch.unsqueeze(roll_init, dim=-1)
            pitch_init = torch.unsqueeze(pitch_init, dim=-1)
            yaw_init = torch.unsqueeze(yaw_init, dim=-1)
            height_init = torch.unsqueeze(height_init, dim=-1)
            vinf_init = torch.unsqueeze(vinf_init, dim=-1)

        wind_vec = self.physics.wind_function(height_init)
            
        new_quat = quat_from_euler_xyz(roll_init*deg_2_rad, pitch_init*deg_2_rad, yaw_init*deg_2_rad)
        

        euler_angles = torch.cat((torch.reshape(yaw_init,(len(env_ids),1))*deg_2_rad,
                                 torch.reshape(pitch_init,(len(env_ids),1))*deg_2_rad,
                                 torch.reshape(roll_init,(len(env_ids),1))*deg_2_rad), dim=-1)
                                 
        new_r_mat = euler_angles_to_matrix(euler_angles, 'ZYX')
        v_apparent = -new_r_mat[:,:,0]*torch.reshape(vinf_init, (len(env_ids),1))
        v_global = v_apparent + torch.squeeze(wind_vec)

        modified_initial_state = self.initial_glider_states.clone()
        modified_initial_state[env_ids, 2] = height_init
        modified_initial_state[env_ids, 3:7] = new_quat
        modified_initial_state[env_ids, 7:10] = v_global
        
        self.glider_states[env_ids, ...] = modified_initial_state[env_ids, ...]
    
        self.progress_buf[env_ids] = 0
        self.reset_buf[env_ids] = 1
        self.reset_goals(env_ids)
        self.physics.batt_man.reset_battery_states(env_ids, self.physics.battery_states)


    def reset_goals(self, env_ids):
        self.target_pos[env_ids, :] = torch_rand_float(self.target_range[0], self.target_range[1], (len(env_ids), 2), device=self.device).squeeze()
        self.goal_states[env_ids, 0:2] = self.target_pos[env_ids, 0:2]
        self.goal_states[env_ids, 2] = -20.0
        self.goal_reached[env_ids] = 0

    def wing_graph(self, wing_num=0, env_num=0, scale=1):
        num_lines = self.glider_params["station_pts"]-1
        line_vertices = torch.zeros((num_lines*2,3), device=self.device, dtype=torch.float)

        line_color = torch.tensor([[1.0, 1.0, 1.0]], device=self.device, dtype=torch.float).repeat(num_lines,1)
        line_vertices[0::2,:] = self.physics.r_global_wnm3[wing_num,0:-1,env_num,:] + self.glider_states[0,0:3]
        line_vertices[1::2,:] = self.physics.r_global_wnm3[wing_num,1:,env_num,:] + self.glider_states[0,0:3]

        self.gym.add_lines(self.viewer, self.envs[0], num_lines, line_vertices.cpu().detach(), line_color.cpu().detach())


    def force_graph(self, wing_num=0, env_num=0, scale=1):
        F_global = self.physics.F_global_wnm3.clone()

        num_lines = self.glider_params["station_pts"]-1
        line_vertices = torch.zeros((num_lines*2,3), device=self.device, dtype=torch.float)

        line_color = torch.tensor([[1.0, 1.0, 0.0]], device=self.device, dtype=torch.float).repeat(num_lines,1)
        line_vertices[0::2,:] = self.physics.r_global_wnm3[wing_num,0:-1,env_num,:] + scale*F_global[wing_num,0:-1,env_num,:] + self.glider_states[0,0:3]
        line_vertices[1::2,:] = self.physics.r_global_wnm3[wing_num,1:,env_num,:] + scale*F_global[wing_num,1:,env_num,:] + self.glider_states[0,0:3]

        self.gym.add_lines(self.viewer, self.envs[0], num_lines, line_vertices.cpu().detach(), line_color.cpu().detach())


    def lift_graph(self, wing_num=0, env_num=0, scale=1):
        L_global = self.physics.Lift_global_wnm3.clone()

        num_lines = self.glider_params["station_pts"]-1
        line_vertices = torch.zeros((num_lines*2,3), device=self.device, dtype=torch.float)

        line_color = torch.tensor([[0.0, 1.0, 0.0]], device=self.device, dtype=torch.float).repeat(num_lines,1)
        line_vertices[0::2,:] = self.physics.r_global_wnm3[wing_num,0:-1,env_num,:] + scale*L_global[wing_num,0:-1,env_num,:] + self.glider_states[0,0:3]
        line_vertices[1::2,:] = self.physics.r_global_wnm3[wing_num,1:,env_num,:] + scale*L_global[wing_num,1:,env_num,:] + self.glider_states[0,0:3]

        self.gym.add_lines(self.viewer, self.envs[0], num_lines, line_vertices.cpu().detach(), line_color.cpu().detach())

    def drag_graph(self, wing_num=0, env_num=0, scale=1):
        D_global = self.physics.Drag_global_wnm3.clone()
        num_lines = self.glider_params["station_pts"]-1
        line_vertices = torch.zeros((num_lines*2,3), device=self.device, dtype=torch.float)

        line_color = torch.tensor([[1.0, 0.0, 0.0]], device=self.device, dtype=torch.float).repeat(num_lines,1)
        line_vertices[0::2,:] = self.physics.r_global_wnm3[wing_num,0:-1,env_num,:] + scale*D_global[wing_num,0:-1,env_num,:] + self.glider_states[0,0:3]
        line_vertices[1::2,:] = self.physics.r_global_wnm3[wing_num,1:,env_num,:] + scale*D_global[wing_num,1:,env_num,:] + self.glider_states[0,0:3]

        self.gym.add_lines(self.viewer, self.envs[0], num_lines, line_vertices.cpu().detach(), line_color.cpu().detach())

    def wind_graph(self, vec_scale=1, sym_scale=1, sym_offset=(0.0, 0.0, 0.5)):
        sym_offset = torch.tensor(sym_offset, device=self.device)
        
        num_lines = 1
        line_vertices = torch.zeros((num_lines*2,3), device=self.device, dtype=torch.float)
        line_color = torch.tensor([[0.0, 1.0, 1.0]], device=self.device, dtype=torch.float).repeat(num_lines,1)
        line_vertices[0,:] = torch.zeros_like(self.physics.W_global[0, :, 0], device=self.device) + self.glider_states[0,0:3] + sym_offset
        line_vertices[1,:] = vec_scale*self.physics.WAG[0, :] + self.glider_states[0,0:3] + sym_offset
        
        self.gym.add_lines(self.viewer, self.envs[0], num_lines, line_vertices.cpu().detach(), line_color.cpu().detach())
        
        num_lines = 4 
        line_vertices = torch.zeros((num_lines*2,3), device=self.device, dtype=torch.float)
        line_color = torch.tensor([[0.0, 0.6, 1.0]], device=self.device, dtype=torch.float).repeat(num_lines,1)
        vert_ = torch.tensor([[-sym_scale, 0.0, sym_scale],
                               [sym_scale, 0.0, -sym_scale],
                               [-sym_scale, 0.0, -sym_scale],
                               [sym_scale, 0.0, sym_scale],
                               [-sym_scale, 0.0, sym_scale]] , device=self.device)
        line_vertices[0::2,:] = vert_[0:-1,:] + sym_offset + self.glider_states[0,0:3]
        line_vertices[1::2,:] = vert_[1:,  :] + sym_offset + self.glider_states[0,0:3]

        self.gym.add_lines(self.viewer, self.envs[0], num_lines, line_vertices.cpu().detach(), line_color.cpu().detach())

    def destination_graph(self, vec_scale=1, sym_scale=1, sym_offset=(0.0, 0.0, 0.5)):
        sym_offset = torch.tensor(sym_offset, device=self.device)
        
        num_lines = 1
        line_vertices = torch.zeros((num_lines*2,3), device=self.device, dtype=torch.float)
        line_color = torch.tensor([[0.0, 1.0, 1.0]], device=self.device, dtype=torch.float).repeat(num_lines,1)
        line_vertices[0,:] = torch.zeros_like(self.physics.W_global[0, :, 0], device=self.device) + self.glider_states[0,0:3] + sym_offset
        line_vertices[1,:] = vec_scale*self.physics.WAG[0, :] + self.glider_states[0,0:3] + sym_offset
        
        self.gym.add_lines(self.viewer, self.envs[0], num_lines, line_vertices.cpu().detach(), line_color.cpu().detach())
        
        num_lines = 4 
        line_vertices = torch.zeros((num_lines*2,3), device=self.device, dtype=torch.float)
        line_color = torch.tensor([[0.0, 0.6, 1.0]], device=self.device, dtype=torch.float).repeat(num_lines,1)
        vert_ = torch.tensor([[-sym_scale, 0.0, sym_scale],
                               [sym_scale, 0.0, -sym_scale],
                               [-sym_scale, 0.0, -sym_scale],
                               [sym_scale, 0.0, sym_scale],
                               [-sym_scale, 0.0, sym_scale]] , device=self.device)
        line_vertices[0::2,:] = vert_[0:-1,:] + sym_offset + self.glider_states[0,0:3]
        line_vertices[1::2,:] = vert_[1:,  :] + sym_offset + self.glider_states[0,0:3]

        self.gym.add_lines(self.viewer, self.envs[0], num_lines, line_vertices.cpu().detach(), line_color.cpu().detach())

    def heading_graph(self, vec_scale=1, sym_scale=1, sym_offset=(0.0, 0.0, 0.25)):
        sym_offset = torch.tensor(sym_offset, device=self.device)
        
        num_lines = 1
        line_vertices = torch.zeros((num_lines*2,3), device=self.device, dtype=torch.float)
        line_color = torch.tensor([[0.0, 1.0, 0.0]], device=self.device, dtype=torch.float).repeat(num_lines,1)
        line_vertices[0,:] = torch.zeros_like(self.physics.W_global[0, :, 0], device=self.device) + self.glider_states[0,0:3] + sym_offset
        north = torch.tensor([1.0, 0.0, 0.0], device=self.device)
        line_vertices[1,:] = vec_scale*north + self.glider_states[0,0:3] + sym_offset
        
        self.gym.add_lines(self.viewer, self.envs[0], num_lines, line_vertices.cpu().detach(), line_color.cpu().detach())
        
        num_lines = 4 
        line_vertices = torch.zeros((num_lines*2,3), device=self.device, dtype=torch.float)
        line_color = torch.tensor([[0.2, 0.1, 0.2]], device=self.device, dtype=torch.float).repeat(num_lines,1)
        vert_ = torch.tensor([[-sym_scale, 0.0, sym_scale],
                               [sym_scale, 0.0, -sym_scale],
                               [-sym_scale, 0.0, -sym_scale],
                               [sym_scale, 0.0, sym_scale],
                               [-sym_scale, 0.0, sym_scale]] , device=self.device)
        line_vertices[0::2,:] = vert_[0:-1,:] + sym_offset + self.glider_states[0,0:3]
        line_vertices[1::2,:] = vert_[1:,  :] + sym_offset + self.glider_states[0,0:3]

        self.gym.add_lines(self.viewer, self.envs[0], num_lines, line_vertices.cpu().detach(), line_color.cpu().detach())
    
    def speedometer(self, location=[0.0, 0.0], max_speed=100):
        pass
        # num_lines = 24
        # circle_pts = num_lines+1
        # circle_radius = 1
        # thetas = torch.unsqueeze(torch.linspace(0, 2*torch.pi, circle_pts, device=self.device), dim=-1)   
        # x_pts = circle_radius*torch.cos(thetas)
        # y_pts = circle_radius*torch.sin(thetas)
        # z_pts = torch.zeros_like(x_pts, device=self.device)
        # print(x_pts.shape)
        # line_verts = torch.cat( (x_pts, y_pts, z_pts), dim=1) + self.root_states[0,0:3]
        # line_color = torch.tensor([[1.0, 1.0, 1.0]], device=self.device, dtype=torch.float).repeat(num_lines,1)
        # self.gym.add_lines(self.viewer, self.envs[0], num_lines, line_verts.cpu().detach(), line_color.cpu().detach())
        # # Draw Circle 
        # # Draw Arrow

    def draw_all_line_graphs(self):
        self.gym.clear_lines(self.viewer)
        self.wind_graph(vec_scale = 1, sym_scale = 0.1)
        self.heading_graph(vec_scale = 1, sym_scale = 0.1)

        self.wing_graph(wing_num=0, env_num=self.env_2_watch, scale=0.1)
        self.wing_graph(wing_num=1, env_num=self.env_2_watch, scale=0.1)
        self.wing_graph(wing_num=2, env_num=self.env_2_watch, scale=0.1)
        self.wing_graph(wing_num=3, env_num=self.env_2_watch, scale=0.1)

        self.lift_graph(wing_num=0, env_num=self.env_2_watch, scale=0.1)
        self.lift_graph(wing_num=1, env_num=self.env_2_watch, scale=0.1)
        self.lift_graph(wing_num=2, env_num=self.env_2_watch, scale=0.1)
        self.lift_graph(wing_num=3, env_num=self.env_2_watch, scale=0.1)

        self.drag_graph(wing_num=0, env_num=self.env_2_watch, scale=0.1)
        self.drag_graph(wing_num=1, env_num=self.env_2_watch, scale=0.1)
        self.drag_graph(wing_num=2, env_num=self.env_2_watch, scale=0.1)
        self.drag_graph(wing_num=3, env_num=self.env_2_watch, scale=0.1)
        
        self.force_graph(wing_num=0, env_num=self.env_2_watch, scale=0.1)
        self.force_graph(wing_num=1, env_num=self.env_2_watch, scale=0.1)
        self.force_graph(wing_num=2, env_num=self.env_2_watch, scale=0.1)
        self.force_graph(wing_num=3, env_num=self.env_2_watch, scale=0.1)

        self.speedometer()


    def camera_setup(self):
        self.env_2_watch = self.debug_flags['cam_target']

    def camera_follow(self, d_pad):
        if(self.input_delay_counter > self.input_delay):
            if(d_pad[1] == 1):
                self.env_2_watch = int(torch.randint(0, self.num_envs, (1,)))
                self.input_delay_counter = 0
                self.debug_flags['control_en'] = False
            if(d_pad[1] == -1):
                self.env_2_watch = 0
                self.input_delay_counter = 0
                self.debug_flags['control_en'] = False
            if(d_pad[0] == 1):
                self.env_2_watch += 1
                self.input_delay_counter = 0
                self.debug_flags['control_en'] = False
            if(d_pad[0] == -1):
                self.env_2_watch -= 1
                self.input_delay_counter = 0
                self.debug_flags['control_en'] = False
            
        # Wrap Around
        if(self.env_2_watch > self.num_envs-1):
            self.env_2_watch = 0
        if(self.env_2_watch < 0):
            self.env_2_watch = int(self.num_envs-1)

        
        num_per_row = int(np.sqrt(self.num_envs))
        spacing = 2
        row_num = int(self.env_2_watch/num_per_row)*spacing
        col_num = (self.env_2_watch % num_per_row)*spacing        
        env_offset = gymapi.Vec3(col_num, row_num, 0.0)
        cam_offset = torch.tensor([[4.0], [0.0], [0.5]], device=self.device)


        Quat = torch.unsqueeze(self.glider_states[self.env_2_watch, 3:7], 0)
        # Quat = torch.unsqueeze(quaternion_invert(self.glider_states[self.env_2_watch, 3:7]), 0)
        Quat = torch.roll(Quat, 1, 1)
        cam_offset_rot = quaternion_apply(Quat, torch.permute(cam_offset,(1,0)))
        cam_o = gymapi.Vec3(cam_offset_rot[0,0],cam_offset_rot[0,1],cam_offset_rot[0,2])
        cam_pos = gymapi.Vec3(self.glider_states[self.env_2_watch,0], self.glider_states[self.env_2_watch,1], self.glider_states[self.env_2_watch,2])
        # cam_pos = gymapi.Vec3(self.glider_states[self.env_2_watch,0], self.glider_states[self.env_2_watch,1], 20.0)

        cam_target = gymapi.Vec3(self.glider_states[self.env_2_watch,0], self.glider_states[self.env_2_watch,1], self.glider_states[self.env_2_watch,2])
        self.gym.viewer_camera_look_at(self.viewer, None, cam_pos+cam_o+env_offset, cam_target+env_offset)
        # self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)

    def print_all_tensors(self):
        name = ''
        value = 0
        num_tensors = 0
        for name, value in zip(vars(self).keys(), vars(self).values()):
            if(torch.is_tensor(value)):
                print('Tensor Name  : {}'.format(name))
                print('Tensor Shape : {}'.format(value.shape))
                num_tensors += 1
        print('Total Tensors : {}'.format(num_tensors))

#####################################################################
###=========================jit functions=========================###
#####################################################################


@torch.jit.script
def compute_glider_reward(
    # tensors
    glider_states,
    glider_states_prev,
    actions,
    episode_lengths,
    alphas,
    V_inf,
    obs,
    target_pos,
    # Dict
    rew_scales,
    # other
    base_index,
    max_episode_length,
    num_envs,
    target_radius
):
    # type: (Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Dict[str, float], int, int, int, float) -> Tuple[Tensor, Tensor, Tensor]

    # prepare quantities (TODO: return from obs ?)
    base_pos = glider_states[:,0:3]
    base_quat = glider_states[:, 3:7]
    base_lin_vel = quat_rotate_inverse(base_quat, glider_states[:, 7:10])
    base_ang_vel = quat_rotate_inverse(base_quat, glider_states[:, 10:13])

    # Energy Rewards
    mass = 1.0
    gravity = 9.81
    # height = obs[:,2]
    # ground_speed = obs[:,5]
    # air_speed = obs[:,6]
    # roll = obs[:,7]
    height = obs[:,0]
    ground_speed = obs[:,1]
    air_speed = obs[:,2]
    roll = obs[:,3]

    PE = mass*gravity*height 
    KE = 0.5*mass*(torch.abs(ground_speed)**2)  
    # reward_vinf = torch.squeeze(V_inf)/100.0
    # reward = reward_vinf
    # speed_reward = (ground_speed)**2 + (air_speed)**2 + (ground_speed-air_speed)**2
    speed_reward = (ground_speed)**2 + (air_speed)**2
    # thrust_reward = actions[:,3] - actions[:, 2] #Generate - Thrust
    # thrust_reward = -actions[:,2]
    vec_2_target = target_pos - base_pos[:,0:2]
    dist_2_target = torch.norm(vec_2_target, dim=1)
    unit_vec_2_target = vec_2_target/torch.unsqueeze(dist_2_target + 0.00001, dim=1).repeat(1,2)
    dist_reward = target_radius/(dist_2_target + 0.00001)
    
    # Don't go above 1 dist reward...
    reward_limit_idx = dist_reward>1.0
    dist_reward[reward_limit_idx] = 1.0
    

    punishment = torch.zeros((num_envs), device='cuda', dtype=torch.float)
    bank_punishment = roll**10

    diff_pos = glider_states[:,0:2] - glider_states_prev[:,0:2]
    target_reward  = torch.sum(diff_pos*unit_vec_2_target, dim=1)
    target_reward = torch.where(dist_2_target>target_radius, target_reward, 1.0)
    # reward = target_reward - bank_punishment

    glider_vel_uv = glider_states[:, 7:9]
    glider_vel = torch.norm(glider_vel_uv, dim=1)

    unit_heading_xy= glider_vel_uv/torch.unsqueeze(glider_vel + 0.00001, dim=1).repeat(1,2)

    vel_max = 30.0
    max_height = 10.0
    clipped_vel = torch.min(glider_vel,vel_max*torch.ones((num_envs), device='cuda', dtype=torch.float))
    clipped_height = torch.min(glider_states[:, 2],max_height*torch.ones((num_envs), device='cuda', dtype=torch.float))

    max_energy = (0.5*torch.pow(vel_max,2) + 9.8*max_height)
    energy_rew = (0.5*torch.pow(clipped_vel,2) + 9.8*clipped_height)/max_energy

    hdg_rew_1 = 0.5*(sum(unit_heading_xy*unit_vec_2_target,dim=1)+torch.ones((num_envs), device='cuda', dtype=torch.float)) # 0     -> 1
    hdg_rew_2 = 0.5*(sum(unit_heading_xy*unit_vec_2_target,dim=1)-torch.ones((num_envs), device='cuda', dtype=torch.float)) # -1    -> 0
    hdg_rew_3 = 0.5*(sum(unit_heading_xy*unit_vec_2_target,dim=1))                                                          # -0.5  -> 0.5
    hdg_rew_4 = (sum(unit_heading_xy*unit_vec_2_target,dim=1))                                                          # -1  -> 1
    hdg_rew_5 = 0.1*(sum(unit_heading_xy*unit_vec_2_target,dim=1))                                                          # -1  -> 1
    hdg_rew = hdg_rew_5
    # reward = hdg_rew + energy_rew
    reward = energy_rew*(torch.ones((num_envs), device='cuda', dtype=torch.float) + hdg_rew - energy_rew)
    battery_reward = obs[:,6]/100.0
    reward += battery_reward

    # reward = sum(unit_heading_xy*unit_vec_2_target,dim=1) + (0.5*torch.pow(clipped_vel,2) + 9.8*clipped_height)*0.01




    # reward = reward * 0.1
    # reward = dist_reward + (0.5*torch.pow(glider_vel,2) + 9.8*glider_states[:, 2])*0.00001




    # reward = PE + KE
    # reset agents if... 
    crash = glider_states[:,2] < 0.

    alpha_fail = torch.abs(alphas) > 0.3
    alpha_reset = torch.where(alpha_fail, torch.ones_like(alpha_fail), torch.zeros_like(alpha_fail))
    alpha_reset = torch.squeeze(torch.sum(alpha_reset, dim=0))>0

    
    # alpha_out_of_range = alpha > alpha_range
    # reset = reset | torch.any(torch.norm(contact_forces[:, knee_indices, :], dim=2) > 1., dim=1)
    time_out = episode_lengths >= max_episode_length - 1  # no terminal reward for time-outs
    
    # reward = torch.where(crash | alpha_reset, punishment, reward)
    reward = torch.where(crash, punishment, reward)
    # reset = crash | time_out | alpha_reset
    reset = crash | time_out

    goal_reached=torch.zeros_like(reset)
    goal_reached = torch.where(dist_2_target>target_radius, goal_reached, 1)

    return reward.detach(), reset, goal_reached


@torch.jit.script #Add Alpha Observation
def compute_glider_observations(glider_states,
                                commands,
                                gravity_vec,
                                actions,
                                alphas,
                                height,
                                ground_speed,
                                air_speed,
                                roll,
                                pitch,
                                yaw,
                                lin_vel_scale,
                                ang_vel_scale,
                                dof_pos_scale,
                                dof_vel_scale
                                ):

    # type: (Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, float, float, float, float) -> Tensor

    # base_pos = glider_states[:,0:3]
    # base_lin_vel = quat_rotate_inverse(base_quat, glider_states[:, 7:10]) * lin_vel_scale
    # base_ang_vel = quat_rotate_inverse(base_quat, glider_states[:, 10:13]) * ang_vel_scale
    # projected_gravity = quat_rotate(base_quat, gravity_vec)
    # commands_scaled = commands*torch.tensor([lin_vel_scale, lin_vel_scale, ang_vel_scale], requires_grad=False, device=commands.device)

    # obs = torch.cat((glider_states, #root state includes our base state... 
    #                  actions
    #                  ), dim=-1)


    

    obs = torch.cat((height,
                     ground_speed,
                     air_speed,
                     roll,
                     pitch,
                     yaw
                     ), dim=-1)


    return obs