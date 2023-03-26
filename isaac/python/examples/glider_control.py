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

# initialize gym
gym = gymapi.acquire_gym()

# parse arguments
args = gymutil.parse_arguments(description="Joint control Methods Example")
print('ARGGGGgggg')
print(args)
# create a simulator
sim_params = gymapi.SimParams()
sim_params.substeps = 2
sim_params.dt = 1.0 / 100.0


sim_params.physx.solver_type = 1
sim_params.physx.num_position_iterations = 4
sim_params.physx.num_velocity_iterations = 1

sim_params.physx.num_threads = 4
sim_params.physx.use_gpu = args.use_gpu

sim_params.use_gpu_pipeline = False
print('!!!')
print(sim_params)
# sim_params.physx.contact_offset=0.02
# sim_params.physx.rest_offset=0.0
# sim_params.physx.bounce_threshold_velocity=0.2
# sim_params.physx.max_depenetration_velocity=100.0
# sim_params.physx.default_buffer_size_multiplier=5.0
# sim_params.physx.max_gpu_contact_pairs=8388608 # 8*1024*1024
# sim_params.physx.num_subscenes=4
# sim_params.physx.contact_collection=1 # 0: CC_NEVER (don't collect contact info), 1: CC_LAST_SUBSTEP (collect only contacts on last substep), 2: CC_ALL_SUBSTEPS (default - all contacts)

if args.use_gpu_pipeline:
    print("WARNING: Forcing CPU pipeline.")

sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, 0.0)
sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)

if sim is None:
    print("*** Failed to create sim")
    quit()



# add ground plane
plane_params = gymapi.PlaneParams()
# set the normal force to be z dimension
plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
gym.add_ground(sim, plane_params)

# set up the env grid
num_envs = 4
spacing = 1.5
env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
env_upper = gymapi.Vec3(spacing, 0.0, spacing)

# add cartpole urdf asset
# asset_root = "../../assets"
# asset_file = "urdf/WalkBot/urdf/WalkBot.urdf"
# asset_file = "urdf/WalkBot_3DOF_330/urdf/WalkBot_3DOF.urdf"
asset_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../assets')
glider_asset_file = "urdf/Glider_S/urdf/Glider_S.urdf"
glider_ui_asset_file = "urdf/Glider_S_UI/urdf/Glider_S_UI.urdf"

# Load asset with default control type of position for all joints
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True
asset_options.angular_damping = 0.0
asset_options.max_angular_velocity = 10000
asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
print("Loading asset '%s' from '%s'" % (glider_ui_asset_file, asset_root))
glider_asset  = gym.load_asset(sim, asset_root, glider_asset_file, asset_options)
glider_ui_asset = gym.load_asset(sim, asset_root, glider_ui_asset_file, asset_options)

# initial root pose for cartpole actors
initial_pose = gymapi.Transform()
initial_pose.p = gymapi.Vec3(0.0, 0.0, 1.0)
initial_pose.r = gymapi.Quat(0, 0.0, 0.0, 1.0)

# Create environment 0
# Cart held steady using position target mode.
# Pole held at a 45 degree angle using position target mode.


env0 = gym.create_env(sim, env_lower, env_upper, 2)
glider = gym.create_actor(env0, glider_asset, initial_pose, 'CubeBot', 0, 0)
glider_ui = gym.create_actor(env0, glider_ui_asset, initial_pose, 'CubeBot', 0, 0)
num_bodies = gym.get_actor_rigid_body_count(env0, glider_ui)

# Configure DOF properties
props = gym.get_actor_dof_properties(env0, glider_ui)


gym.set_actor_dof_properties(env0, glider_ui, props)
# Set DOF drive targets
dof_dict = gym.get_actor_dof_dict(env0, glider_ui)
joint_dict = gym.get_actor_joint_dict(env0, glider_ui)
dof_keys = list(dof_dict.keys())
actor_root_state = gym.acquire_actor_root_state_tensor(sim)
root_states = gymtorch.wrap_tensor(actor_root_state)
actor_dof_state = gym.acquire_dof_state_tensor(sim)
dof_states = gymtorch.wrap_tensor(actor_dof_state)
# create viewer using the default camera properties
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    raise ValueError('*** Failed to create viewer')

# Look at the first env
cam_pos = gymapi.Vec3(2, 1, 1)
cam_target = initial_pose.p
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

# Simulate
F_or_B = False
dial_angle = 0
diff_angle = 0.01
diff_pos = -0.001
while not gym.query_viewer_has_closed(viewer):
    gym.refresh_actor_root_state_tensor(sim)
    gym.refresh_dof_state_tensor(sim)
    # print(root_states)
    # print(dof_states)
    

    if(F_or_B):
        dial_angle += diff_angle
        if(dial_angle > 5):
            F_or_B = not F_or_B
    else:
        dial_angle -= diff_angle
        if(dial_angle < 0):
            F_or_B = not F_or_B

    dof_states[0,0] = dial_angle
    dof_states[1,0] = dial_angle
    dof_states[2,0] = dial_angle

    gym.set_dof_state_tensor(sim, gymtorch.unwrap_tensor(dof_states))

    root_states[0,0] += diff_pos
    root_states[1,0] += diff_pos*0.5
    gym.set_actor_root_state_tensor(sim, gymtorch.unwrap_tensor(root_states))
        
    # step the physics
    # gym.simulate(sim)
    # gym.fetch_results(sim, True)

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)
    
 
    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)

print('Done')

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
