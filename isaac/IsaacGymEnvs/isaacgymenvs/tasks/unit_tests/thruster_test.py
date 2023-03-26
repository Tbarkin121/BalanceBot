import sys
# Figure something about with this rediculous pathing later... 
# why is including modules from lower directory levels so fucking annoying? 
sys.path.append("../")
# sys.path.append("tasks/")
from glider_components.thruster_component import Thruster
from glider_components.thruster_manager import Thruster_Manager
import torch

root_state_dimensions = 17
num_envs = 3
num_actions = 3
num_obs =  12
glider_states = torch.zeros((num_envs, root_state_dimensions), device='cuda')
battery_states = glider_states.view(num_envs, root_state_dimensions)[:,13:15] #Battery Capacity(1), Power Usage(1)
thruster_states = glider_states.view(num_envs, root_state_dimensions)[:,15:17] #Thruster Force(1), Power Usage(1)
actions = torch.rand((num_envs, num_actions), device='cuda')*2-1
actions[0,:] = 1.0
actions[1,:] = 0.0
actions[2,:] = -1.0
obs_raw = torch.rand((num_envs, num_obs), device='cuda')
dt = 0.1
# 2 Cell battery is nominally 7.4V 
# Assuming 260 mAh of capacity
# We have about 7000 J of energy
thuster_1 = Thruster(0.05,                                              # Mass
                torch.tensor([0.0, 0.0, 0.0, 1.0], device='cuda'),  # Orientaiton
                torch.tensor([0.0, 0.0, 0.0], device='cuda'),       # Translation
                1,                                               # Max Thrust
                0.95)                                                # Efficency
                                                    
thrust_list = [thuster_1]
print(thrust_list)

thrust_man = Thruster_Manager('cuda',
                            [15,16],
                            thrust_list)

battery_states[:,0] = 100  # Initial Energy
battery_states[:,1] = -0.1 # Static Power Burn

for _ in range(3):
    dYdt = thrust_man.physics_step(glider_states, actions, obs_raw)
    print(thruster_states)

