import sys
# Figure something about with this rediculous pathing later... 
# why is including modules from lower directory levels so fucking annoying? 
sys.path.append("../")
# sys.path.append("tasks/")
from glider_components.battery_component import Battery
from glider_components.battery_manager import Battery_Manager
import torch

root_state_dimensions = 15
num_envs = 2
num_actions = 3
num_obs =  12
glider_states = torch.zeros((num_envs, root_state_dimensions), device='cuda')
battery_states = glider_states.view(num_envs, root_state_dimensions)[:,13:] #Battery Capacity(1), PowerUsage(1)
actions = torch.rand((num_envs, num_actions))*2-1
obs_raw = torch.rand((num_envs, num_obs))
dt = 0.1
# 2 Cell battery is nominally 7.4V 
# Assuming 260 mAh of capacity
# We have about 7000 J of energy
batt_1 = Battery(0.05,                                              # Mass
                torch.tensor([0.0, 0.0, 0.0, 1.0], device='cuda'),  # Orientaiton
                torch.tensor([0.0, 0.0, 0.0], device='cuda'),       # Translation
                7000,                                               # Max Energy
                0.5,                                                # Initial Charge Percent
                0.8,                                                # Charge Eff (For every J we try to generate, we store 0.8J)
                1.05)                                               # Discharge Eff (For every J we try to draw, we draw 1.05J)
batt_2 = Battery(0.05,                                            
                torch.tensor([0.0, 0.0, 0.0, 1.0], device='cuda'),  
                torch.tensor([0.0, 0.0, 0.0], device='cuda'),       
                7000,                                               
                0.5,                                                
                0.8,                                                
                1.05)      


batt_list = [batt_1, batt_2]
print(batt_list)

batt_man = Battery_Manager('cuda',
                            [13,14],
                            batt_list)

battery_states[:,0] = batt_man.inital_energy
# battery_states[:,1] = torch.rand((num_envs,))*200-100
battery_states[0,1] = 100
battery_states[1,1] = -100

for _ in range(3):
    dYdt = batt_man.physics_step(glider_states, actions, obs_raw)
    battery_states += dYdt*dt
    print(glider_states)

