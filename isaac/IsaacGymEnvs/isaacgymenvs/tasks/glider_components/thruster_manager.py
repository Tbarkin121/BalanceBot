from .base_manager import Manager
from .thruster_component import Thruster
import torch

class Thruster_Manager(Manager):
    def __init__(self, device, state_cols, thruster_list):
        super().__init__(device, state_cols)
        print('Thruster Manager Init')
        self.thruster_list = thruster_list
        self.thruster_count = len(thruster_list)
        for thruster in self.thruster_list:
            # Check if the item is a thruster, exit if not
            thruster_check = isinstance(thruster, Thruster)
            if(not thruster_check):
                print('There was something weird in the thruster list')
                exit()
        self.battery_low_cuttoff = 1000.0

        
    def physics_step(self, states, actions, obs_raw):
        wind_vel = obs_raw[:,2]
        force = 0
        power = 0
        # I know this can be tensorized... just trying to get everything working first
        for thruster in self.thruster_list:
            force_, power_ = thruster.get_force_and_power(actions[:,2], wind_vel)
            force += force_
            power += power_
        # Where battery Capacity is above the low cuttoff, we can apply force. After going below the cutoff we lose the ability to thrust
        states[:,self.state_cols[0]] = torch.where(torch.logical_and(states[:,13]<self.battery_low_cuttoff, force[:]>0.0), torch.zeros_like(force, device=self.device), force)
        states[:,self.state_cols[1]] = torch.where(torch.logical_and(states[:,13]<self.battery_low_cuttoff, force[:]>0.0), torch.zeros_like(power, device=self.device), power)
        dYdt = torch.cat( (torch.zeros_like(torch.unsqueeze(force,dim=1)), torch.zeros_like(torch.unsqueeze(power,dim=1))), dim=1)
        return dYdt

