from .base_manager import Manager
from .battery_component import Battery
import torch

class Battery_Manager(Manager):
    def __init__(self, device, state_cols, battery_list):
        super().__init__(device, state_cols)
        print('Battery Manager Init')

        self.max_energy = 0
        self.battery_count = 0
        self.inital_energy = 0
        self.avg_charge_eff = 0
        self.avg_discharge_eff = 0
        for battery in battery_list:
            # Check if the item is a battery, exit if not
            battery_check = isinstance(battery, Battery)
            if(not battery_check):
                print('There was something weird in the battery list')
                exit()

            self.battery_count += 1
            self.max_energy += battery.max_energy
            self.inital_energy += battery.initial_energy
            self.avg_charge_eff += battery.charge_eff
            self.avg_discharge_eff += battery.discharge_eff

        self.avg_charge_eff = self.avg_charge_eff / self.battery_count
        self.avg_discharge_eff = self.avg_discharge_eff / self.battery_count
        self.static_power = -0.01 # Static power draw from systems like microcontrollers, sensors, ect

    def reset_battery_states(self, env_ids, states):
        states[env_ids, 0] = self.inital_energy



    def physics_step(self, states, actions, observations):
        battery_states = states[:,self.state_cols]
        states[:,self.state_cols[0]] = torch.clamp(states[:,self.state_cols[0]], 0.0, self.max_energy)
        power = battery_states[:,1].reshape(len(battery_states[:,1]),1)
        eff_mult = torch.where(power>0, self.avg_charge_eff, self.avg_discharge_eff)
        dYdt = torch.cat( (power*eff_mult, torch.zeros_like(power)), dim=1)
        return dYdt

