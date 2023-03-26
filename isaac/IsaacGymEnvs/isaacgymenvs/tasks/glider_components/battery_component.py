from .base_component import Component
import torch

class Battery(Component):
    def __init__(self, mass, ori, pos, max_energy, charge_percent, charge_eff, discharge_eff):
        print('Battery Init')
        # Set Standard Parameters
        super().__init__(mass, ori, pos)
        # Set Additional Paramaters
        self.max_energy = max_energy
        self.initial_energy = self.max_energy * charge_percent
        self.charge_eff = charge_eff
        self.discharge_eff = discharge_eff

