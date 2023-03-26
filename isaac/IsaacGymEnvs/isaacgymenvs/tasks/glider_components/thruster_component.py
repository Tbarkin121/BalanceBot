from .base_component import Component
import torch

class Thruster(Component):
    def __init__(self, mass, ori, pos, max_thrust, efficency):
        print('Thruster Init')
        # Set Standard Parameters
        super().__init__(mass, ori, pos)
        # Set Additional Paramaters
        self.max_forward_thrust = max_thrust
        self.max_reverse_thrust = max_thrust
        self.efficency = efficency
        self.prop_radius = 0.05
        self.prop_area = torch.tensor(torch.pi*self.prop_radius**2, device='cuda')
        self.rho = 1.2
        self.Cd0 = 100
        self.P0 = 0.1


    def get_force_and_power(self, action, wind_vel):
        # Cd = self.Cd0*action
        # force = 0.5*self.rho*self.prop_area*Cd*wind_vel**2
        # power = self.efficency*(force*wind_vel+self.P0)

        force = action*self.max_forward_thrust
        power = -force*1000.0 
        return force, power
        


