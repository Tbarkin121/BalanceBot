import abc
from abc import ABC
import torch

class Manager:
    def __init__(self, device, state_cols):
        self.device=device
        self.state_cols = state_cols #The columns of the glider_state tensor the component is associated with
        self.components = []
        print('Component Init')
    
    def add_component(self, component):
        """Add a component to self.components"""
        self.components.add(component)

    @abc.abstractmethod 
    def physics_step(self, states, actions, observations):
        """Returns dY/dt"""