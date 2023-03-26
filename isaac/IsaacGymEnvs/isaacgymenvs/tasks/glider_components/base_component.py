import abc
from abc import ABC

class Component:
    def __init__(self, mass, ori, pos):
        print('Component Init')
        self.shape = "Box"
        self.mass = mass
        self.orientation = ori
        self.translation = pos
    
