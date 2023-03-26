import pygame
import numpy as np

class Joystick:
    def __init__(self):
        pygame.joystick.quit()
        pygame.quit()
        pygame.display.init()
        pygame.joystick.init()
        joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        self.joystick = joysticks[0]
        self.num_axis = self.joystick.get_numaxes() 
        self.num_buttons = self.joystick.get_numbuttons()
        self.num_hats = self.joystick.get_numhats()
        self.joystick.rumble(1, 1, 1)
        self.zero_vals = np.zeros(self.num_axis)
        self.zero()

    def zero(self):
        pygame.event.pump()
        for i in range(self.num_axis):
            self.zero_vals[i] = self.joystick.get_axis(i)

    def get_axis(self):
        pygame.event.pump()
        a = np.zeros(self.num_axis)
        for i in range(self.num_axis):
            a[i] = self.joystick.get_axis(i) - self.zero_vals[i]
        return a

    def get_button(self):
        pygame.event.pump()
        b = np.zeros(self.num_buttons)
        for i in range(self.num_buttons):
            b[i] = self.joystick.get_button(i)
        return b

    def get_dpad(self):
        pygame.event.pump()
        x,y = self.joystick.get_hat(0)
        d = [x,y]
        return d