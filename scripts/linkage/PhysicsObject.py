from typing import Any

import numpy as np
from numpy import dtype, ndarray


class PhysicsObject:
    orientation: ndarray[tuple[Any, ...], dtype[Any]]
    position: ndarray[tuple[Any, ...], dtype[Any]]

    def __init__(self):
        self.position = np.array([0.0, 00.0])
        self.orientation = np.array([0.0, 0.0])
        self.velocity = np.array([0.0, 0.0])
        self.linear_acceleration = np.array([0.0, 0.0])
        self.angular_position = 0
        self.angular_velocity =0
        self.angular_acceleration = 0.0
        self.dt = 1/120
        self.length : float = 0.0
        self.mass = 1.0

    def update(self):
        self.position += self.velocity * self.dt
        self.velocity += self.linear_acceleration * self.dt
        self.angular_position += self.angular_velocity * self.dt
        self.angular_velocity += self.angular_acceleration * self.dt
        pass

    def set_position(self, x,y):
        self.position[0] = x
        self.position[1] = y
    def draw(self, dt, surface) -> None:

        pass

    def event_handler(self):
        pass
