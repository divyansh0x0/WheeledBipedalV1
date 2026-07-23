import numpy as np


class PhysicsObject:
    def __init__(self):
        self.position = np.array([0, 0])
        self.orientation = np.array([0, 0])
        self.velocity = np.array([0, 0])
        self.angular_velocity = np.array([0, 0])
        self.angular_acceleration = np.array([0, 0])
        self.linear_acceleration = np.array([0, 0])
        self.force = np.array([0, 0])

    def update(self):
        pass

    def draw(self):
        pass
