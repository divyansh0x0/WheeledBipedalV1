import numpy as np


class PhysicsObject:
    def __init__(self):
        self.position = np.array([0.0, 0.0])
        self.orientation = np.array([0.0, 0.0])
        self.velocity = np.array([0.0, 0.0])
        self.linear_acceleration = np.array([0.0, 0.0])
        self.angular_position = 0.0
        self.angular_velocity = 0.0
        self.angular_acceleration = 0.0
        self.dt = 1000/120
        self.mass = 1.0

    def update(self):
        self.position += self.velocity * self.dt
        self.velocity += self.linear_acceleration * self.dt
        self.angular_position += self.angular_velocity * self.dt
        self.angular_velocity += self.angular_acceleration * self.dt
        pass

    def draw(self, dt, surface) -> None:

        pass
