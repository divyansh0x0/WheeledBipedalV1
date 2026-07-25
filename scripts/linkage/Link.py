import math

import numpy as np
import pygame.draw

from linkage import PhysicsObject


class Link(PhysicsObject.PhysicsObject):
    def __init__(self, length):
        super().__init__()
        self.length = length
    def __getPoints(self):
        start_x, startY = self.position
        end_x = start_x + self.length * math.cos(self.angular_position * math.pi / 180)
        end_y = startY + self.length * math.sin(self.angular_position * math.pi / 180)
        return start_x, startY, end_x, end_y
    def update(self):
        super().update()
    def draw(self, dt, surface) -> None:
        start_x, start_y, end_x, end_y = self.__getPoints()
        pygame.draw.line(surface, (0,0,0), (start_x, start_y), (end_x, end_y), 2)

    def fixStart(self):
        self.velocity = np.array([0.0, 0.0])
    def fixEnd(self):
        self.velocity = np.array([0.0, 0.0])
        start_x, start_y, end_x, end_y = self.__getPoints()
        self.position = np.array([end_x, end_y])
        self.angular_position -= 180.0
