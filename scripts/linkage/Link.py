import pygame.draw

from linkage import PhysicsObject


class Link(PhysicsObject.PhysicsObject):
    def __init__(self, length):
        super().__init__()
        self.length = length

    def update(self):
        super().update()
    def draw(self, dt, surface) -> None:

        pygame.draw.rect(surface, (0,0,0), [10,10,10,10])
