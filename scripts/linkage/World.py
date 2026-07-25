import time

import pygame

from linkage.PhysicsObject import PhysicsObject


class World:
    def __init__(self):
        pygame.init()
        self.__window = pygame.display.set_mode((500, 400), pygame.DOUBLEBUF)
        self.__physics_objects: list[PhysicsObject] = []
        pass

    def start_loop(self) -> None:
        t1 = time.time_ns()
        while True:
            self.__window.fill((255, 255, 255))
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    break
            self.__update()
            t2 = time.time_ns()
            self.__draw(t2 - t1)
            t1 = t2
            pygame.display.flip()
    def add(self, physics_object: PhysicsObject) -> None:
        self.__physics_objects.append(physics_object)
    def __update(self) -> None:
        for obj in self.__physics_objects:
            obj.update()
        pass

    def __draw(self, dt) -> None:
        for obj in self.__physics_objects:
            obj.draw(dt, self.__window)
        pass
