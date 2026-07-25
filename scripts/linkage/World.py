import time

import numpy as np
import pygame
from pygame.event import Event

from linkage.Link import Link, Pivot, Constraint
from linkage.PhysicsObject import PhysicsObject


def check_intersection(startx, starty, endx2, endy2):
    dx = startx - endx2
    dy = starty - endy2
    # Increased snap radius to 20 pixels (20^2 = 400)
    return dx ** 2 + dy ** 2 < 400

class World:
    def __init__(self):
        pygame.init()
        self.__last_entity_add_time = time.time_ns()
        self.__window = pygame.display.set_mode((500, 400), pygame.DOUBLEBUF)
        self.__linkages: list[Link] = []
        self.__joints: list[Constraint] = []
        pass

    def start_loop(self) -> None:
        t1 = time.time_ns()
        while True:
            self.__window.fill((255, 255, 255))
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    break
                self.event_handler()
            self.__update()
            t2 = time.time_ns()
            self.__draw(t2 - t1)
            t1 = t2
            pygame.display.flip()

    def event_handler(self) -> None:
        keys = pygame.key.get_pressed()
        if keys[pygame.K_l]:
            if time.time_ns() - self.__last_entity_add_time > 1e9 * 0.5:
                link = Link()
                x, y = pygame.mouse.get_pos()
                link.set_position(x, y)
                self.add(link)
                self.__last_entity_add_time = time.time_ns()
        for obj in self.__linkages:
            obj.event_handler()
        pass

    def add(self, physics_object: Link) -> None:
        self.__linkages.append(physics_object)

    def __update(self) -> None:
        for obj in self.__linkages:
            obj.update()

        # Track pairs that already have a joint to prevent duplicates
        existing_joints = set()
        for joint in self.__joints:
            # Create a sorted tuple of the body references to identify the connection
            pair = tuple(sorted([id(joint.body_a), id(joint.body_b)]))
            existing_joints.add(pair)

        for i, link1 in enumerate(self.__linkages):
            for j in range(i + 1, len(self.__linkages)):
                link2 = self.__linkages[j]

                # Check if these two links are already connected
                pair_id = tuple(sorted([id(link1), id(link2)]))
                if pair_id in existing_joints:
                    continue

                startx, starty, endx, endy = link1.get_points()
                startx2, starty2, endx2, endy2 = link2.get_points()

                # Define Local Anchors
                # Start points are at (0,0) relative to the body's position
                # End points are at (length, 0) relative to the body's position
                local_start_1 = np.array([0.0, 0.0])
                local_end_1 = np.array([link1.length, 0.0])

                local_start_2 = np.array([0.0, 0.0])
                local_end_2 = np.array([link2.length, 0.0])

                # Pass the correct local anchors based on what intersected
                if check_intersection(startx, starty, endx2, endy2):
                    self.add_joint(link1, link2, local_start_1, local_end_2)
                elif check_intersection(startx, starty, startx2, starty2):
                    self.add_joint(link1, link2, local_start_1, local_start_2)
                elif check_intersection(endx, endy, endx2, endy2):
                    self.add_joint(link1, link2, local_end_1, local_end_2)
                elif check_intersection(endx, endy, startx2, starty2):
                    self.add_joint(link1, link2, local_end_1, local_start_2)
        iterations = 5
        for _ in range(iterations):
            for joint in self.__joints:
                error = joint.get_error()
                correction = error * 0.5

                if not getattr(joint.body_a, 'is_static', False):
                    joint.body_a.position -= correction
                if not getattr(joint.body_b, 'is_static', False):
                    joint.body_b.position += correction
    def __draw(self, dt) -> None:
        for obj in self.__linkages:
            obj.draw(dt, self.__window)
        for joint in self.__joints:
            joint.draw(self.__window)
        pass

    def add_joint(self, link1, link2, param, param1):
        self.__joints.append(Pivot(link1, link2, param, param1))
