import time

import numpy as np
import pygame
from pygame.event import Event

from linkage.Link import Link, Pivot, Constraint, FixedPivot, MouseConstraint
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
        self.selected_node = None

        self.mouse_constraint = None  # Track active drag
        self.selected_node = None  # Track right-click selection

    def start_loop(self) -> None:
        t1 = time.time_ns()
        while True:
            self.__window.fill((255, 255, 255))
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    break
                self.event_handler(event)
            self.__update()
            t2 = time.time_ns()
            self.__draw(t2 - t1)
            t1 = t2
            pygame.display.flip()

    def event_handler(self, event) -> None:
        keys = pygame.key.get_pressed()
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            x, y = pygame.mouse.get_pos()
            for link in self.__linkages:
                node = link.get_node_at(x, y)
                if node is not None:
                    # Create a temporary joint attached to the mouse
                    self.mouse_constraint = MouseConstraint(link, node)
                    break

        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.mouse_constraint = None  # Drop the link
        if keys[pygame.K_l]:
            if time.time_ns() - self.__last_entity_add_time > 1e9 * 0.5:
                link = Link()
                x, y = pygame.mouse.get_pos()
                link.set_position(x, y)
                self.add(link)
                self.__last_entity_add_time = time.time_ns()
        for obj in self.__linkages:
            obj.event_handler()
            # --- NEW SELECTION LOGIC ---

            # 1. Handle Right-Click Selection
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:  # button 3 is Right-Click
                x, y = pygame.mouse.get_pos()

                # Check all links to see if we clicked an endpoint
                found_selection = False
                for link in self.__linkages:
                    node_ratio = link.get_node_at(x, y)
                    if node_ratio is not None:
                        self.selected_node = (link, node_ratio)
                        found_selection = True
                        break

                # If we clicked empty space, deselect
                if not found_selection:
                    self.selected_node = None

            # 2. Handle Applying Properties to the Selected Node
            if event.type == pygame.KEYDOWN:
                if self.selected_node is not None:
                    selected_link, ratio = self.selected_node

                    # Press 'F' to apply a FixedPivot
                    if event.key == pygame.K_f:
                        global_pos = selected_link.get_global_position(ratio)
                        new_fixed_pivot = FixedPivot(selected_link, ratio, global_pos)
                        self.__joints.append(new_fixed_pivot)
                        print(f"Fixed Pivot added at ratio {ratio}")

                    # Press 'M' to apply a Motor (placeholder for your Motor class)
                    if event.key == pygame.K_m:
                        # self.__joints.append(MotorConstraint(selected_link, target_rpm=60))
                        print(f"Motor applied to link")

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
                # Pass the correct local anchors based on what intersected
                if check_intersection(startx, starty, endx2, endy2):
                    self.add_joint(link1, link2, 0.0, 1.0)
                elif check_intersection(startx, starty, startx2, starty2):
                    self.add_joint(link1, link2, 0.0, 0.0)
                elif check_intersection(endx, endy, endx2, endy2):
                    self.add_joint(link1, link2, 1.0, 1.0)
                elif check_intersection(endx, endy, startx2, starty2):
                    self.add_joint(link1, link2, 1.0, 0.0)
                # --- NEW SOLVER LOOP ---
            iterations = 5
            for _ in range(iterations):
                # Solve structural joints
                for joint in self.__joints:
                    joint.solve()
                # Solve mouse drag joint
                if self.mouse_constraint:
                    self.mouse_constraint.solve()

    def __draw(self, dt) -> None:
        for obj in self.__linkages:
            obj.draw(dt, self.__window)
        for joint in self.__joints:
            joint.draw(self.__window)

        if self.selected_node is not None:
            selected_link, ratio = self.selected_node
            pos = selected_link.get_global_position(ratio)
            # Draw a green circle (radius 14, thickness 3) around the selected point
            pygame.draw.circle(self.__window, (0, 255, 0), pos, 14, 3)


    def add_joint(self, link1, link2, param, param1):
        self.__joints.append(Pivot(link1, link2, param, param1))
