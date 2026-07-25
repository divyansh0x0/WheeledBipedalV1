import time
import numpy as np
import pygame
from pygame.event import Event

# NEW: Import standard Python UI for the popup window
import tkinter as tk
from tkinter import simpledialog

from linkage.Link import Link, Pivot, Constraint, FixedPivot, MouseConstraint, CollisionConstraint
from linkage.PhysicsObject import PhysicsObject


class World:
    def __init__(self):
        pygame.init()

        # Initialize an invisible Tkinter root for popups
        self.tk_root = tk.Tk()
        self.tk_root.withdraw()

        self.__last_entity_add_time = time.time_ns()
        self.__window = pygame.display.set_mode((600, 500), pygame.DOUBLEBUF)
        self.__linkages: list[Link] = []
        self.__joints: list[Constraint] = []

        self.selected_nodes = []
        self.mouse_constraint = None

        print("=" * 40)
        print("⚙️ LINKAGE ENGINE CONTROLS ⚙️")
        print("L         : Spawn a new link (Prompts for length)")
        print("LEFT-CLICK: Drag a link endpoint to move it")
        print("RIGHT-CLICK: Select/Deselect endpoints or body (Max 2)")
        print("J         : Join two selected endpoints together (Pivot)")
        print("F         : Fix selected endpoint permanently to the wall")
        print("X / DEL   : Delete selected joints (if endpoint) or entire link (if body)")
        print("=" * 40)

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

        # Add Link (With Popup)
        if keys[pygame.K_l]:
            if time.time_ns() - self.__last_entity_add_time > 1e9 * 0.5:
                # Open the Tkinter popup (blocks Pygame until answered)
                length = simpledialog.askfloat("Linkage Length", "Enter linkage length:",
                                               initialvalue=100.0, minvalue=10.0, maxvalue=1000.0, parent=self.tk_root)

                # If they didn't hit cancel, create the link
                if length is not None:
                    link = Link(length=length)
                    x, y = pygame.mouse.get_pos()
                    link.set_position(x, y)
                    self.add(link)
                self.__last_entity_add_time = time.time_ns()

        # Handle Dragging
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            x, y = pygame.mouse.get_pos()
            for link in self.__linkages:
                node = link.get_node_at(x, y)
                if node is not None:
                    self.mouse_constraint = MouseConstraint(link, node)
                    break

        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.mouse_constraint = None

        # --- SELECTION LOGIC ---
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:  # RIGHT CLICK
            x, y = pygame.mouse.get_pos()
            found_selection = False
            for link in self.__linkages:
                node_ratio = link.get_node_at(x, y)
                if node_ratio is not None:
                    node_tuple = (link, node_ratio)

                    if node_tuple in self.selected_nodes:
                        self.selected_nodes.remove(node_tuple)
                        print(f"Deselected. ({len(self.selected_nodes)}/2 selected)")
                    else:
                        if len(self.selected_nodes) >= 2:
                            self.selected_nodes.pop(0)
                        self.selected_nodes.append(node_tuple)

                        target_name = "Body" if node_ratio == 'center' else "Endpoint"
                        print(f"SUCCESS: Selected {target_name}. ({len(self.selected_nodes)}/2 selected)")

                    found_selection = True
                    break

            if not found_selection:
                self.selected_nodes.clear()
                print("Cleared all selections.")

        # --- KEYBOARD COMMANDS (F, J, X) ---
        if event.type == pygame.KEYDOWN:

            # [J] JOIN LINKS
            if event.key == pygame.K_j:
                if len(self.selected_nodes) == 2:
                    link1, ratio1 = self.selected_nodes[0]
                    link2, ratio2 = self.selected_nodes[1]

                    if link1 == link2:
                        print("WARNING: Cannot join a link to itself!")
                    elif ratio1 == 'center' or ratio2 == 'center':
                        print("WARNING: You can only join endpoints, not the body of a link.")
                    else:
                        new_pivot = Pivot(link1, link2, ratio1, ratio2)
                        self.__joints.append(new_pivot)
                        print("SUCCESS: Joined two links with a Pivot!")
                        self.selected_nodes.clear()
                else:
                    print("WARNING: You must select exactly 2 endpoints to join them.")

            # [F] FIX TO WALL
            if event.key == pygame.K_f:
                if len(self.selected_nodes) > 0:
                    for selected_link, ratio in self.selected_nodes:
                        if ratio == 'center':
                            print("WARNING: Cannot fix the body of a link. Select an endpoint.")
                            continue

                        global_pos = selected_link.get_global_position(ratio)
                        new_fixed_pivot = FixedPivot(selected_link, ratio, global_pos)
                        self.__joints.append(new_fixed_pivot)
                        print(f"SUCCESS: Fixed Pivot permanently added at {global_pos}!")
                    self.selected_nodes.clear()
                else:
                    print("WARNING: Select at least one endpoint to fix it to the wall.")

            # [X] or [DELETE] DELETE LINK/JOINT
            if event.key in (pygame.K_x, pygame.K_DELETE, pygame.K_BACKSPACE):
                if not self.selected_nodes:
                    print("WARNING: Select a body or endpoint to delete it.")
                    return

                for selected_link, ratio in self.selected_nodes:
                    if ratio == 'center':
                        # Delete the entire linkage
                        if selected_link in self.__linkages:
                            self.__linkages.remove(selected_link)

                        # Find and delete ALL joints attached to this linkage
                        joints_to_delete = []
                        for joint in self.__joints:
                            if joint.body_a == selected_link or (
                                    hasattr(joint, 'body_b') and joint.body_b == selected_link):
                                joints_to_delete.append(joint)

                        for j in joints_to_delete:
                            self.__joints.remove(j)

                        print("SUCCESS: Deleted Linkage and its constraints.")

                    else:
                        # Delete ONLY the joints attached to this specific endpoint
                        joints_to_delete = []
                        for joint in self.__joints:
                            if joint.body_a == selected_link and joint.ratio_a == ratio:
                                joints_to_delete.append(joint)
                            elif hasattr(joint, 'body_b') and joint.body_b == selected_link and joint.ratio_b == ratio:
                                joints_to_delete.append(joint)

                        for j in joints_to_delete:
                            self.__joints.remove(j)

                        print(f"SUCCESS: Deleted constraints on endpoint {ratio}.")

                self.selected_nodes.clear()

    def add(self, physics_object: Link) -> None:
        self.__linkages.append(physics_object)

    def __update(self) -> None:
        for obj in self.__linkages:
            obj.update()

        existing_joints = set()
        for joint in self.__joints:
            if hasattr(joint, 'body_b'):
                pair = tuple(sorted([id(joint.body_a), id(joint.body_b)]))
                existing_joints.add(pair)

        active_collisions = []
        for i, link1 in enumerate(self.__linkages):
            for j in range(i + 1, len(self.__linkages)):
                link2 = self.__linkages[j]

                pair_id = tuple(sorted([id(link1), id(link2)]))
                if pair_id not in existing_joints:
                    active_collisions.append(CollisionConstraint(link1, link2, thickness=14.0))

        # --- SOLVER LOOP ---
        iterations = 30
        for _ in range(iterations):
            if self.mouse_constraint:
                self.mouse_constraint.solve()

            for joint in self.__joints:
                if isinstance(joint, Pivot):
                    joint.solve()

            for collision in active_collisions:
                collision.solve()

            for joint in self.__joints:
                if isinstance(joint, FixedPivot):
                    joint.solve()

    def __draw(self, dt) -> None:
        for obj in self.__linkages:
            obj.draw(dt, self.__window)

        for joint in self.__joints:
            joint.draw(self.__window)

        if self.mouse_constraint:
            self.mouse_constraint.draw(self.__window)

        # Draw the Selection Highlight for ALL selected nodes
        for selected_link, ratio in self.selected_nodes:
            pos = selected_link.get_global_position(ratio)
            px, py = int(pos[0]), int(pos[1])
            pygame.draw.circle(self.__window, (0, 255, 0), (px, py), 16, 4)