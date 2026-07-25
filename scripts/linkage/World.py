import time
import numpy as np
import pygame
from pygame.event import Event
import tkinter as tk
from tkinter import simpledialog

from linkage.Link import Link, Pivot, Constraint, FixedPivot, MouseConstraint, CollisionConstraint, ServoConstraint
from linkage.PhysicsObject import PhysicsObject


class SimpleSlider:
    def __init__(self, servo_ref):
        self.servo = servo_ref
        self.width = 200
        self.height = 20
        self.min_v = -180
        self.max_v = 180
        self.val = 0.0
        self.dragging = False
        self.rect = pygame.Rect(10, 10, self.width, self.height)

    def draw(self, surface, index):
        self.rect.y = 30 + (index * 60)
        pygame.draw.rect(surface, (200, 200, 200), self.rect, border_radius=10)
        ratio = (self.val - self.min_v) / (self.max_v - self.min_v)
        kx = self.rect.x + ratio * self.rect.width
        pygame.draw.circle(surface, (255, 140, 0), (int(kx), self.rect.centery), 12)

        font = pygame.font.SysFont(None, 24)
        img = font.render(f"Motor {index + 1}: {int(self.val)} deg", True, (0, 0, 0))
        surface.blit(img, (self.rect.x, self.rect.y - 20))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.dragging = True
                return True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False

        if self.dragging and event.type in (pygame.MOUSEMOTION, pygame.MOUSEBUTTONDOWN):
            mx = max(self.rect.left, min(event.pos[0], self.rect.right))
            ratio = (mx - self.rect.left) / self.rect.width
            self.val = self.min_v + ratio * (self.max_v - self.min_v)
            return True
        return False


class World:
    def __init__(self):
        pygame.init()
        self.tk_root = tk.Tk()
        self.tk_root.withdraw()

        self.__last_entity_add_time = time.time_ns()
        self.__window = pygame.display.set_mode((800, 600), pygame.DOUBLEBUF)
        self.__linkages: list[Link] = []
        self.__joints: list[Constraint] = []
        self.selected_nodes = []
        self.mouse_constraint = None
        self.sliders: list[SimpleSlider] = []

        print("=" * 40)
        print("⚙️ LINKAGE ENGINE CONTROLS ⚙️")
        print("L         : Spawn a new link")
        print("RIGHT-CLICK: Select/Deselect endpoints or body")
        print("J         : Join 2 selected endpoints together (Pivot)")
        print("F         : Fix selected endpoint to the wall")
        print("M         : Add Motor (Select 1 endpoint + bodies to spin)")
        print("X / DEL   : Delete selected object and attached constraints")
        print("=" * 40)

    def add(self, physics_object: Link):
        w, h = self.__window.get_size()
        physics_object.position = np.array([w / 2, h / 2])
        self.__linkages.append(physics_object)

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

        slider_interacted = False
        for slider in self.sliders:
            if slider.handle_event(event):
                slider_interacted = True

        if keys[pygame.K_l]:
            if time.time_ns() - self.__last_entity_add_time > 1e9 * 0.5:
                length = simpledialog.askfloat("Linkage Length", "Enter linkage length:",
                                               initialvalue=100.0, minvalue=10.0, maxvalue=1000.0, parent=self.tk_root)
                if length is not None:
                    link = Link(length=length)
                    x, y = pygame.mouse.get_pos()
                    link.set_position(x, y)
                    self.__linkages.append(link)
                self.__last_entity_add_time = time.time_ns()

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and not slider_interacted:
            x, y = pygame.mouse.get_pos()
            for link in self.__linkages:
                node = link.get_node_at(x, y)
                if node is not None:
                    self.mouse_constraint = MouseConstraint(link, node)
                    break

        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.mouse_constraint = None

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
            x, y = pygame.mouse.get_pos()
            found_selection = False
            for link in self.__linkages:
                node_ratio = link.get_node_at(x, y)
                if node_ratio is not None:
                    node_tuple = (link, node_ratio)
                    if node_tuple in self.selected_nodes:
                        self.selected_nodes.remove(node_tuple)
                    else:
                        self.selected_nodes.append(node_tuple)
                    found_selection = True
                    break

            if not found_selection:
                self.selected_nodes.clear()

        if event.type == pygame.KEYDOWN:

            # --- JOIN LOGIC ---
            if event.key == pygame.K_j:
                endpoints = [n for n in self.selected_nodes if n[1] != 'center']
                if len(endpoints) == 2:
                    link1, ratio1 = endpoints[0]
                    link2, ratio2 = endpoints[1]
                    if link1 != link2:
                        self.__joints.append(Pivot(link1, link2, ratio1, ratio2))
                        self.selected_nodes.clear()
                        print("SUCCESS: Joined two endpoints!")
                else:
                    print("WARNING: Select exactly 2 endpoints to join.")

            # --- FIXED WALL LOGIC ---
            if event.key == pygame.K_f:
                for selected_link, ratio in self.selected_nodes:
                    if ratio != 'center':
                        global_pos = selected_link.get_global_position(ratio)
                        self.__joints.append(FixedPivot(selected_link, ratio, global_pos))
                        print("SUCCESS: Fixed pivot added.")
                self.selected_nodes.clear()

            # --- NEW MOTOR LOGIC (Separate Joint from Bodies) ---
            if event.key == pygame.K_m:
                # 1. Sort selections into Joint coordinates vs Link Bodies
                pivot_nodes = [n for n in self.selected_nodes if n[1] != 'center']
                rotor_nodes = [n for n in self.selected_nodes if n[1] == 'center']

                # 2. Check if the user specified WHERE to put the motor
                if not pivot_nodes:
                    print("WARNING: Select the Joint (Endpoint) to place the motor on.")
                    return

                # Define the pivot point
                base_pivot_link, base_pivot_ratio = pivot_nodes[0]
                pivot_pos = base_pivot_link.get_global_position(base_pivot_ratio)

                # Ensure all selected endpoints are at the same physical coordinate
                for link, ratio in pivot_nodes:
                    if np.linalg.norm(link.get_global_position(ratio) - pivot_pos) > 25:
                        print(
                            "WARNING: You selected multiple endpoints that are not touching. Clear selections and try again.")
                        return

                # 3. Check if the user specified WHAT should spin
                if not rotor_nodes:
                    print("WARNING: Select the bodies (centers) of the linkages you want to spin.")
                    return

                # 4. Extract the spinning linkages (Rotors)
                rotors = []
                for link, _ in rotor_nodes:
                    if np.linalg.norm(link.get_global_position(0.0) - pivot_pos) <= 25 or \
                            np.linalg.norm(link.get_global_position(1.0) - pivot_pos) <= 25:
                        rotors.append(link)
                    else:
                        print("INFO: Ignored a link body because it is not connected to the selected joint.")

                if not rotors:
                    self.selected_nodes.clear()
                    return

                # 5. Extract the base linkages (Stators)
                stators = []
                for link in self.__linkages:
                    if link in rotors: continue
                    if np.linalg.norm(link.get_global_position(0.0) - pivot_pos) <= 25 or \
                            np.linalg.norm(link.get_global_position(1.0) - pivot_pos) <= 25:
                        stators.append(link)

                # Find the math ratio of the first rotor for the constraint to track
                base_rotor = rotors[0]
                dist0 = np.linalg.norm(base_rotor.get_global_position(0.0) - pivot_pos)
                dist1 = np.linalg.norm(base_rotor.get_global_position(1.0) - pivot_pos)
                actual_base_ratio = 0.0 if dist0 < dist1 else 1.0

                # Append the motor
                new_servo = ServoConstraint(rotors, stators, actual_base_ratio)
                self.__joints.append(new_servo)
                self.sliders.append(SimpleSlider(new_servo))

                print(f"SUCCESS: Motor Added! Spinning Links: {len(rotors)} | Base Links: {len(stators)}")
                self.selected_nodes.clear()

            # --- SAFELY DELETE MOTORS AND JOINTS ---
            if event.key in (pygame.K_x, pygame.K_DELETE, pygame.K_BACKSPACE):
                for selected_link, ratio in self.selected_nodes:
                    if ratio == 'center':
                        if selected_link in self.__linkages:
                            self.__linkages.remove(selected_link)

                        self.__joints = [j for j in self.__joints if not (
                                (hasattr(j, 'body_a') and j.body_a == selected_link) or
                                (hasattr(j, 'body_b') and j.body_b == selected_link) or
                                (hasattr(j, 'rotors') and selected_link in j.rotors) or
                                (hasattr(j, 'stators') and selected_link in j.stators)
                        )]
                    else:
                        joints_to_delete = []
                        for j in self.__joints:
                            if hasattr(j, 'ratio_a') and getattr(j, 'body_a') == selected_link and getattr(j,
                                                                                                           'ratio_a') == ratio:
                                joints_to_delete.append(j)
                            elif hasattr(j, 'ratio_b') and getattr(j, 'body_b') == selected_link and getattr(j,
                                                                                                             'ratio_b') == ratio:
                                joints_to_delete.append(j)
                            elif isinstance(j, ServoConstraint):
                                servo_anchor = j.get_global_anchor()
                                endpoint_pos = selected_link.get_global_position(ratio)
                                if np.linalg.norm(servo_anchor - endpoint_pos) < 5:
                                    joints_to_delete.append(j)

                        for j in joints_to_delete:
                            if j in self.__joints:
                                self.__joints.remove(j)

                self.selected_nodes.clear()
                self.sliders = [s for s in self.sliders if s.servo in self.__joints]

    def __update(self) -> None:
        for obj in self.__linkages:
            obj.update()

        parent = {}

        def get_root(node):
            if parent[node] == node: return node
            parent[node] = get_root(parent[node])
            return parent[node]

        for joint in self.__joints:
            if isinstance(joint, Pivot):
                node_a = (id(joint.body_a), joint.ratio_a)
                node_b = (id(joint.body_b), joint.ratio_b)
                if node_a not in parent: parent[node_a] = node_a
                if node_b not in parent: parent[node_b] = node_b
                root_a = get_root(node_a)
                root_b = get_root(node_b)
                if root_a != root_b:
                    parent[root_a] = root_b

        ignore_collisions = set()
        clusters = {}
        for node in parent:
            root = get_root(node)
            if root not in clusters: clusters[root] = set()
            clusters[root].add(node[0])

        for link_ids in clusters.values():
            id_list = list(link_ids)
            for i in range(len(id_list)):
                for j in range(i + 1, len(id_list)):
                    ignore_collisions.add(tuple(sorted([id_list[i], id_list[j]])))

        active_collisions = []
        for i, link1 in enumerate(self.__linkages):
            for j in range(i + 1, len(self.__linkages)):
                link2 = self.__linkages[j]
                pair_id = tuple(sorted([id(link1), id(link2)]))
                if pair_id not in ignore_collisions:
                    active_collisions.append(CollisionConstraint(link1, link2, thickness=14.0))

        iterations = 30
        for _ in range(iterations):
            if self.mouse_constraint: self.mouse_constraint.solve()
            for joint in self.__joints:
                if isinstance(joint, Pivot): joint.solve()

            for slider in self.sliders:
                slider.servo.set_target_angle(slider.val)
                slider.servo.solve()

            for collision in active_collisions: collision.solve()
            for joint in self.__joints:
                if isinstance(joint, FixedPivot): joint.solve()

    def __draw(self, dt) -> None:
        for obj in self.__linkages:
            obj.draw(dt, self.__window)
        for joint in self.__joints:
            joint.draw(self.__window)
        if self.mouse_constraint:
            self.mouse_constraint.draw(self.__window)

        for selected_link, ratio in self.selected_nodes:
            pos = selected_link.get_global_position(ratio)
            px, py = int(pos[0]), int(pos[1])
            pygame.draw.circle(self.__window, (0, 255, 0), (px, py), 16, 4)

        for i, slider in enumerate(self.sliders):
            slider.draw(self.__window, i)