import time
import json
import numpy as np
import pygame
from pygame.event import Event
import tkinter as tk
from tkinter import simpledialog

from linkage.Link import Link, Pivot, Constraint, FixedPivot, MouseConstraint, CollisionConstraint, ServoConstraint, \
    CollinearConstraint
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

        # Double-Click Tracking
        self.double_click_time = 300
        self.last_click_time = 0
        self.last_clicked_link = None
        self.link_to_edit = None

        print("=" * 40)
        print("⚙️ LINKAGE ENGINE CONTROLS ⚙️")
        print("L         : Spawn a new link")
        print("L-CLICK   : Drag object (Double-click to edit length!)")
        print("R-CLICK   : Select/Deselect endpoints or body")
        print("J         : Join 2 selected endpoints together (Pivot)")
        print("C         : Make 2 connected linkages completely Collinear")
        print("F         : Fix selected endpoint to the wall")
        print("M         : Add Motor (Select 1 endpoint + bodies to spin)")
        print("X / DEL   : Delete selected object")
        print("S         : Quick Save Structure")
        print("O         : Quick Load Save")
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
                self.tk_root.update()

                if length is not None:
                    link = Link(length=length)
                    x, y = pygame.mouse.get_pos()
                    link.set_position(x, y)
                    self.__linkages.append(link)
                self.__last_entity_add_time = time.time_ns()

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and not slider_interacted:
            x, y = pygame.mouse.get_pos()
            clicked_link = None
            clicked_node = None

            for link in self.__linkages:
                node = link.get_node_at(x, y)
                if node is not None:
                    clicked_link = link
                    clicked_node = node
                    break

            if clicked_link is not None:
                current_time = pygame.time.get_ticks()

                if clicked_link == self.last_clicked_link and (
                        current_time - self.last_click_time) < self.double_click_time:
                    self.mouse_constraint = None
                    self.link_to_edit = clicked_link
                    self.last_click_time = 0
                    self.last_clicked_link = None
                else:
                    self.last_click_time = current_time
                    self.last_clicked_link = clicked_link
                    self.mouse_constraint = MouseConstraint(clicked_link, clicked_node)

        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.mouse_constraint = None

            if self.link_to_edit is not None:
                new_len = simpledialog.askfloat("Edit Length", "Enter new linkage length:",
                                                initialvalue=self.link_to_edit.length, minvalue=10.0, maxvalue=1000.0,
                                                parent=self.tk_root)
                self.tk_root.update()

                if new_len is not None:
                    self.link_to_edit.length = new_len
                    print(f"SUCCESS: Length updated to {new_len}")

                self.link_to_edit = None

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

            # --- QUICK SAVE SYSTEM ---
            if event.key == pygame.K_s:
                export_data = {"linkages": [], "joints": []}
                link_map = {link: i for i, link in enumerate(self.__linkages)}

                for i, link in enumerate(self.__linkages):
                    export_data["linkages"].append({
                        "length": link.length,
                        "position": [link.position[0], link.position[1]],
                        "angle": link.angular_position
                    })

                for j in self.__joints:
                    if isinstance(j, Pivot):
                        export_data["joints"].append({
                            "type": "Pivot",
                            "link_a": link_map.get(j.body_a, -1),
                            "ratio_a": j.ratio_a,
                            "link_b": link_map.get(j.body_b, -1),
                            "ratio_b": j.ratio_b
                        })
                    elif isinstance(j, FixedPivot):
                        export_data["joints"].append({
                            "type": "FixedPivot",
                            "link_a": link_map.get(j.body_a, -1),
                            "ratio_a": j.ratio_a,
                            "fixed_anchor": [j.fixed_anchor[0], j.fixed_anchor[1]]
                        })
                    elif isinstance(j, CollinearConstraint):
                        export_data["joints"].append({
                            "type": "CollinearConstraint",
                            "link_a": link_map.get(j.body_a, -1),
                            "link_b": link_map.get(j.body_b, -1)
                        })
                    elif isinstance(j, ServoConstraint):
                        export_data["joints"].append({
                            "type": "ServoConstraint",
                            "rotors": [link_map.get(r, -1) for r in j.rotors if link_map.get(r, -1) != -1],
                            "stators": [link_map.get(s, -1) for s in j.stators if link_map.get(s, -1) != -1],
                            "base_ratio": j.base_ratio,
                            "target_angle": j.target_angle
                        })

                with open("quick_save.json", "w") as f:
                    json.dump(export_data, f, indent=4)
                print("SUCCESS: Quick Save completed!")

            # --- QUICK LOAD SYSTEM ---
            if event.key == pygame.K_o:
                try:
                    with open("quick_save.json", "r") as f:
                        import_data = json.load(f)

                    # Wipe the current slate clean
                    self.__linkages.clear()
                    self.__joints.clear()
                    self.selected_nodes.clear()
                    self.sliders.clear()
                    self.mouse_constraint = None

                    # Reconstruct Linkages
                    for l_data in import_data.get("linkages", []):
                        link = Link(length=l_data["length"])
                        link.position = np.array(l_data["position"], dtype=float)
                        link.angular_position = float(l_data["angle"])
                        self.__linkages.append(link)

                    # Reconstruct Joints & Constraints
                    for j_data in import_data.get("joints", []):
                        j_type = j_data.get("type")
                        if j_type == "Pivot":
                            l_a = self.__linkages[j_data["link_a"]]
                            l_b = self.__linkages[j_data["link_b"]]
                            self.__joints.append(Pivot(l_a, l_b, j_data["ratio_a"], j_data["ratio_b"]))

                        elif j_type == "FixedPivot":
                            l_a = self.__linkages[j_data["link_a"]]
                            self.__joints.append(FixedPivot(l_a, j_data["ratio_a"], j_data["fixed_anchor"]))

                        elif j_type == "CollinearConstraint":
                            l_a = self.__linkages[j_data["link_a"]]
                            l_b = self.__linkages[j_data["link_b"]]
                            self.__joints.append(CollinearConstraint(l_a, l_b))

                        elif j_type == "ServoConstraint":
                            rotors = [self.__linkages[r] for r in j_data["rotors"]]
                            stators = [self.__linkages[s] for s in j_data["stators"]]
                            new_servo = ServoConstraint(rotors, stators, j_data["base_ratio"])
                            new_servo.target_angle = j_data["target_angle"]
                            self.__joints.append(new_servo)

                            # Recreate UI Slider
                            slider = SimpleSlider(new_servo)
                            slider.val = j_data["target_angle"]
                            self.sliders.append(slider)

                    print("SUCCESS: Quick Load completed!")
                except FileNotFoundError:
                    print("WARNING: No quick save file found (quick_save.json).")
                except Exception as e:
                    print(f"WARNING: Failed to load save file: {e}")

            if event.key == pygame.K_p:  # Keep generic debug JSON for diagnostics
                pass

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

            if event.key == pygame.K_c:
                body_nodes = [n for n in self.selected_nodes if n[1] == 'center']
                if len(body_nodes) == 2:
                    link1, _ = body_nodes[0]
                    link2, _ = body_nodes[1]

                    if link1 != link2:
                        are_connected = False
                        for j in self.__joints:
                            if isinstance(j, Pivot):
                                if (j.body_a == link1 and j.body_b == link2) or (
                                        j.body_a == link2 and j.body_b == link1):
                                    are_connected = True
                                    break

                        if are_connected:
                            self.__joints.append(CollinearConstraint(link1, link2))
                            print("SUCCESS: Links made permanently collinear!")
                            self.selected_nodes.clear()
                        else:
                            print(
                                "WARNING: Links must be connected by a Pivot joint first (Select endpoints & press J).")
                else:
                    print("WARNING: Select exactly 2 link bodies (centers) to make them collinear.")

            if event.key == pygame.K_f:
                for selected_link, ratio in self.selected_nodes:
                    if ratio != 'center':
                        global_pos = selected_link.get_global_position(ratio)
                        self.__joints.append(FixedPivot(selected_link, ratio, global_pos))
                        print("SUCCESS: Fixed pivot added.")
                self.selected_nodes.clear()

            if event.key == pygame.K_m:
                pivot_nodes = [n for n in self.selected_nodes if n[1] != 'center']
                rotor_nodes = [n for n in self.selected_nodes if n[1] == 'center']

                if not pivot_nodes:
                    print("WARNING: Select the Joint (Endpoint) to place the motor on.")
                    return

                base_pivot_link, base_pivot_ratio = pivot_nodes[0]
                pivot_pos = base_pivot_link.get_global_position(base_pivot_ratio)

                for link, ratio in pivot_nodes:
                    if np.linalg.norm(link.get_global_position(ratio) - pivot_pos) > 25:
                        print(
                            "WARNING: You selected multiple endpoints that are not touching. Clear selections and try again.")
                        return

                if not rotor_nodes:
                    print("WARNING: Select the bodies (centers) of the linkages you want to spin.")
                    return

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

                stators = []
                for link in self.__linkages:
                    if link in rotors: continue
                    if np.linalg.norm(link.get_global_position(0.0) - pivot_pos) <= 25 or \
                            np.linalg.norm(link.get_global_position(1.0) - pivot_pos) <= 25:
                        stators.append(link)

                base_rotor = rotors[0]
                dist0 = np.linalg.norm(base_rotor.get_global_position(0.0) - pivot_pos)
                dist1 = np.linalg.norm(base_rotor.get_global_position(1.0) - pivot_pos)
                actual_base_ratio = 0.0 if dist0 < dist1 else 1.0

                new_servo = ServoConstraint(rotors, stators, actual_base_ratio)
                self.__joints.append(new_servo)
                self.sliders.append(SimpleSlider(new_servo))

                print(f"SUCCESS: Motor Added! Spinning Links: {len(rotors)} | Base Links: {len(stators)}")
                self.selected_nodes.clear()

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

            for joint in self.__joints:
                if isinstance(joint, CollinearConstraint): joint.solve()

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