import math

import numpy as np
import pygame.draw
from pygame import mouse

from linkage.PhysicsObject import PhysicsObject


class Constraint:
    def get_error(self):
        pass

    def draw(self, __window):
        pass


class FixedPivot(Constraint):
    def __init__(self, body_a: PhysicsObject, anchor_ratio_a, fixed_anchor):
        self.body_a = body_a
        self.ratio_a = anchor_ratio_a
        self.fixed_anchor = fixed_anchor

    def get_global_anchors(self):
        theta_a = np.radians(self.body_a.angular_position)
        rot_a = np.array([[np.cos(theta_a), -np.sin(theta_a)], [np.sin(theta_a), np.cos(theta_a)]])
        local_a = np.array([self.body_a.length * self.ratio_a, 0.0])
        return self.body_a.position + rot_a @ local_a

    def get_error(self):
        return self.get_global_anchors() - self.fixed_anchor

    def solve(self):
        error = self.get_error()
        global_a = self.get_global_anchors()

        # Vector from body origin to the joint
        r_a = global_a - self.body_a.position

        # 1. Translate
        self.body_a.position -= error * 0.8

        # 2. Rotate (2D Cross Product: r X F)
        torque = r_a[0] * (-error[1]) - r_a[1] * (-error[0])
        self.body_a.angular_position += np.degrees(torque) * 0.005

    def draw(self, __window):
        pygame.draw.circle(__window, (0, 0, 255), self.get_global_anchors(), 10, 1)
        pygame.draw.circle(__window, (0, 0, 255), self.fixed_anchor, 10, 1)

class Pivot(Constraint):
    def __init__(self, body_a: PhysicsObject, body_b: PhysicsObject, anchor_ratio_a, anchor_ratio_b):
        self.body_a = body_a
        self.body_b = body_b
        self.ratio_a = anchor_ratio_a  # Store the ratio (0.0 to 1.0)
        self.ratio_b = anchor_ratio_b  # Store the ratio (0.0 to 1.0)

    def get_global_anchors(self):
        theta_a = np.radians(self.body_a.angular_position)
        theta_b = np.radians(self.body_b.angular_position)

        rot_a = np.array([[np.cos(theta_a), -np.sin(theta_a)],
                          [np.sin(theta_a), np.cos(theta_a)]])
        rot_b = np.array([[np.cos(theta_b), -np.sin(theta_b)],
                          [np.sin(theta_b), np.cos(theta_b)]])

        # Dynamically calculate local anchors based on current lengths
        local_a = np.array([self.body_a.length * self.ratio_a, 0.0])
        local_b = np.array([self.body_b.length * self.ratio_b, 0.0])

        global_a = self.body_a.position + rot_a @ local_a
        global_b = self.body_b.position + rot_b @ local_b

        return global_a, global_b

    def get_error(self):
        global_a, global_b = self.get_global_anchors()
        return global_a - global_b

    def draw(self, __window):
        global_a, global_b = self.get_global_anchors()
        pygame.draw.circle(__window, (0, 0, 255), global_a, 10, 1)
        pygame.draw.circle(__window, (0, 0, 255), global_b, 10, 1)

    def solve(self):
        error = self.get_error()
        global_a, global_b = self.get_global_anchors()

        r_a = global_a - self.body_a.position
        r_b = global_b - self.body_b.position
        correction = error * 0.5
        rot_weight = 0.005  # Tuning constant for rotation stability

        # Fix Body A
        self.body_a.position -= correction * 0.5
        torque_a = r_a[0] * (-correction[1]) - r_a[1] * (-correction[0])
        self.body_a.angular_position += np.degrees(torque_a) * rot_weight

        # Fix Body B
        self.body_b.position += correction * 0.5
        torque_b = r_b[0] * (correction[1]) - r_b[1] * (correction[0])
        self.body_b.angular_position += np.degrees(torque_b) * rot_weight


class MouseConstraint(Constraint):
    def __init__(self, body_a: PhysicsObject, anchor_ratio_a):
        self.body_a = body_a
        self.ratio_a = anchor_ratio_a

    def get_global_anchor(self):
        theta_a = np.radians(self.body_a.angular_position)
        rot_a = np.array([[np.cos(theta_a), -np.sin(theta_a)], [np.sin(theta_a), np.cos(theta_a)]])
        local_a = np.array([self.body_a.length * self.ratio_a, 0.0])
        return self.body_a.position + rot_a @ local_a

    def solve(self):
        global_a = self.get_global_anchor()
        mouse_pos = np.array(pygame.mouse.get_pos(), dtype=float)

        error = global_a - mouse_pos
        r_a = global_a - self.body_a.position

        self.body_a.position -= error * 0.5
        torque_a = r_a[0] * (-error[1]) - r_a[1] * (-error[0])
        self.body_a.angular_position += np.degrees(torque_a) * 0.005

    def draw(self, __window):
        pygame.draw.line(__window, (255, 0, 0), self.get_global_anchor(), pygame.mouse.get_pos(), 2)
class Link(PhysicsObject):
    def __init__(self, length=100, thickness=3):
        super().__init__()
        self.__drag_offset = np.array([0, 0])
        # Replace the boolean with a state string to track exactly what is being dragged
        self.length = length
        self.thickness = thickness
        self.__end_point_radius = 10

    def get_points(self):
        start_x, start_y = self.position
        end_x = start_x + self.length * math.cos(self.angular_position * math.pi / 180)
        end_y = start_y + self.length * math.sin(self.angular_position * math.pi / 180)
        return start_x, start_y, end_x, end_y

    def draw(self, dt, surface) -> None:
        start_x, start_y, end_x, end_y = self.get_points()
        line_color = (0, 0, 0)

        pygame.draw.line(surface, line_color, (start_x, start_y), (end_x, end_y), self.thickness)
        pygame.draw.circle(surface, line_color, (start_x, start_y), self.__end_point_radius)
        pygame.draw.circle(surface, line_color, (end_x, end_y), self.__end_point_radius)

    def is_intersecting_start(self, x, y):
        start_x, start_y, _, _ = self.get_points()
        mouse = np.array([x, y])
        start = np.array([start_x, start_y])
        return np.linalg.norm(mouse - start) <= self.__end_point_radius

    def is_intersecting_end(self, x, y):
        _, _, end_x, end_y = self.get_points()
        mouse = np.array([x, y])
        end = np.array([end_x, end_y])
        return np.linalg.norm(mouse - end) <= self.__end_point_radius

    def is_intersecting_center(self, x, y):
        start_x, start_y, end_x, end_y = self.get_points()

        start = np.array([start_x, start_y])
        end = np.array([end_x, end_y])
        mouse = np.array([x, y])

        # Guard Clause: Check if the mouse is inside the corner circles
        if self.is_intersecting_start(x, y) or self.is_intersecting_end(x, y):
            return False

        line_vec = end - start
        mouse_vec = mouse - start
        line_len = self.length

        if line_len == 0:
            return False

        line_unit = line_vec / line_len
        projection_length = np.dot(mouse_vec, line_unit)

        if 0 <= projection_length <= line_len:
            closest_point = start + projection_length * line_unit
            distance = np.linalg.norm(mouse - closest_point)
            return distance < 5

        return False
# Returns 0.0 (start), 1.0 (end), or None based on mouse position
    def get_node_at(self, x, y):
        if self.is_intersecting_start(x, y):
            return 0.0
        if self.is_intersecting_end(x, y):
            return 1.0
        return None

    # Returns the global (x, y) NumPy array of the requested ratio
    def get_global_position(self, ratio):
        start_x, start_y, end_x, end_y = self.get_points()
        if ratio == 0.0:
            return np.array([start_x, start_y])
        elif ratio == 1.0:
            return np.array([end_x, end_y])
        return None