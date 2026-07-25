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

    def solve(self):
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

    def solve(self):
        global_a = self.get_global_anchors()

        # Vector pointing FROM the anchor TO the fixed wall point
        error = self.fixed_anchor - global_a
        r_a = global_a - self.body_a.position

        # 1. Translate: Apply 100% of the correction (1.0) because the wall is infinitely heavy
        self.body_a.position += error

        # 2. Rotate: Cross Product (r x F) without np.degrees inflation
        torque = r_a[0] * error[1] - r_a[1] * error[0]
        self.body_a.angular_position += torque * 0.01

    def draw(self, __window):
        pygame.draw.circle(__window, (0, 0, 255), self.get_global_anchors(), 10, 1)
        pygame.draw.circle(__window, (0, 0, 255), self.fixed_anchor, 10, 1)


class Pivot(Constraint):
    def __init__(self, body_a: PhysicsObject, body_b: PhysicsObject, anchor_ratio_a, anchor_ratio_b):
        self.body_a = body_a
        self.body_b = body_b
        self.ratio_a = anchor_ratio_a
        self.ratio_b = anchor_ratio_b

    def get_global_anchors(self):
        theta_a = np.radians(self.body_a.angular_position)
        theta_b = np.radians(self.body_b.angular_position)

        rot_a = np.array([[np.cos(theta_a), -np.sin(theta_a)], [np.sin(theta_a), np.cos(theta_a)]])
        rot_b = np.array([[np.cos(theta_b), -np.sin(theta_b)], [np.sin(theta_b), np.cos(theta_b)]])

        local_a = np.array([self.body_a.length * self.ratio_a, 0.0])
        local_b = np.array([self.body_b.length * self.ratio_b, 0.0])

        global_a = self.body_a.position + rot_a @ local_a
        global_b = self.body_b.position + rot_b @ local_b
        return global_a, global_b

    def solve(self):
        global_a, global_b = self.get_global_anchors()

        # Vector pointing FROM A TO B
        error = global_b - global_a

        r_a = global_a - self.body_a.position
        r_b = global_b - self.body_b.position

        # Split the correction equally (assumes equal mass)
        correction_a = error * 0.5
        correction_b = -error * 0.5

        # Fix Body A
        self.body_a.position += correction_a
        torque_a = r_a[0] * correction_a[1] - r_a[1] * correction_a[0]
        self.body_a.angular_position += torque_a * 0.01

        # Fix Body B
        self.body_b.position += correction_b
        torque_b = r_b[0] * correction_b[1] - r_b[1] * correction_b[0]
        self.body_b.angular_position += torque_b * 0.01

    def draw(self, __window):
        global_a, global_b = self.get_global_anchors()
        pygame.draw.circle(__window, (0, 0, 255), global_a, 10, 1)
        pygame.draw.circle(__window, (0, 0, 255), global_b, 10, 1)


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

        # Vector pointing FROM anchor TO mouse
        error = mouse_pos - global_a
        r_a = global_a - self.body_a.position

        # 0.1 makes the mouse act like a stretchy rubber band so it doesn't break the solver
        correction = error * 0.1

        self.body_a.position += correction
        torque = r_a[0] * correction[1] - r_a[1] * correction[0]
        self.body_a.angular_position += torque * 0.01

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