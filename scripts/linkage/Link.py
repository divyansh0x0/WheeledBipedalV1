import math

import numpy as np
import pygame.draw
from pygame import mouse

from linkage.PhysicsObject import PhysicsObject


class Constraint:
    def get_error(self): pass

    def draw(self, __window): pass


def get_closest_points(p1, q1, p2, q2):
    # Calculates the shortest distance between two finite line segments
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = np.dot(d1, d1)
    e = np.dot(d2, d2)
    f = np.dot(d2, r)
    c = np.dot(d1, r)
    b = np.dot(d1, d2)
    denom = a * e - b * b

    if denom != 0.0:
        s = np.clip((b * f - c * e) / denom, 0.0, 1.0)
    else:
        s = 0.0

    t = (b * s + f) / e if e != 0.0 else 0.0

    if t < 0.0:
        t = 0.0
        s = np.clip(-c / a, 0.0, 1.0) if a != 0.0 else 0.0
    elif t > 1.0:
        t = 1.0
        s = np.clip((b - c) / a, 0.0, 1.0) if a != 0.0 else 0.0

    c1 = p1 + s * d1
    c2 = p2 + t * d2
    return c1, c2


class CollisionConstraint(Constraint):
    def __init__(self, body_a: PhysicsObject, body_b: PhysicsObject, thickness: float):
        self.body_a = body_a
        self.body_b = body_b
        # How far apart the centers of the lines need to be to not touch
        self.thickness = thickness

    def solve(self):
        # 1. Get endpoints for both links
        start_a, start_y_a, end_x_a, end_y_a = self.body_a.get_points()
        start_b, start_y_b, end_x_b, end_y_b = self.body_b.get_points()

        p1 = np.array([start_a, start_y_a])
        q1 = np.array([end_x_a, end_y_a])
        p2 = np.array([start_b, start_y_b])
        q2 = np.array([end_x_b, end_y_b])

        # 2. Find the absolute closest points between the two links
        c1, c2 = get_closest_points(p1, q1, p2, q2)

        # 3. Check distance
        dist_vec = c1 - c2
        dist = np.linalg.norm(dist_vec)

        if dist < self.thickness:
            # They are colliding! Calculate push-out normal
            if dist < 1e-5:
                # Edge Case: Perfectly overlapping lines. Push orthogonally.
                line_vec = q1 - p1
                normal = np.array([-line_vec[1], line_vec[0]])
                n_len = np.linalg.norm(normal)
                if n_len > 0: normal = normal / n_len
            else:
                normal = dist_vec / dist

            penetration = self.thickness - dist

            # 50/50 split to push bodies apart
            correction = normal * (penetration * 0.5)

            r_a = c1 - self.body_a.position
            r_b = c2 - self.body_b.position

            # Apply Translation & Torque to Body A
            self.body_a.position += correction
            torque_a = r_a[0] * correction[1] - r_a[1] * correction[0]
            self.body_a.angular_position += torque_a * 0.05

            # Apply Translation & Torque to Body B
            self.body_b.position -= correction
            torque_b = r_b[0] * (-correction[1]) - r_b[1] * (-correction[0])
            self.body_b.angular_position += torque_b * 0.05
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
        error = self.fixed_anchor - global_a
        r_a = global_a - self.body_a.position

        # 1. Rotate FIRST
        torque = r_a[0] * error[1] - r_a[1] * error[0]
        self.body_a.angular_position += torque * 0.01

        # 2. Recalculate where the anchor ended up after the twist
        global_a_new = self.get_global_anchors()

        # 3. Translate LAST (100% rigid snap to the immovable wall)
        error_new = self.fixed_anchor - global_a_new
        self.body_a.position += error_new

    def draw(self, __window):
        global_a = self.get_global_anchors()

        # Safely convert NumPy float arrays to integer tuples for Pygame
        ax, ay = int(global_a[0]), int(global_a[1])
        fx, fy = int(self.fixed_anchor[0]), int(self.fixed_anchor[1])

        # Draw a thick RED circle to represent an immovable Fixed Pivot
        pygame.draw.circle(__window, (255, 0, 0), (fx, fy), 14, 4)

        # Draw a smaller red dot for the anchor currently on the body
        pygame.draw.circle(__window, (255, 100, 100), (ax, ay), 6)

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
        return self.body_a.position + rot_a @ local_a, self.body_b.position + rot_b @ local_b

    def solve(self):
        global_a, global_b = self.get_global_anchors()
        error = global_b - global_a
        r_a = global_a - self.body_a.position
        r_b = global_b - self.body_b.position

        correction_a = error * 0.5
        correction_b = -error * 0.5

        # 1. Rotate FIRST
        torque_a = r_a[0] * correction_a[1] - r_a[1] * correction_a[0]
        self.body_a.angular_position += torque_a * 0.01

        torque_b = r_b[0] * correction_b[1] - r_b[1] * correction_b[0]
        self.body_b.angular_position += torque_b * 0.01

        # 2. Recalculate
        global_a_new, global_b_new = self.get_global_anchors()
        error_new = global_b_new - global_a_new

        # 3. Translate LAST
        self.body_a.position += error_new * 0.5
        self.body_b.position -= error_new * 0.5

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
        error = mouse_pos - global_a
        r_a = global_a - self.body_a.position
        correction = error * 0.1

        # 1. Rotate FIRST
        torque = r_a[0] * correction[1] - r_a[1] * correction[0]
        self.body_a.angular_position += torque * 0.01

        # 2. Recalculate
        global_a_new = self.get_global_anchor()
        error_new = mouse_pos - global_a_new

        # 3. Translate LAST
        self.body_a.position += error_new * 0.1

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
        # Increased hitbox from 10 to 20 for easy clicking
        return np.linalg.norm(mouse - start) <= 20

    def is_intersecting_end(self, x, y):
        _, _, end_x, end_y = self.get_points()
        mouse = np.array([x, y])
        end = np.array([end_x, end_y])
        # Increased hitbox from 10 to 20 for easy clicking
        return np.linalg.norm(mouse - end) <= 20
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
            return distance < 10

        return False


    def get_node_at(self, x, y):
        if self.is_intersecting_start(x, y):
            return 0.0
        if self.is_intersecting_end(x, y):
            return 1.0
        if self.is_intersecting_center(x, y):
            return 'center'
        return None


    # Returns the global (x, y) NumPy array of the requested ratio
    def get_global_position(self, ratio):
        start_x, start_y, end_x, end_y = self.get_points()
        if ratio == 0.0:
            return np.array([start_x, start_y])
        elif ratio == 1.0:
            return np.array([end_x, end_y])
        elif ratio == 'center':
            return np.array([(start_x + end_x) / 2, (start_y + end_y) / 2])
        return None