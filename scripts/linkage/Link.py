import math
import numpy as np
import pygame.draw
from pygame import mouse
from linkage.PhysicsObject import PhysicsObject


def get_closest_points(p1, q1, p2, q2):
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


class Constraint:
    def get_error(self): pass

    def draw(self, __window): pass

    def solve(self): pass


class CollinearConstraint(Constraint):
    def __init__(self, body_a: PhysicsObject, body_b: PhysicsObject):
        self.body_a = body_a
        self.body_b = body_b

        diff = (self.body_b.angular_position - self.body_a.angular_position) % 360
        self.target_diff = 180 if 90 < diff < 270 else 0

        error = (diff - self.target_diff + 180) % 360 - 180
        self.body_a.angular_position += error * 0.5
        self.body_b.angular_position -= error * 0.5

    def solve(self):
        diff = (self.body_b.angular_position - self.body_a.angular_position) % 360
        error = (diff - self.target_diff + 180) % 360 - 180

        step = np.clip(error * 0.8, -15.0, 15.0)
        self.body_a.angular_position += step * 0.5
        self.body_b.angular_position -= step * 0.5

    def draw(self, __window):
        c1 = self.body_a.get_global_position('center')
        c2 = self.body_b.get_global_position('center')
        px1, py1 = int(c1[0]), int(c1[1])
        px2, py2 = int(c2[0]), int(c2[1])
        pygame.draw.line(__window, (138, 43, 226), (px1, py1), (px2, py2), 4)


class FixedPivot(Constraint):
    def __init__(self, body_a: PhysicsObject, anchor_ratio_a, fixed_anchor):
        self.body_a = body_a
        self.ratio_a = anchor_ratio_a
        self.fixed_anchor = np.array(fixed_anchor, dtype=float)

    def get_global_anchors(self):
        theta_a = np.radians(self.body_a.angular_position)
        rot_a = np.array([[np.cos(theta_a), -np.sin(theta_a)], [np.sin(theta_a), np.cos(theta_a)]])
        local_a = np.array([self.body_a.length * self.ratio_a, 0.0])
        return self.body_a.position + rot_a @ local_a

    def solve(self):
        global_a = self.get_global_anchors()
        error = self.fixed_anchor - global_a

        self.body_a.position += error

        r_a = global_a - self.body_a.position
        r_len_sq = np.dot(r_a, r_a)

        if r_len_sq > 1e-4:
            cross_prod = r_a[0] * error[1] - r_a[1] * error[0]
            d_theta_deg = np.degrees(cross_prod / r_len_sq)

            d_theta_deg = np.clip(d_theta_deg, -15.0, 15.0)
            self.body_a.angular_position += d_theta_deg * 0.8

    def draw(self, __window):
        global_a = self.get_global_anchors()
        ax, ay = int(global_a[0]), int(global_a[1])
        fx, fy = int(self.fixed_anchor[0]), int(self.fixed_anchor[1])
        pygame.draw.circle(__window, (255, 0, 0), (fx, fy), 14, 4)
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

        self.body_a.position += error * 0.5
        self.body_b.position -= error * 0.5

        r_a = global_a - self.body_a.position
        r_b = global_b - self.body_b.position

        r_a_sq = np.dot(r_a, r_a)
        r_b_sq = np.dot(r_b, r_b)

        if r_a_sq > 1e-4:
            cross_a = r_a[0] * error[1] - r_a[1] * error[0]
            d_theta_a = np.clip(np.degrees(cross_a / r_a_sq), -15.0, 15.0)
            self.body_a.angular_position += d_theta_a * 0.6

        if r_b_sq > 1e-4:
            cross_b = r_b[0] * (-error[1]) - r_b[1] * (-error[0])
            d_theta_b = np.clip(np.degrees(cross_b / r_b_sq), -15.0, 15.0)
            self.body_b.angular_position += d_theta_b * 0.6

    def draw(self, __window):
        global_a, global_b = self.get_global_anchors()
        ax, ay = int(global_a[0]), int(global_a[1])
        bx, by = int(global_b[0]), int(global_b[1])
        pygame.draw.circle(__window, (0, 0, 255), (ax, ay), 12, 3)
        pygame.draw.circle(__window, (0, 0, 255), (bx, by), 12, 3)


class MouseConstraint(Constraint):
    def __init__(self, body_a: PhysicsObject, anchor_ratio_a):
        self.body_a = body_a
        self.ratio_a = anchor_ratio_a

    def get_global_anchor(self):
        theta_a = np.radians(self.body_a.angular_position)
        rot_a = np.array([[np.cos(theta_a), -np.sin(theta_a)], [np.sin(theta_a), np.cos(theta_a)]])
        numeric_ratio = 0.5 if self.ratio_a == 'center' else self.ratio_a
        local_a = np.array([self.body_a.length * numeric_ratio, 0.0])
        return self.body_a.position + rot_a @ local_a

    def solve(self):
        global_a = self.get_global_anchor()
        mouse_pos = np.array(pygame.mouse.get_pos(), dtype=float)
        error = mouse_pos - global_a

        self.body_a.position += error * 0.02

        r_a = global_a - self.body_a.position
        r_len_sq = np.dot(r_a, r_a)
        if r_len_sq > 1e-4:
            cross_prod = r_a[0] * error[1] - r_a[1] * error[0]
            d_theta = np.clip(np.degrees(cross_prod / r_len_sq), -10.0, 10.0)
            self.body_a.angular_position += d_theta * 0.01

    def draw(self, __window):
        global_a = self.get_global_anchor()
        ax, ay = int(global_a[0]), int(global_a[1])
        mx, my = pygame.mouse.get_pos()
        pygame.draw.line(__window, (255, 0, 0), (ax, ay), (mx, my), 3)


class CollisionConstraint(Constraint):
    def __init__(self, body_a: PhysicsObject, body_b: PhysicsObject, thickness: float):
        self.body_a = body_a
        self.body_b = body_b
        self.thickness = thickness

    def solve(self):
        start_a, start_y_a, end_x_a, end_y_a = self.body_a.get_points()
        start_b, start_y_b, end_x_b, end_y_b = self.body_b.get_points()

        p1, q1 = np.array([start_a, start_y_a]), np.array([end_x_a, end_y_a])
        p2, q2 = np.array([start_b, start_y_b]), np.array([end_x_b, end_y_b])

        c1, c2 = get_closest_points(p1, q1, p2, q2)
        dist_vec = c1 - c2
        dist = np.linalg.norm(dist_vec)

        if dist < self.thickness:
            if dist < 1e-5:
                line_vec = q1 - p1
                normal = np.array([-line_vec[1], line_vec[0]])
                n_len = np.linalg.norm(normal)
                if n_len > 0: normal = normal / n_len
            else:
                normal = dist_vec / dist

            penetration = self.thickness - dist
            correction = normal * (penetration * 0.5)

            self.body_a.position += correction
            self.body_b.position -= correction


class ServoConstraint(Constraint):
    def __init__(self, rotors: list, stators: list, base_ratio: float):
        self.rotors = rotors
        self.stators = stators
        self.base_ratio = base_ratio

        # Enable flag dictates if it hunts for the target angle
        self.enabled = True

        self.base_rotor = rotors[0]
        self.rotor_offsets = [self._norm(r.angular_position - self.base_rotor.angular_position) for r in rotors]

        if stators:
            self.base_stator = stators[0]
            self.initial_angle_diff = self._norm(self.base_rotor.angular_position - self.base_stator.angular_position)
        else:
            self.base_stator = None
            self.initial_angle_diff = self._norm(self.base_rotor.angular_position)

        self.target_angle = 0.0

    def _norm(self, ang):
        return (ang + 180) % 360 - 180

    def set_target_angle(self, angle):
        self.target_angle = angle

    def get_global_anchor(self):
        theta = np.radians(self.base_rotor.angular_position)
        rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        local = np.array([self.base_rotor.length * self.base_ratio, 0.0])
        return self.base_rotor.position + rot @ local

    def solve(self):
        # 1. ALWAYS lock rotors together (keep the rigid body intact)
        for i in range(1, len(self.rotors)):
            rotor = self.rotors[i]
            current_diff = self._norm(rotor.angular_position - self.base_rotor.angular_position)
            error = self._norm(self.rotor_offsets[i] - current_diff)
            rotor.angular_position += error * 0.5
            self.base_rotor.angular_position -= error * 0.5

        # 2. Skip servo torque if disabled (acts like a free pivot)
        if not self.enabled:
            return

        # 3. Apply Servo Torque
        if self.base_stator:
            current_diff = self._norm(self.base_rotor.angular_position - self.base_stator.angular_position)
            desired_diff = self._norm(self.initial_angle_diff + self.target_angle)
            error = self._norm(desired_diff - current_diff)

            step = np.clip(error * 0.5, -10.0, 10.0)
            self.base_rotor.angular_position += step * 0.5
            self.base_stator.angular_position -= step * 0.5
        else:
            current_ang = self._norm(self.base_rotor.angular_position)
            desired_ang = self._norm(self.initial_angle_diff + self.target_angle)
            error = self._norm(desired_ang - current_ang)

            step = np.clip(error * 0.5, -10.0, 10.0)
            self.base_rotor.angular_position += step

    def draw(self, __window):
        global_pos = self.get_global_anchor()
        px, py = int(global_pos[0]), int(global_pos[1])
        # Draw gray if disabled to show it is free-spinning
        color = (255, 140, 0) if self.enabled else (150, 150, 150)
        pygame.draw.circle(__window, color, (px, py), 18, 5)


class Link(PhysicsObject):
    def __init__(self, length=100, thickness=3):
        super().__init__()
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
        return np.linalg.norm(mouse - start) <= 10

    def is_intersecting_end(self, x, y):
        _, _, end_x, end_y = self.get_points()
        mouse = np.array([x, y])
        end = np.array([end_x, end_y])
        return np.linalg.norm(mouse - end) <= 10

    def is_intersecting_center(self, x, y):
        start_x, start_y, end_x, end_y = self.get_points()
        start = np.array([start_x, start_y])
        end = np.array([end_x, end_y])
        mouse = np.array([x, y])

        if self.is_intersecting_start(x, y) or self.is_intersecting_end(x, y):
            return False

        line_vec = end - start
        mouse_vec = mouse - start
        line_len = self.length

        if line_len == 0: return False
        line_unit = line_vec / line_len
        projection_length = np.dot(mouse_vec, line_unit)

        if 0 <= projection_length <= line_len:
            closest_point = start + projection_length * line_unit
            distance = np.linalg.norm(mouse - closest_point)
            return distance < 5
        return False

    def get_node_at(self, x, y):
        if self.is_intersecting_start(x, y): return 0.0
        if self.is_intersecting_end(x, y): return 1.0
        if self.is_intersecting_center(x, y): return 'center'
        return None

    def get_global_position(self, ratio):
        start_x, start_y, end_x, end_y = self.get_points()
        if ratio == 0.0:
            return np.array([start_x, start_y])
        elif ratio == 1.0:
            return np.array([end_x, end_y])
        elif ratio == 'center':
            return np.array([(start_x + end_x) / 2, (start_y + end_y) / 2])
        return None