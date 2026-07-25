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
class FixedPivot:
    def __init__(self, body_a: PhysicsObject , anchor_a, fixed_anchor):
        self.body_a = body_a
        self.anchor_a = anchor_a
        self.fixed_anchor = fixed_anchor
    def get_global_anchors(self):
        theta_a = np.radians(self.body_a.angular_position)

        # Create separate rotation matrices for A and B
        rot_a = np.array([[np.cos(theta_a), -np.sin(theta_a)],
                          [np.sin(theta_a), np.cos(theta_a)]])

        # Apply the correct matrix to the correct anchor
        global_a = self.body_a.position + rot_a @ self.anchor_a

        return global_a

    def get_error(self):
        global_a = self.get_global_anchors()
        return global_a - self.fixed_anchor

    def draw(self, __window):
        global_a= self.get_global_anchors()
        pygame.draw.circle(__window, (0,0,255), global_a, 10,1)
        pygame.draw.circle(__window, (0,0,255), self.fixed_anchor, 10,1)

class Pivot(Constraint):
    def __init__(self, body_a: PhysicsObject, body_b: PhysicsObject, anchor_a, anchor_b):
        self.body_a = body_a
        self.body_b = body_b
        self.anchor_a = anchor_a
        self.anchor_b = anchor_b

    def get_global_anchors(self):
        theta_a = np.radians(self.body_a.angular_position)
        theta_b = np.radians(self.body_b.angular_position)

        # Create separate rotation matrices for A and B
        rot_a = np.array([[np.cos(theta_a), -np.sin(theta_a)],
                          [np.sin(theta_a), np.cos(theta_a)]])
        rot_b = np.array([[np.cos(theta_b), -np.sin(theta_b)],
                          [np.sin(theta_b), np.cos(theta_b)]])

        # Apply the correct matrix to the correct anchor
        global_a = self.body_a.position + rot_a @ self.anchor_a
        global_b = self.body_b.position + rot_b @ self.anchor_b

        return global_a, global_b

    def get_error(self):
        global_a, global_b = self.get_global_anchors()
        return global_a - global_b

    def draw(self, __window):
        global_a, global_b = self.get_global_anchors()
        pygame.draw.circle(__window, (0,0,255), global_a, 10,1)
        pygame.draw.circle(__window, (0,0,255), global_b, 10,1)


class Link(PhysicsObject):
    def __init__(self, length=100,thickness=3):
        super().__init__()
        self.__drag_offset = np.array([0,0])
        self.__is_dragging:bool = False
        self.length = length
        self.thickness = thickness
        self.__end_point_radius = 10

    def get_points(self):
        start_x, startY = self.position
        end_x = start_x + self.length * math.cos(self.angular_position * math.pi / 180)
        end_y = startY + self.length * math.sin(self.angular_position * math.pi / 180)
        return start_x, startY, end_x, end_y

    def update(self):
        super().update()
        if self.__is_dragging:
            x, y = pygame.mouse.get_pos()
            mouse_pos = np.array([float(x), float(y)])

            # Apply the offset we calculated when the mouse was first clicked
            self.position = mouse_pos + self.__drag_offset

    def draw(self, dt, surface) -> None:
        start_x, start_y, end_x, end_y = self.get_points()
        line_color = (0,0,0)
        if self.__is_dragging:
            line_color = (0,0,255)

        pygame.draw.line(surface, line_color, (start_x, start_y), (end_x, end_y), self.thickness)
        pygame.draw.circle(surface, line_color, (start_x, start_y), self.__end_point_radius)

        pygame.draw.circle(surface, line_color, (end_x, end_y), self.__end_point_radius)

    def event_handler(self):
        mouse_btns = pygame.mouse.get_pressed()
        x, y = pygame.mouse.get_pos()

        if not mouse_btns[0]:
            # Mouse released, stop dragging
            self.__is_dragging = False
        else:
            # If the button is held, ONLY check intersection if we aren't already dragging.
            # This prevents the drag from dropping if the mouse moves too fast.
            if not self.__is_dragging:
                if self.is_intersecting_center(x, y):
                    self.__is_dragging = True
                    # NEW: Calculate and save the offset once!
                    mouse_pos = np.array([float(x), float(y)])
                    self.__drag_offset = self.position - mouse_pos

    def is_intersecting_center(self, x, y):
        start_x, start_y, end_x, end_y = self.get_points()

        # Convert points to NumPy arrays for easy vector math
        start = np.array([start_x, start_y])
        end = np.array([end_x, end_y])
        mouse = np.array([x, y])

        # 1. Guard Clause: Check if the mouse is inside the corner circles
        # np.linalg.norm calculates the Euclidean distance between two points
        if np.linalg.norm(mouse - start) <= self.__end_point_radius:
            return False
        if np.linalg.norm(mouse - end) <= self.__end_point_radius:
            return False

        # 2. Calculate line vectors
        line_vec = end - start
        mouse_vec = mouse - start

        # We must use the true total length to create an accurate unit vector
        line_len = self.length

        if line_len == 0:
            return False

        # Create a unit vector (length of exactly 1) pointing from start to end
        line_unit = line_vec / line_len

        # Project the mouse vector onto the line to find how far along the line it is
        projection_length = np.dot(mouse_vec, line_unit)

        # 3. Check if the mouse falls within the segment's length
        if 0 <= projection_length <= line_len:
            # Find the exact closest point mathematically on the line
            closest_point = start + projection_length * line_unit

            # Calculate distance from the actual mouse position to that projected point
            distance = np.linalg.norm(mouse - closest_point)

            # Return true if within 5 pixels (line thickness tolerance)
            return distance < 5

        return False




