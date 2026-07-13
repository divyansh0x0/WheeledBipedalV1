import pygame
import numpy as np
import sys

# --- 1. Hardware Configuration (Meters) ---
L1 = 1.0  # Platform driven by Motor 1
L2 = 1.8  # Passive link from Motor 1 to End Effector
L3 = 1.0  # First link driven by Motor 2
L4 = 1.5  # Link from Elbow to End Effector


def forward_kinematics(theta1, theta2):
    """
    Computes the physical coordinates of the linkage.
    Returns (nodeA, nodeB, end_effector, determinant)
    """
    xA = L1 * np.cos(theta1)
    yA = L1 * np.sin(theta1)

    xB = xA + L3 * np.cos(theta2)
    yB = yA + L3 * np.sin(theta2)

    d = np.sqrt(xB ** 2 + yB ** 2)

    if d > L2 + L4 or d < abs(L2 - L4) or d == 0:
        return None, None, None, 0.0  # Unreachable configuration

    a = (L2 ** 2 - L4 ** 2 + d ** 2) / (2 * d)
    h = np.sqrt(abs(L2 ** 2 - a ** 2))

    x3 = (a / d) * xB
    y3 = (a / d) * yB

    xE = x3 + (h / d) * (-yB)
    yE = y3 + (h / d) * (xB)

    det = (xE * yB) - (yE * xB)

    return (xA, yA), (xB, yB), (xE, yE), det


# --- 2. System Initialization ---
pygame.init()
WIDTH, HEIGHT = 1000, 500
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("5-Bar Custom Linkage Real-Time State")
clock = pygame.time.Clock()
font = pygame.font.SysFont("monospace", 14)

# --- 3. Viewport Configuration ---
# Mechanism View (Left side)
SCALE = 80  # Pixels per physical meter
OFFSET_X = int(WIDTH * 0.25)
OFFSET_Y = int(HEIGHT * 0.5)


def to_screen(x, y):
    """Transforms Cartesian physical coordinates to Pygame raster coordinates."""
    screen_x = int(OFFSET_X + (x * SCALE))
    screen_y = int(OFFSET_Y - (y * SCALE))  # Invert Y axis
    return (screen_x, screen_y)


# Graph View (Right side)
GRAPH_X = int(WIDTH * 0.6)
GRAPH_Y_CENTER = int(HEIGHT * 0.5)
GRAPH_WIDTH = 350
GRAPH_HEIGHT = 400
GRAPH_SCALE_Y = 40  # Pixels per unit of determinant
history_det = []

# --- 4. Main Execution Loop ---
frame = 0
running = True

while running:
    # 1. Event Handling (Input processing)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 2. State Update (Simulating motor control)
    t = frame * 0.05
    theta1 = 0.5 * np.sin(t) + np.pi / 2
    theta2 = 1.2 * np.cos(t * 1.5) + np.pi / 2

    nodeA, nodeB, ee, det = forward_kinematics(theta1, theta2)

    # 3. Buffer Clearing
    screen.fill((240, 240, 240))  # Off-white background

    if ee is not None:
        # --- 4. Render Mechanism ---
        p0 = to_screen(0, 0)
        pA = to_screen(nodeA[0], nodeA[1])
        pB = to_screen(nodeB[0], nodeB[1])
        pE = to_screen(ee[0], ee[1])

        # Link 1 (Motor 1 Platform) - Thick Black
        pygame.draw.line(screen, (0, 0, 0), p0, pA, 6)
        # Link 2 (Passive Link) - Blue
        pygame.draw.line(screen, (0, 0, 255), p0, pE, 3)
        # Link 3 & 4 (Motor 2 Chain) - Red
        pygame.draw.line(screen, (255, 0, 0), pA, pB, 4)
        pygame.draw.line(screen, (255, 0, 0), pB, pE, 4)

        # Joints
        pygame.draw.circle(screen, (50, 50, 50), p0, 8)
        pygame.draw.circle(screen, (50, 50, 50), pA, 6)
        pygame.draw.circle(screen, (50, 50, 50), pB, 6)
        pygame.draw.circle(screen, (0, 180, 0), pE, 8)  # End Effector

        # --- 5. Render Singularity Graph ---
        history_det.append(det)
        if len(history_det) > GRAPH_WIDTH:
            history_det.pop(0)  # Maintain rolling window

        # Draw Graph Axes
        pygame.draw.line(screen, (150, 150, 150), (GRAPH_X, GRAPH_Y_CENTER), (GRAPH_X + GRAPH_WIDTH, GRAPH_Y_CENTER), 2)
        pygame.draw.line(screen, (150, 150, 150), (GRAPH_X, int(HEIGHT * 0.1)), (GRAPH_X, int(HEIGHT * 0.9)), 2)

        # Draw Data Line
        if len(history_det) > 1:
            points = []
            for i, val in enumerate(history_det):
                px = GRAPH_X + i
                py = GRAPH_Y_CENTER - int(val * GRAPH_SCALE_Y)
                # Clamp rendering to graph bounds to prevent drawing over the mechanism
                py = max(int(HEIGHT * 0.1), min(int(HEIGHT * 0.9), py))
                points.append((px, py))

            pygame.draw.lines(screen, (128, 0, 128), False, points, 2)

        # Draw Singularity Warning Threshold (Zero line)
        zero_text = font.render("Singularity (Det=0)", True, (255, 0, 0))
        screen.blit(zero_text, (GRAPH_X + 10, GRAPH_Y_CENTER - 20))

    else:
        # State: Unreachable hardware configuration
        err_text = font.render("ERROR: Target out of physical bounds.", True, (255, 0, 0))
        screen.blit(err_text, (20, 20))

    # 6. Hardware Update
    pygame.display.flip()  # Swap frame buffers
    clock.tick(60)  # Throttle execution to 60Hz
    frame += 1

pygame.quit()
sys.exit()