from dataclasses import dataclass

RED = (255, 0, 0)
YELLOW = (220, 220, 0)
GREEN = (0, 255, 0)
BLUE = (50, 164, 168)
GRAY = (150, 150, 150)
ORANGE = (205, 100, 0)
DARKORANGE = (204, 99, 0)
PURPLE = (255, 0, 255)
DARKPURPLE = (119, 50, 168)

coord_list = [[1, 'A1', (1.6, -0.9, 0.0)], [2, 'A1', (1.4, -1.7, 0.0)], [3, 'A1', (2.5, -1.4, 0.0)], [4, 'A1', (3.0, -2.3, 0.0)], [5, 'A1', (4.7, -1.8, 0.0)], [6, 'A1', (7.58, -2.7, 0.0)], [7, 'A2', (1.2, -2.5, 0.0)], [8, 'A2', (2.0, -0.6, 0.0)], [9, 'A2', (3.0, -0.5, 0.0)], [10, 'A2', (4.5, -1.25, 0.0)], [11, 'A2', (7.38, -1.0, 0.0)], [15, 'B2', (2.7, -0.7, 0.0)], [16, 'B2', (4.5, -3.0, 0.0)], [17, 'B2', (8.58, -0.9, 0.0)], [18, 'SP', (0.0, -1.2, 0.0)], [19, 'T', (3.3, -0.825, 0.0)], [20, 'WP', (2.9, -2.1, 0.0)], [21, 'FP', (9.25, -3.25, 0.0)]]
# [12, 'B1', (2.3, -2.2, 0.0)], [13, 'B1', (4.7, -2.4, 0.0)], [14, 'B1', (7.58, -1.8, 0.0)]
@dataclass
class Object:
    name: str
    cost: int
    color: tuple[int, int, int]
    width: int = None
    height: int = None
    radius: int = None


# object_map = {
#     "A1": Object("Small Obstacle", 15, YELLOW, width=15, height=15),
#     "A2": Object("Big Obstacle", 60, GREEN, width=30, height=30),
#     "B1": Object("Small Crater", 20, PURPLE, radius=10),
#     "B2": Object("Big Crater", 60, DARKPURPLE, radius=20),
#     "WP": Object("Waypoint", -10, GRAY, width=110, height=110),
#     "SP": Object("Start Point", 0, ORANGE, width=120, height=120),
#     "FP": Object("Final Point", -10, DARKORANGE, radius=75),
#     "F": Object("Final", -10, BLUE, width=10, height=20),
#     "T": Object("Tube", -10, RED, width=8, height=12)
# }

object_map = {
    "A1": Object("Small Obstacle", 0, YELLOW, width=15, height=15),
    "A2": Object("Big Obstacle", 0, GREEN, width=30, height=30),
    "B1": Object("Small Crater", 0, PURPLE, radius=10),
    "B2": Object("Big Crater", 0, DARKPURPLE, radius=20),
    "WP": Object("Waypoint", 255, GRAY, width=110, height=110),
    "SP": Object("Start Point", 255, ORANGE, width=120, height=120),
    "FP": Object("Final Point", 255, DARKORANGE, radius=75),
    "F": Object("Final", 255, BLUE, width=10, height=20),
    "T": Object("Tube", 0, RED, width=8, height=12)
}

color_map = {
    YELLOW: "A1",
    GREEN: "A2",
    PURPLE: "B1",
    DARKPURPLE: "B2",
    GRAY: "WP",
    ORANGE: "SP",
    DARKORANGE: "FP",
    BLUE: "F",
    RED: "T"
}

mandatory_points = [2, 12, 4]

from matplotlib import pyplot as plt

import numpy

# Size (200, 200) | Center (100, 100) | radius = 50
xx, yy = numpy.mgrid[:200, :200]
# circles contains the squared distance to the (100, 100) point
# we are just using the circle equation learnt at school
circle = (xx - 100) ** 2 + (yy - 100) ** 2
donut = circle < (900)
square = numpy.logical_and((xx - 10) > 0, (xx - 10) < 10) & numpy.logical_and((yy-10) > 10, (yy-10) < 20)

map = numpy.zeros((200, 200))
map[donut] = 1
map[square] = 2

import numpy as np

# def save_pgm(filename, data):
#     height, width = data.shape
#     max_val = np.max(data)

#     with open(filename, 'wb') as f:
#         f.write(b'P5\n')  # PGM magic number for binary gray map
#         f.write(f'{width} {height}\n'.encode())  # Image width and height
#         f.write(f'{max_val}\n'.encode())  # Maximum pixel value
#         f.write(data.astype(np.uint8).tobytes())  # Image data

# # Example usage
# # Create a random 2D NumPy array
data = np.random.randint(0, 256, size=(100, 100), dtype=np.uint8)
print(data)
# # Save the array as a PGM file
# save_pgm('output.pgm', data)