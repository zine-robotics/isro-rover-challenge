from enum import IntEnum
from dataclasses import dataclass
from typing import Tuple
from geometry_msgs.msg import Pose

from scipy.ndimage import gaussian_filter
import numpy as np


RED = (255, 0, 0)
YELLOW = (220, 220, 0)
GREEN = (0, 255, 0)
BLUE = (50, 164, 168)
GRAY = (150, 150, 150)
ORANGE = (205, 100, 0)
DARKORANGE = (204, 99, 0)
PURPLE = (255, 0, 255)
DARKPURPLE = (119, 50, 168)
WHITE = (255,255,255)

class ObjectType(IntEnum):
  SmallObstacle = 1
  BigObstacle = 2
  SmallCrater = 3
  BigCrater = 4
  Waypoint = 5
  StartPoint = 6
  FinalPoint = 7
  Final = 8
  Tube = 9

@dataclass
class Object:
  tag: ObjectType
  cost: int
  occupancy: int
  color: Tuple[int, int, int]
  width: int = None
  height: int = None
  radius: int = None

class AddObject:
  pose: Pose
  tag: int


object_map = {
  ObjectType.SmallObstacle: Object(ObjectType.SmallObstacle, 15, 20, YELLOW, width=15, height=15),
  ObjectType.BigObstacle: Object(ObjectType.BigObstacle, 60, 100, GREEN, width=30, height=30),
  ObjectType.SmallCrater: Object(ObjectType.SmallCrater, 20, 20, PURPLE, radius=10),
  ObjectType.BigCrater: Object(ObjectType.BigCrater, 60, 100, DARKPURPLE, radius=20),
  ObjectType.Waypoint: Object(ObjectType.Waypoint, 0, 0, GRAY, width=110, height=110),
  ObjectType.StartPoint: Object(ObjectType.StartPoint, 0, 0, ORANGE, width=120, height=120),
  ObjectType.FinalPoint: Object(ObjectType.FinalPoint, 0, 0, DARKORANGE, radius=75),
  ObjectType.Final: Object(ObjectType.Final, 20, 20, BLUE, width=10, height=20),
  ObjectType.Tube: Object(ObjectType.Tube, 20, 20, RED, width=8, height=12)
}

def add_object(map: np.array, obj1: AddObject):
  height, width = map.shape
  center_x = int(obj1.pose.position.x * 100)
  center_y = int(obj1.pose.position.y * 100)
  if obj1.tag == 0:
    return map
  
  obj = object_map[obj1.tag]
  # xx, yy = np.mgrid[:height, :width]
  x_grid = np.arange(width)
  y_grid = np.arange(height)
  xx, yy = np.meshgrid(x_grid, y_grid)

  def draw_square(obj: Object, center_x, center_y):
    square = np.logical_and((xx - center_x) > 0, (xx - center_x) < obj.width) & np.logical_and((yy - center_y) > 0, (yy - center_y) < obj.height)
    map[square] = obj.tag

  #draw_circle(obj, center_x, center_y)
  radius = obj.radius or obj.width
  circle = (xx - center_x) ** 2 + (yy - center_y) ** 2
  map[circle <= radius ** 2] = obj.tag
  # print(map)
  return map


def create_occupancymap(map: np.array):
  height, width = map.shape

  costmap = [-1] * (width * height)
  for i in range(height):
    for j in range(width):
      if map[i][j] == 0:
        continue
  
      obj = object_map[int(map[i][j])]
      costmap[i*width + j] = obj.occupancy
  
  return np.flipud(costmap).tolist()
  #return costmap

def create_costmap(map: np.array, inflation):
  height, width = map.shape

  costmap = np.full((height, width), 0)
  for i in range(height):
    for j in range(width):
      if map[i][j] == 0:
        continue
  
      obj = object_map[int(map[i][j])]
      costmap[i][j] = obj.cost

  # High cost on borders
  costmap[0, :] = 100
  costmap[:, 0] = 100
  costmap[-1, :] = 100
  costmap[:, -1] = 100
  
  # creates an inflation
  sigma = inflation
  costmap = gaussian_filter(costmap, sigma=sigma).astype('int8')
  return costmap.flatten()
