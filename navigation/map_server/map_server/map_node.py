#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

from map_server.map_maker import AddObject, add_object, create_occupancymap, create_costmap
#from interfaces.srv import AddObjectSrv
import yaml
import os
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

def read_pgm(pgmf):
    """Return a raster of integers from a PGM as a list of lists."""
    assert pgmf.readline() == b'P5\n'
    (width, height) = [int(i) for i in pgmf.readline().split()]
    depth = int(pgmf.readline())
    assert depth <= 255

    raster = []
    for _ in range(height * width):
        # 0 => Free, 33 => Unknown, 85 => Occupied
        value = int.from_bytes(pgmf.read(1), 'big') // 3
        raster.append(value)
    return width, height, raster



class MapServer(Node):
    def __init__(self):
        super().__init__('map_node')

        self.map = None
        self.costmap = None

        self.object_map = None
        self.objects = []

        qos_profile = QoSProfile(depth=3,durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.yaml_filename = self.declare_parameter('yaml_filename', '/workspaces/isaac_ros-dev/src/isro-rover-challenge/navigation/map_server/config/map.yaml').get_parameter_value().string_value
        self.frame_id = self.declare_parameter('frame_id', 'odom').get_parameter_value().string_value
        self.inflation = 10#self.declare_parameter('inflation', 10).get_parameter_value()

        #self.add_object_service = self.create_service(AddObjectSrv, 'add_object', self.add_object_callback)
        self.occ_service = self.create_service(GetMap, 'get_map', self.get_map_callback)
        self.occ_pub = self.create_publisher(OccupancyGrid,'map', qos_profile=qos_profile)
        self.costmap_pub = self.create_publisher(OccupancyGrid,'cost_map_static' ,qos_profile=qos_profile)

        self.on_configure(None)
    
    def get_map_callback(self, request):
        if self.map:
            return self.map
        return None

    def on_configure(self, state):
        self.get_logger().info('Configuring')

        if self.yaml_filename:
            if not self.load_map_from_yaml(self.yaml_filename):
                raise RuntimeError("Failed to load map yaml file: " + self.yaml_filename)
        else:
            self.get_logger().info("yaml-filename parameter is empty")
    

        #self.costmap_pub.publish(self.costmap)

    def on_cleanup(self, state):
        self.occ_pub.destroy()
        self.add_object_service.destroy()
        self.occ_service.destroy()
        self.object_map = None
        self.objects = []
        self.map = None
    
    def load_map_from_yaml(self, yaml_file):
        with open(yaml_file, 'r') as file:
            yaml_data = yaml.safe_load(file)

        yaml_dir = os.path.dirname(yaml_file)
        image_path = os.path.join(yaml_dir, yaml_data['image'])
        objectmap_path = os.path.join(yaml_dir, yaml_data['objectmap'])

        width, height, image = read_pgm(open(image_path, 'rb'))
        resolution = yaml_data['resolution']
        origin = yaml_data['origin']
        object_map = np.loadtxt(objectmap_path)
        object_map = np.flipud(object_map)
        # object_map = np.fliplr(object_map)

        # Create Object Map


        # Load Occupancy Map
        map = OccupancyGrid()
        map.header.frame_id = self.frame_id
        map.info.width = width
        map.info.height = height
        map.info.resolution = resolution
        map.info.origin.position.x = origin[0]
        map.info.origin.position.y = origin[1]
        map.info.origin.position.z = origin[2]
        map.info.origin.orientation.x = 0.0 #origin[3]
        map.info.origin.orientation.y = 0.0 #origin[4]
        map.info.origin.orientation.z = 0.0 #origin[5]
        map.info.origin.orientation.w = 0.0 #origin[6]
        map.data = create_occupancymap(object_map)
        self.map = map
        self.occ_pub.publish(self.map)

        costmap = OccupancyGrid()
        costmap.header.frame_id = self.frame_id
        costmap.info = map.info
        costmap.info.origin.position.x = origin[0] - 10
        costmap.info.origin.position.y = origin[1] - 4
        costmap.info.origin.position.z = 0.0
        costmap.info.origin.orientation.z = 0.0
        costmap.info.origin.orientation.w = -1.0 #origin[5]
        costmap.data = create_occupancymap(np.fliplr(np.flipud(object_map)))
        self.costmap = costmap
        self.costmap_pub.publish(self.costmap)

        self.get_logger().info("Loaded yaml file!")
        self.get_logger().info(f"({min(image), max(image)})")
        # self.occ_pub.publish(self.map)
        
        self.get_logger().info('Published map')
        return True

def main(args=None):
    try:
        rclpy.init(args=args)
        mapserver = MapServer()
        rclpy.spin(mapserver)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

