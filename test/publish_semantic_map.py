import sys
import yaml

import rclpy
from rclpy.node import Node
#from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from hypermap_msgs.msg import SemanticMap, SemanticObject
from geometry_msgs.msg import Point32, Point

class SemanticMapPublisher(Node):

    def __init__(self, semantic_map_file):
        super().__init__('semantic_map_publisher')
        #latched: QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(SemanticMap, 'semantic_map', 1)
        self.msg = self.read_semantic_map(semantic_map_file)
        self.timer = self.create_timer(1.0, self.publish_map)

    def read_semantic_map(self, semantic_map_file):
        msg = SemanticMap()
        id = 0
        with open(semantic_map_file, 'r') as file:
            try:
                semantic_map = yaml.full_load(file)
                if not isinstance(semantic_map, list):
                    raise yaml.YAMLError("Semantic map must be a list of objects")
                for obj in semantic_map:
                    semantic_object = SemanticObject()
                    semantic_object.id = id
                    id += 1
                    semantic_object.name = obj['name']
                    shape_str = obj['shape'][9:-2] # ignore POLYGON(( and ))
                    points_str = shape_str.split(', ')
                    centroid = Point()
                    for point_str in points_str:
                        coord_str = point_str.split(' ')
                        point = Point32()
                        point.x = float(coord_str[0])
                        point.y = float(coord_str[1])
                        semantic_object.shape.points.append(point)
                        centroid.x += point.x
                        centroid.y += point.y

                    centroid.x /= len(points_str)
                    centroid.y /= len(points_str)
                    semantic_object.position = centroid                    
                    msg.objects.append(semantic_object)
                    
            except yaml.YAMLError as exc:
                print(exc)

        return msg

    def publish_map(self):
        self.msg.header.frame_id = "map"
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    if (len(sys.argv) < 2):
        print("Usage: publish_semantic_map.py <semantic_map.yaml>")
        exit()

    semantic_map_publisher = SemanticMapPublisher(sys.argv[1])

    rclpy.spin(semantic_map_publisher)

    semantic_map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()