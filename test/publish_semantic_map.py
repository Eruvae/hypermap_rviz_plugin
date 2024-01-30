import sys
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from hypermap_msgs.msg import SemanticMap, SemanticObject
from geometry_msgs.msg import Point32, Point

#int32 id
#geometry_msgs/Polygon shape
#geometry_msgs/Point position # should be centroid of shape
#string name
#string[] tags
#float64[] confidence

class SemanticMapPublisher(Node):

    def __init__(self, semantic_map_file):
        super().__init__('semantic_map_publisher')
        self.publisher_ = self.create_publisher(SemanticMap, 'semantic_map', QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        msg = self.read_semantic_map(semantic_map_file)
        self.publish_map(msg)

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

    def publish_map(self, msg):
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    if (len(sys.argv) < 2):
        print("Usage: publish_semantic_map.py <semantic_map.yaml>")
        exit()

    semantic_map_publisher = SemanticMapPublisher(sys.argv[1])

    rclpy.spin(semantic_map_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    semantic_map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()