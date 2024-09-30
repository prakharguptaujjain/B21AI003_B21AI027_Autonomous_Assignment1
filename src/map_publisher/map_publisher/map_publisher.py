import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from campus_map.campus_map import CampusMap

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.get_logger().info(f"Map Started")

        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 50)
        self.campus_map = CampusMap()
        self.publish_map()
        self.timer = self.create_timer(10.0, self.publish_map)

    def publish_map(self):
        marker_array = MarkerArray()
        marker_array.markers.extend(self.create_node_markers())
        marker_array.markers.extend(self.create_path_markers())
        self.publisher.publish(marker_array)

    def create_node_markers(self):
        markers = []
        for i, (node_name, (x, y)) in enumerate(self.campus_map.get_nodes().items()):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns = 'nodes'
            marker.id = i
            if node_name == 'Main_Gate' or 'Entrance' in node_name:
                marker.type = Marker.CUBE
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
            else:
                marker.type = Marker.SPHERE
                marker.scale.x = 0.5
                marker.scale.y = 0.5
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.color.g = 1.0
            marker.color.a = 1.0
            markers.append(marker)

            self.get_logger().info(f"Creating Node {node_name}")
        return markers

    def create_path_markers(self):
        self.get_logger().info(f"Path Marker")
        markers = []
        for i, (start, end, way) in enumerate(self.campus_map.get_paths()):
            self.get_logger().info(f"Creating path from {start} to {end}")
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns = 'paths'
            marker.id = i + 100
            marker.type = Marker.LINE_STRIP
            marker.scale.x = 0.05  # Width of the line
            marker.color.r = 1.0   # Red color
            marker.color.g = 0.0   # No green
            marker.color.b = 0.0   # No blue
            marker.color.a = 1.0    # Fully opaque
            
            # Get the start and end positions
            start_pos = self.campus_map.get_nodes()[start]
            end_pos = self.campus_map.get_nodes()[end]

            # Ensure the values are float before creating Point
            marker.points.append(Point(x=float(start_pos[0]), y=float(start_pos[1]), z=0.0))
            marker.points.append(Point(x=float(end_pos[0]), y=float(end_pos[1]), z=0.0))
            
            self.get_logger().info(f"Creating path from {start_pos} to {end_pos}")
            markers.append(marker)
        return markers

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
