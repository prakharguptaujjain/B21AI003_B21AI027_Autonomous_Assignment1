import rclpy
from rclpy.node import Node
from agent_interfaces.srv import NavigationRequest, EscortRequest
from campus_map.campus_map import CampusMap
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import time
import os
import random
import pickle

class CIAgent(Node):
    def __init__(self):
        super().__init__('ci_agent')
        self.get_logger().info("CI Agent is ready.")
        
        # Service client to communicate with BI agent
        self.nav_request_client = self.create_client(NavigationRequest, 'navigation_request')
        
        # Service to handle visitor escort requests
        self.escort_request_srv = self.create_service(EscortRequest, 'escort_request', self.handle_escort_request)

        # Log file for communication
        self.log_file = open("/tmp/ci_agent_log.txt", "w")

        # Publisher for visualization in RViz using MarkerArray
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        # Timer to periodically publish the markers (every 1 second)
        self.timer = self.create_timer(10.0, self.publish_map)

        # Initialize CI agent's initial position (e.g., at the entrance)
        self.agent_marker = self.create_ci_agent_marker()

        self.campus_map = CampusMap()
        self.campus_map.build_campus()

        self.locked=False

    def publish_map(self):
        # Create an empty MarkerArray to hold the CI agent's marker
        self.marker_array = MarkerArray()

        # Add the CI agent marker to the MarkerArray
        self.marker_array.markers.append(self.agent_marker)

        # Publish the MarkerArray to RViz
        self.publisher.publish(self.marker_array)

    def create_ci_agent_marker(self):
        """Create a marker for the CI agent in RViz."""
        marker = Marker()
        marker.header.frame_id = 'map'  # Use 'map' frame or whatever global frame you are using
        marker.ns = 'ci_agent'  # Namespace for the marker
        marker.id = 0  # Unique ID for the CI agent marker
        marker.type = Marker.SPHERE  # Use a sphere marker to represent the CI agent
        marker.action = Marker.ADD  # Add or modify the marker
        marker.scale.x = 0.5  # Scale of the marker (0.5m in diameter)
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 0.0  # Blue color for the CI agent marker
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Alpha (fully opaque)

        # Set the position of the marker (CI agent position)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0  # Assume the agent moves in 2D (on the ground)

        return marker
        
    def handle_escort_request(self, request, response):
        visitor_id = request.visitor_id
        building_location = request.building_location
        room_number = int(request.room_number)
        oosrequest = request.oosrequest
        oosmeettime = request.oosmeettime

        while self.locked:
            self.get_logger().warn("Pending Requests to assist")
            time.sleep(3)

            
        self.locked=True

        

        self.get_logger().info(f"Received escort request from visitor {visitor_id} to building {building_location}")

        # Log the request
        self.log(f"Escort request from visitor {visitor_id} to {building_location}")

        # Simulate pathfinding from the Main Gate to the Building Entrance
        # mode = random.choices(['walk', 'vehicle', 'cycle'])[0]
        mode='vehicle'
        path_to_building = self.campus_map.shortest_path_campus("Main_Gate", f"{building_location}_Entrance", mode)
        path_to_building = eval(str(path_to_building))
        self.get_logger().info(f"Path to {building_location}_Entrance: {path_to_building}")

        # Visualize the path using RViz markers
        self.visualize_path(path_to_building)
        self.get_logger().info(f"Escorting visitor {visitor_id} to {building_location}")

        # Simulate agent movement along the path
        self.get_logger().info(f"Path to building: {path_to_building}")
        # save pickle 
        os.makedirs('/tmp', exist_ok=True)

        with open(f'/tmp/{visitor_id}_path.pkl_ci', 'wb') as f:
            pickle.dump({'success': True, 'message': 'Path found', 'path': path_to_building, 'mode': mode}, f)

        time.sleep(1)
        self.move_agent_along_path(path_to_building, mode)
        

        # with open(f'/tmp/{visitor_id}_cihasmoved1', 'w') as f:
        #     f.write("True")

        # while not os.path.exists(f'/tmp/{visitor_id}_hasmoved'):
        #     time.sleep(1)

        building_room = f"{building_location}_Room{room_number}"
        result = self.request_navigation_from_bi(building_room, visitor_id, oosrequest,oosmeettime)
        if result['authorized']:
            response.success = True
            response.message = f"Escorting visitor {visitor_id} to {building_room}."
            response.path = str(result.get('path', []))
        
        self.get_logger().info(f"Path to building saved to /tmp/{visitor_id}_path.pkl_ci")

        # Simulate navigation inside the building by contacting the BI agent
        # room_number = 1
        self.locked=True
        self.get_logger().info(f"Navigation result: {result}")
        if result['authorized']:
            response.success = True
            response.message = f"Escorting visitor {visitor_id} to {building_room}."
            # self.log(f"Escorting visitor {visitor_id} to {building_room}")
            self.get_logger().info(f"Escorting visitor {visitor_id} to {building_room}")
            
            # Simulate agent movement along the path
            try:
                path = eval(str(result.get('path', [])))
            except Exception as e:
                self.get_logger().error(f"Error: {result}")
                path = []
                self.locked=False
            response.path = str(path)
            self.get_logger().info(f"Path to building_room: {path}")
            self.move_agent_along_path(path, mode)

            with open(f'/tmp/{visitor_id}_cihasmoved2', 'w') as f:
                f.write("True")
            self.locked=False
        else:
            response.success = False
            response.message = f"Access denied for visitor {visitor_id}."
            response.path = ''
            self.log(f"Access denied for visitor {visitor_id} to {building_location}")
            self.locked=False
            

        return response

    def visualize_path(self, path):
        """Visualizes the path on the campus map using RViz markers within the MarkerArray."""
        # Clear previous path markers if needed
        # check if self.marker_array exists else call self.publish_map()
        if not hasattr(self, 'marker_array'):
            self.publish_map()
        self.marker_array.markers = [self.agent_marker]  # Keep the agent marker

        # Create markers for the path
        for i in range(len(path) - 1):
            start_pos = self.campus_map.get_position(path[i])
            end_pos = self.campus_map.get_position(path[i + 1])
            
            path_marker = Marker()
            path_marker.header.frame_id = 'map'
            path_marker.ns = 'paths'
            path_marker.id = i + 1  # Unique ID for each path segment
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.1  # Width of the line
            path_marker.color.r = 0.0   # Red color for the path
            path_marker.color.g = 1.0   # No green
            path_marker.color.b = 0.0   # No blue
            path_marker.color.a = 1.0    # Fully opaque

            # Add start and end positions to the line marker
            start_point = Point(x=start_pos[0], y=start_pos[1], z=0.0)
            end_point = Point(x=end_pos[0], y=end_pos[1], z=0.0)
            path_marker.points.append(start_point)
            path_marker.points.append(end_point)

            # Add path marker to the MarkerArray
            self.marker_array.markers.append(path_marker)

        # Publish the updated MarkerArray with path markers
        self.publisher.publish(self.marker_array)

    def move_agent_along_path(self, path, mode):
        """Simulate moving the CI agent along the provided path and updating the marker."""
        # for location in path:
        #     pos = self.campus_map.get_position(location)
            
        #     # Update the marker's position in RViz
        #     self.agent_marker.pose.position.x = pos[0]
        #     self.agent_marker.pose.position.y = pos[1]
        #     self.agent_marker.pose.position.z = 0.0  # 2D visualization

        #     self.get_logger().info(f"Moving agent to position ({pos[0]}, {pos[1]})")

        #     # Update the agent marker in the MarkerArray
        #     self.marker_array.markers[0] = self.agent_marker  # Update the agent marker in the array

        #     # Publish the updated MarkerArray with the agent's new position
        #     self.publisher.publish(self.marker_array)

        #     # Simulate low velocity by adding a delay
        #     time.sleep(1)  # Adjust the sleep time as necessary to control speed

        for idx in range(len(path) - 1):
            start = self.campus_map.get_position(path[idx])
            end = self.campus_map.get_position(path[idx + 1])
            try:
                m = (end[1] - start[1]) / (end[0] - start[0])
            except Exception as e:
                self.get_logger().error(f"Error: {path}")
                self.locked=False
                continue
            self.__speeds = {
                'walk': 0.5,    # Average walking speed
                'cycle': 1.0,   # Average cycling speed
                'vehicle': 2.0  # Average vehicle speed
            }

            new_positions = []
            # Calculate the new positions along the path with a step size depending on the speed
            # number of steps = distance / speed
            step_size = self.__speeds[mode]
            distance = ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5
            num_steps = int(distance / step_size)
            lamdas = [1/num_steps * i for i in range(num_steps)]
            self.get_logger().info(f"lamdas: {lamdas}")
            for i in range(num_steps - 1):
                x = start[0] + lamdas[i] * (end[0] - start[0])
                y = start[1] + lamdas[i] * (end[1] - start[1])
                new_positions.append((x, y))

            new_positions.append(end)

            self.get_logger().info(f"New positions: {new_positions}")
            for pos in new_positions:
                # Update the marker's position in RViz
                self.agent_marker.pose.position.x = float(pos[0])
                self.agent_marker.pose.position.y = float(pos[1])
                self.agent_marker.pose.position.z = 0.0

                self.marker_array.markers[0] = self.agent_marker
                self.publisher.publish(self.marker_array)



                time.sleep(1)


    def request_navigation_from_bi(self, building_room, visitor_id,oosrequest,oosmeettime):
        # Request navigation inside the building
        while not self.nav_request_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for BI agent to become available...')
        
        req = NavigationRequest.Request()
        req.host_location = building_room
        req.visitor_id = visitor_id
        req.oosrequest = oosrequest
        req.oosmeettime = oosmeettime
        
        self.get_logger().info(f"Requesting navigation to {building_room}")
        future = self.nav_request_client.call_async(req)
        # rclpy.spin_until_future_complete(self, future)
        # while not future.done():
        #     rclpy.spin_once(self, timeout_sec=1.0)

        while os.path.exists(f'/tmp/{req.visitor_id}_path.pkl_bi') == False:
            time.sleep(1)
        
        with open(f"/tmp/{req.visitor_id}_path.pkl_bi", "rb") as f:
            response = pickle.load(f)

        # self.get_logger().info(f"Navigation result: {future}")
        if response is not None:
            try:
                # Get the result from the service call
                result = response
                self.locked=False
                self.get_logger().info(f"Navigation result received: authorized={result.authorized}, path={result.path}")
                return {'authorized': result.authorized, 'path': result.path}
            except Exception as e:
                # Log any exceptions raised during the service call
                self.get_logger().error(f"Service call failed with error: {e}")
                self.locked=False
                return {'authorized': False, 'path': ''}
        else:
            # If the service did not return any result or failed
            self.get_logger().error("Failed to receive a response from the BI agent.")
            return {'authorized': False, 'path': ''}

    def log(self, message):
        self.log_file.write(f"{message}\n")
        self.log_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = CIAgent()
    rclpy.spin(node)
    node.log_file.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
