import rclpy
from rclpy.node import Node
from agent_interfaces.srv import EscortRequest
from campus_map.campus_map import CampusMap
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import os
import pickle
import time
import random

class VisitorAgent(Node):
    def __init__(self):
        super().__init__('visitor_agent')
        self.get_logger().info("Visitor Agent is ready.")
        
        # Service client to request escort from CI agent
        self.escort_request_client = self.create_client(EscortRequest, 'escort_request')
        
        # Publisher for visualization in RViz using MarkerArray
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        # Timer to periodically publish the markers (every 1 second)
        # self.timer = self.create_timer(1.0, self.publish_map)

        # Initialize CI agent's initial position (e.g., at the entrance)
        self.agent_marker = self.create_visitor_marker()
        self.publish_map()

        if not hasattr(self, 'marker_array'):
            self.publish_map()

        self.get_logger().info(f"Publishing visitor agent marker at ({self.agent_marker.pose.position.x}, {self.agent_marker.pose.position.y})")

        self.campus_map = CampusMap()
        self.campus_map.build_campus()

        self.oosrequest=random.getrandbits(1)
        # self.oosrequest = 1

        visitor_number = random.randint(2, 50)
        path = f'/tmp/visitor_{visitor_number}_path.pkl_ci'
        while os.path.exists(path):
            visitor_number = random.randint(1, 3)
            path = f'/tmp/visitor_{visitor_number}_path.pkl_ci'
            
        self.get_logger().info(f"Visitor number: {visitor_number}")
        self.request_escort(f"visitor_{visitor_number}", f"Building_{random.choices(['A', 'B', 'C'])[0]}", f"{random.randint(1, 3)}",self.oosrequest)

    def publish_map(self):
        # Create an empty MarkerArray to hold the CI agent's marker
        self.marker_array = MarkerArray()

        # Add the CI agent marker to the MarkerArray
        self.marker_array.markers.append(self.agent_marker)

        # Publish the MarkerArray to RViz
        self.publisher.publish(self.marker_array)

    def create_visitor_marker(self):
        """Create a marker for the CI agent in RViz."""
        marker = Marker()
        marker.header.frame_id = 'map'  # Use 'map' frame or whatever global frame you are using
        marker.ns = 'visitor'  # Namespace for the marker
        marker.id = 0  # Unique ID for the CI agent marker
        marker.type = Marker.SPHERE  # Use a sphere marker to represent the CI agent
        marker.action = Marker.ADD  # Add or modify the marker
        marker.scale.x = 0.5  # Scale of the marker (0.5m in diameter)
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 0.0  
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0  

        # Set the position of the marker (CI agent position)
        marker.pose.position.x = 0.0
        marker.pose.position.y = -1.0
        marker.pose.position.z = 0.0  # Assume the agent moves in 2D (on the ground)
        
        return marker

    def move_agent_along_path(self, path, mode = 'cycle'):
        """Simulate moving the CI agent along the provided path and updating the marker."""
        for idx in range(len(path) - 1):
            start = self.campus_map.get_position(path[idx])
            end = self.campus_map.get_position(path[idx + 1])
            try:
                m = (end[1] - start[1]) / (end[0] - start[0])
            except Exception as e:
                self.get_logger().error(f"Error: {path}")
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
            lamdas = [(1/num_steps * i) for i in range(num_steps)]
            self.get_logger().info(f"lamdas: {lamdas}")
            for i in range(num_steps - 1):
                x = start[0] + lamdas[i] * (end[0] - start[0]) 
                y = start[1] + lamdas[i] * (end[1] - start[1])
                if(end[0] > start[0]):
                    x -= 0.5 * (end[0] - start[0]) / distance
                    y -= 0.5 * (end[1] - start[1]) / distance
                else:
                    x += 0.5 * (end[0] - start[0]) / distance
                    y += 0.5 * (end[1] - start[1]) / distance

                new_positions.append((x, y))

            if end[0] > start[0]:
                new_positions.append((end[0] - 0.5 * (end[0] - start[0]) / distance, end[1] - 0.5 * (end[1] - start[1]) / distance))
            else:
                new_positions.append((end[0] + 0.5 * (end[0] - start[0]) / distance, end[1] + 0.5 * (end[1] - start[1]) / distance))

            self.get_logger().info(f"New positions: {new_positions}")
            for pos in new_positions:
                # Update the marker's position in RViz
                self.agent_marker.pose.position.x = float(pos[0])
                self.agent_marker.pose.position.y = float(pos[1])
                self.agent_marker.pose.position.z = 0.0

                self.marker_array.markers[0] = self.agent_marker
                self.publisher.publish(self.marker_array)



                time.sleep(1)

    def request_escort(self, visitor_id, building_location, room_number,oosrequest):
        while not self.escort_request_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for CI agent to become available...')
        
        req = EscortRequest.Request()
        req.visitor_id = visitor_id
        req.building_location = building_location
        req.room_number = room_number
        self.get_logger().info(f"oos request: {oosrequest}")
        oosmeettime = 0
        if oosrequest:
            req.oosrequest = True
            req.oosmeettime = random.randint(25, 40)
            oosmeettime = req.oosmeettime
            self.get_logger().info(f"oos request time: {req.oosmeettime}")
        else:
            req.oosrequest = False
            req.oosmeettime = 0
        
        # do the process in separate thread
        future = self.escort_request_client.call_async(req)
        # rclpy.spin_until_future_complete(self, future)
        while os.path.exists(f'/tmp/{visitor_id}_path.pkl_ci') == False:
            time.sleep(1)


        self.get_logger().info(f"Requesting escort to {building_location} room {room_number}")
        # load pickle
        with open(f'/tmp/{visitor_id}_path.pkl_ci', 'rb') as f:
            result = pickle.load(f)
            self.get_logger().info(f"Loaded picklessss: {result}")

        # while os.path.exists(f'/tmp/{visitor_id}_cihasmoved1') == False:
        #     time.sleep(1)

        # follow the path
        try:
            path_to_follow = eval(str(result['path']))
        except Exception as e:
            self.get_logger().error(f"Some error: {result}")
            self.clean_up(visitor_id)
            self.get_logger().info(f"Escort request completed for visitor {visitor_id}")
            self.shutdown()

        self.move_agent_along_path(path_to_follow, result['mode'])

        with open(f'/tmp/{visitor_id}_hasmoved', 'w') as f:
            f.write("True")



        while True:
            while os.path.exists(f'/tmp/{visitor_id}_path.pkl_bi') == False:
                time.sleep(1)

            time.sleep(1)

            with open(f'/tmp/{visitor_id}_path.pkl_bi', 'rb') as f:
                res2 = pickle.load(f)

            if res2.ooswaittime == 0:
                break
            else:
                self.get_logger().warn(f"Waiting for oos to unlock for {res2.ooswaittime} seconds")
                time.sleep(res2.ooswaittime+1)
                self.escort_request_client.call_async(req)

            if os.path.exists(f'/tmp/{visitor_id}_path.pkl_bi'):
                os.remove(f'/tmp/{visitor_id}_path.pkl_bi')


        # while os.path.exists(f'/tmp/{visitor_id}_cihasmoved2') == False:
        #     time.sleep(1)


        try:
            path_to_follow = eval(str(res2.path))
        except Exception as e:
            self.get_logger().error(f"Some error: {res2}")
            self.clean_up(visitor_id)
            self.get_logger().info(f"Escort request completed for visitor {visitor_id}")
            self.shutdown()

        if oosmeettime > 0:
            self.get_logger().warn(f"Starting Meeting with BI (oos) for {oosmeettime} seconds")
            time.sleep(oosmeettime)
            self.get_logger().warn(f"Meeting with BI (oos) completed")
        self.move_agent_along_path(path_to_follow)

        self.get_logger().info(f"Moving agent to position ({path_to_follow})")
        
        try:
            if result['success']:
                self.get_logger().info("Success result")
            else:
                self.get_logger().info(f"Escort failed: ")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        self.clean_up(visitor_id)
        self.get_logger().info(f"Escort request completed for visitor {visitor_id}")
        self.shutdown()

    def clean_up(self, visitor_id):
        """Remove temporary files after the escort process."""
        files_to_remove = [
            f'/tmp/{visitor_id}_path.pkl_ci',
            f'/tmp/{visitor_id}_path.pkl_bi',
            f'/tmp/{visitor_id}_cihasmoved1',
            f'/tmp/{visitor_id}_cihasmoved2'
        ]
        for file_path in files_to_remove:
            if os.path.exists(file_path):
                os.remove(file_path)

    def remove_marker(self):
        """Remove the visitor marker from RViz."""
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.ns = 'visitor'
        delete_marker.id = 0  # ID of the marker to be deleted
        delete_marker.action = Marker.DELETE  # Action to delete the marker
        marker_array.markers = [delete_marker]

        self.publisher.publish(marker_array)
        self.get_logger().info("Visitor marker removed from RViz.")

    def shutdown(self):
        """Shut down the visitor agent and remove the marker."""
        self.get_logger().info("Shutting down Visitor Agent...")
        self.remove_marker()  # Remove marker from RViz before shutting down
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisitorAgent()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error during spin: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
