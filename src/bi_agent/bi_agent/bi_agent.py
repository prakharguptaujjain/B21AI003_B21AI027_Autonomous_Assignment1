import rclpy
from rclpy.node import Node
from agent_interfaces.srv import NavigationRequest
from campus_map.campus_map import CampusMap
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import os
import pickle
import time
import random

class BIAGent(Node):
    def __init__(self):
        super().__init__('bi_agent')
        self.get_logger().info("BI Agent is ready.")

        # Service to handle navigation requests
        self.navigation_service = self.create_service(NavigationRequest, 'navigation_request', self.handle_navigation_request)

        # Log file for communication
        self.log_file = open("/tmp/bi_agent_log.txt", "w")
        self.ooswaittime = {"A":10,"B":10,"C":10}
        self.campus_map = CampusMap()
        self.campus_map.build_campus()
        self.penalties=0
        self.lockedtill={"A":time.time(),"B":time.time(),"C":time.time()}

    def handle_navigation_request(self, request, response):
        if time.time()<self.lockedtill[request.host_location.split('_')[1]]:
            self.penalties+=1
            self.get_logger().info(f"Total Penalties till now = {self.penalties}")
            self.get_logger().warn(f"oos is locked. Please try again later. for {request.host_location}")
            response.authorized = False
            response.path = "[]"
            response.ooswaittime = self.ooswaittime[request.host_location.split('_')[1]]
            with open(f"/tmp/{request.visitor_id}_path.pkl_bi", "wb") as f:
                pickle.dump(response, f)
            return response
        
        if request.oosrequest:
            self.oosLocked = True
            self.get_logger().info(f"Received oos navigation request for visitor {request.visitor_id} to {request.host_location}")
            self.get_logger().info(f"Locking oos for {request.oosmeettime} seconds for {request.host_location}")
            oosmeettime=request.oosmeettime
            self.lockedtill[request.host_location.split('_')[1]]=max(time.time()+oosmeettime,self.lockedtill[request.host_location.split('_')[1]])
            self.oosLocked = False
            response.authorized = True
            response.path = "[]"
            response.ooswaittime = 0
            with open(f"/tmp/{request.visitor_id}_path.pkl_bi", "wb") as f:
                pickle.dump(response, f)
            return response

        visitor_id = request.visitor_id
        host_location = request.host_location

        not_authorized_rooms = self.campus_map.get_not_authorized_rooms(int(visitor_id.split('_')[1]))

        self.get_logger().info(f"Received navigation request for visitor {visitor_id} to {host_location}")

        # Log the navigation request
        self.log(f"Navigation request from {visitor_id} to {host_location}")

        # Authorize access to certain locations
        response.ooswaittime = 0
        if host_location not in not_authorized_rooms:
            response.authorized = True
            response.path = str(self.get_internal_path(host_location))  
            response.ooswaittime = 0
            self.get_logger().info(f"Access granted to visitor {visitor_id} for {host_location}, Path: {response.path}")
        else:
            response.authorized = False
            response.path = "[]"
            self.get_logger().warn(f"Access denied for visitor {visitor_id} to {host_location}")

        with open(f"/tmp/{visitor_id}_path.pkl_bi", "wb") as f:
            pickle.dump(response, f)

        self.get_logger().info(f"Created pickle at{'/tmp/{visitor_id}_path.pkl_bi'}")

        return response

    def is_access_granted(self, host_location):
        # Implement logic to determine if access is granted
        authorized_rooms = [
            'Building_A_HostOffice',
            'Building_B_HostOffice',
            'Building_C_HostOffice'
        ]
        return host_location in authorized_rooms

    def get_internal_path(self, host_location):
        # Implement logic to determine the internal path
        # host_location = 'Building_A_Room1
        building_number=host_location.split('_')[1]
        room=host_location.split('_')[-1]
        room_number=int(room[-1])

        # dijkstra's algorithm
        # queue = [(0, f'Building_{building_number}_Entrance')]
        # visited = set()
        # paths = {f'Building_{building_number}_Entrance': []}
        # min_time = {f'Building_{building_number}_Entrance': 0}

        # while queue:
        #     current_time, current_node = heapq.heappop(queue)

        #     if current_node in visited:
        #         continue
        #     visited.add(current_node)

        #     for neighbor, travel_time in self.graph.get(current_node, []):
        #         time = current_time + travel_time

        #         if neighbor not in min_time or time < min_time[neighbor]:
        #             min_time[neighbor] = time
        #             paths[neighbor] = paths[current_node] + [neighbor]
        #             heapq.heappush(queue, (time, neighbor))
        
        path = [f'Building_{building_number}_Entrance']
        for i in range(1, room_number+1):
            path.append(f'Building_{building_number}_Room{i}')
        return path

    def log(self, message):
        self.log_file.write(f"{message}\n")
        self.log_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = BIAGent()
    rclpy.spin(node)
    node.log_file.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
