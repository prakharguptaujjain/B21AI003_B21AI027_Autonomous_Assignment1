import heapq  # Importing heapq for priority queue operations

class CampusMap:
    def __init__(self):
        self.__buildings = {
            'Main_Gate': (0.0, 0.0),
            'Building_A_Entrance': (5.0, 10.0),
            'Building_B_Entrance': (10.0, 15.0),
            'Building_C_Entrance': (15.0, 5.0),
            'Building_A_Room1': (6.0, 10.0),
            'Building_A_Room2': (7.0, 10.0),
            'Building_A_Room3': (8.0, 10.0),
            'Building_B_Room1': (11.0, 15.0),
            'Building_B_Room2': (12.0, 15.0),
            'Building_B_Room3': (13.0, 15.0),
            'Building_C_Room1': (16.0, 5.0),
            'Building_C_Room2': (17.0, 5.0),
            'Building_C_Room3': (18.0, 5.0),
        }

        # Paths now include transport modes as well
        self.__paths = [
            ('Main_Gate', 'Building_A_Entrance', 'walk'),
            ('Main_Gate', 'Building_A_Entrance', 'vehicle'),
            ('Main_Gate', 'Building_A_Entrance', 'cycle'),
            ('Main_Gate', 'Building_C_Entrance', 'cycle'),
            ('Main_Gate', 'Building_C_Entrance', 'vehicle'),
            ('Main_Gate', 'Building_C_Entrance', 'walk'),
            ('Building_A_Entrance', 'Building_B_Entrance', 'cycle'),
            ('Building_A_Entrance', 'Building_B_Entrance', 'vehicle'),
            ('Building_A_Entrance', 'Building_B_Entrance', 'walk'),
            ('Building_A_Entrance', 'Building_A_Room1', 'walk'),
            ('Building_A_Entrance', 'Building_A_Room2', 'walk'),
            ('Building_A_Entrance', 'Building_A_Room3', 'walk'),
            ('Building_B_Entrance', 'Building_B_Room1', 'walk'),
            ('Building_B_Entrance', 'Building_B_Room2', 'walk'),
            ('Building_B_Entrance', 'Building_B_Room3', 'walk'),
            ('Building_C_Entrance', 'Building_C_Room1', 'walk'),
            ('Building_C_Entrance', 'Building_C_Room2', 'walk'),
            ('Building_C_Entrance', 'Building_C_Room3', 'walk'),
        ]

        # Average speeds (in meters per second)
        self.__speeds = {
            'walk': 1.4,    # Average walking speed
            'cycle': 5.0,   # Average cycling speed
            'vehicle': 10.0  # Average vehicle speed
        }
        # visitor_id: [not_authorized_rooms]
        self.not_authorization = {
            1: ['Building_A_Room1', 'Building_B_Room2', 'Building_C_Room3'],
            2: ['Building_B_Room1', 'Building_B_Room2'],
            3: ['Building_C_Room2', 'Building_C_Room3']
            }



    def build_campus(self):
        self.graph = {}
        for path in self.__paths:
            start, end, mode = path
            self.__add_edge(start, end, mode)
            self.__add_edge(end, start, mode)  # Bidirectional paths

    def __add_edge(self, start, end, mode):
        if start not in self.graph:
            self.graph[start] = []
        distance = self.__calculate_distance(start, end)
        self.graph[start].append((end, distance / self.__speeds[mode]))  # Store time taken to travel

    def __calculate_distance(self, start, end):
        x1, y1 = self.__buildings[start]
        x2, y2 = self.__buildings[end]
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def shortest_path_campus(self, start, end, mode):
        if hasattr(self, '__graph') is False:
            self.build_campus()
        queue = [(0, start)]
        visited = set()
        paths = {start: []}
        min_time = {start: 0}
        
        while queue:
            current_time, current_node = heapq.heappop(queue)
            current_time = -current_time
            if current_node in visited:
                continue
            visited.add(current_node)

            for neighbor, travel_time in self.graph.get(current_node, []):
                time = current_time + travel_time

                if neighbor not in min_time or time < min_time[neighbor]:
                    min_time[neighbor] = time
                    paths[neighbor] = paths[current_node] + [current_node]
                    heapq.heappush(queue, (-time, neighbor))

        return paths.get(end, []) + [end]  # Return the path to the destination

    def get_nodes(self):
        return self.__buildings

    def get_paths(self):
        return self.__paths

    def get_position(self, building):
        return self.__buildings.get(building, None)

    def get_not_authorized_rooms(self, visitor_id):
        return self.not_authorization.get(visitor_id, [])

# def main():
#     campus_map = CampusMap()
#     campus_map.build_campus()

#     print(campus_map.shortest_path_campus('Main_Gate', 'Building_A_Room3', 'walk'))
#     print(campus_map.shortest_path_campus('Main_Gate', 'Building_A_Room3', 'cycle'))
#     print(campus_map.shortest_path_campus('Main_Gate', 'Building_A_Room3', 'vehicle'))

# if __name__ == '__main__':
#     main()