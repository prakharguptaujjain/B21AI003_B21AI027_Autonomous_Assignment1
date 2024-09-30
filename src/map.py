import networkx as nx

class CampusMap:
    def __init__(self):
        self.graph = nx.DiGraph()

    def build_campus(self):
        # Adding nodes (locations)
        self.graph.add_node("Entrance")
        self.graph.add_node("Building_A")
        self.graph.add_node("Building_B")
        self.graph.add_node("Building_C")

        # Adding edges (paths between locations)
        self.graph.add_edge("Entrance", "Building_A", transport="walk", time=5)
        self.graph.add_edge("Entrance", "Building_B", transport="walk", time=10)
        self.graph.add_edge("Entrance", "Building_C", transport="cycle", time=7)

    def get_shortest_path(self, start, end):
        return nx.shortest_path(self.graph, start, end)

    def print_graph(self):
        print("Campus Map Nodes:")
        print(self.graph.nodes)
        print("Campus Map Edges:")
        print(self.graph.edges(data=True))

if __name__ == "__main__":
    campus = CampusMap()
    campus.build_campus()
    campus.print_graph()
    print("Shortest path from Entrance to Building A:", campus.get_shortest_path("Entrance", "Building_A"))
