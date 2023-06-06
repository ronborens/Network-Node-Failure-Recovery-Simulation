import random
import matplotlib.pyplot as plt
import math

# Globals used to keep track of the initial graph data
previous_positions = {}     # Keeps track of the positions of nodes in the previous state
best_path = {}              # Keeps track of the best path found so far
best_path_edges = set()     # Keeps track of the edges in the best path found so far
best_path_nodes = set()     # Keeps track of the nodes in the best path found so far

class Graph:

    def __init__(self, num_nodes, additional_edge_prob=.7):
        self.nodes = list(range(num_nodes))                                             # List of nodes in the graph
        self.edges = self.generate_connected_edges(num_nodes, additional_edge_prob)     # List of edges in the graph
        self.adj_list = self.generate_adj_list()                                        # Adjacency list representation of the graph

    # Generate a list of edges to ensure connectivity and add additional edges with given probability
    def generate_connected_edges(self, num_nodes, additional_edge_prob):
        edges = []

        # Generate a tree structure to ensure connectivity
        for i in range(1, num_nodes):
            parent = random.randint(0, i - 1)
            edges.append((i, parent))

        # Add additional edges with a given probability
        for i in range(num_nodes):
            for j in range(i + 1, num_nodes):
                if random.random() < additional_edge_prob:
                    edges.append((i, j))

        return edges

    # Get the positions of nodes in the graph
    def get_node_positions(self, failure = None):
        grid_size = int(math.ceil(math.sqrt(len(self.nodes))))
        positions = {}                                              # If a failure has occurred, use the positions of nodes in the previous state
        for node in self.nodes:
            if failure:
                positions[node] = previous_positions[node]
            else:
                x = (random.randrange(0, 100) / grid_size)          # x-coordinate of the node
                y = (random.randrange(0, 100) / grid_size)          # y-coordinate of the node
                positions[node] = (x, y)
                previous_positions[node] = positions[node]
        return positions


    # Generate an adjacency list representation of the graph
    def generate_adj_list(self):
        adj_list = {node: set() for node in self.nodes}
        for u, v in self.edges:
            adj_list[u].add(v)
            adj_list[v].add(u)
        return adj_list

    # Fail nodes and edges in the graph
    def fail_nodes_and_edges(self, node_fail_prob, edge_fail_prob):
        failed_nodes = set()
        failed_edges = set()
    
        # Fail only one node and only if it is in the best path
        for node in best_path_nodes:
            if random.random() < node_fail_prob and node != self.nodes[0] and node != self.nodes[-1]:
                failed_nodes.add(node)
                break
    
        # Fail only one edge and only if it is in the best path
        for u, v in self.edges:
            if random.random() < edge_fail_prob and (u, v) in best_path_edges and not failed_nodes:
                failed_edges.add((u, v))
                break

        #If no node or edge has failed, force a failue on a random edge in the best path
        for u, v in self.edges:
            if (u, v) in best_path_edges and not failed_edges and not failed_nodes:
                failed_edges.add((u, v))
                break
    
        # Remove failed nodes and edges from the graph
        self.nodes = [node for node in self.nodes if node not in failed_nodes]
        self.edges = [edge for edge in self.edges if edge not in failed_edges]
        for node in failed_nodes:
            self.adj_list.pop(node)
        for u, v in failed_edges:
            self.adj_list[u].remove(v)
            self.adj_list[v].remove(u)
        
        return failed_nodes, failed_edges


    # Find the shortest path from source to destination
    def dijkstra_algorithm(self, source, target, failed_nodes):
        distances = {node: float('inf') for node in self.nodes if node not in failed_nodes}
        distances[source] = 0
        unvisited_nodes = set(self.nodes) - failed_nodes
        previous_nodes = {node: None for node in self.nodes if node not in failed_nodes}
    
        while unvisited_nodes:
            current_node = min(unvisited_nodes, key=lambda node: distances[node])
            unvisited_nodes.remove(current_node)
    
            if current_node == target:
                break
    
            for neighbor in self.adj_list[current_node]:
                if neighbor in failed_nodes:
                    continue
    
                new_dist = distances[current_node] + 1
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous_nodes[neighbor] = current_node
    
        # Reconstruct the best path
        best_path = []
        node = target
        while node is not None:
            best_path.append(node)
            best_path_nodes.add(node)
            node = previous_nodes[node]
        best_path.reverse()
    
        return distances[target], best_path



    # Graphs the nodes and edges 
    def display_graph(self, failed_nodes, failed_edges, title, best_path=None):
        if failed_nodes or failed_edges:
            pos = self.get_node_positions(True)
        else:
            pos = self.get_node_positions(False)
        fig, ax = plt.subplots()
    
        # Draw the edges
        for u, v in self.edges:
            if (u, v) not in failed_edges and (v, u) not in failed_edges and u not in failed_nodes and v not in failed_nodes:
                color = 'k-'
                size = 1
                if best_path is not None and ((u, v) in zip(best_path, best_path[1:]) or (v, u) in zip(best_path, best_path[1:])):
                    best_path_edges.add((u, v))
                    color = 'r-'
                    size = 3
                ax.plot([pos[u][0], pos[v][0]], [pos[u][1], pos[v][1]], color, linewidth = size)
    
        # Draw the nodes
        for node in self.nodes:
            if node not in failed_nodes:
                ax.scatter(pos[node][0], pos[node][1], s = 100, color = "b")
                if node == 0:
                    ax.annotate("Source: " + str(node), (pos[node][0], pos[node][1]), textcoords="offset points", xytext=(6, 6), ha='center')
                else:
                    if node == 9:
                        ax.annotate("Destination: " + str(node), (pos[node][0], pos[node][1]), textcoords="offset points", xytext=(6, 6), ha='center')
                    else:
                        ax.annotate(str(node), (pos[node][0], pos[node][1]), textcoords="offset points", xytext=(6, 6), ha='center')
    
        ax.set_title(title)
        
        plt.axis('off')
        plt.show()





#DRIVER CODE     
def main():
    num_nodes = 15
    node_fail_prob = 0.25
    edge_fail_prob = 0.25
    source = 0
    target = num_nodes - 1

    G = Graph(num_nodes)

    shortest_path_length, best_path = G.dijkstra_algorithm(source, target, set())

    print(f"The shortest path from node {source} to node {target} is {best_path} with a length of {shortest_path_length}.")

    # Display the initial graph
    G.display_graph(set(), set(), "Initial Graph", best_path)

    failed_nodes, failed_edges = G.fail_nodes_and_edges(node_fail_prob, edge_fail_prob)
    if failed_nodes:
        print(f"Failed node: {list(failed_nodes)[0]}")

    if failed_edges:
        print(f"Failed edge: {list(failed_edges)[0]}")

    shortest_path_length, best_path = G.dijkstra_algorithm(source, target, failed_nodes)

    print(f"The shortest path from node {source} to node {target} is {best_path} with a length of {shortest_path_length}.")

    # Display the graph after removing faulty nodes/edges and highlight the best path
    G.display_graph(failed_nodes, failed_edges, "Graph after Failures", best_path)




if __name__ == "__main__":
    main()