import sys
from collections import defaultdict

class Graph(object):
    def __init__(self, nodes, adjacent_lists):
        self.nodes = nodes
        self.graph = self.construct_graph(nodes, adjacent_lists)
        self.links_dic = adjacent_lists
        
    def construct_graph(self, nodes, adjacent_lists):
        '''
        This method makes sure that the graph is symmetrical. In other words, if there's a path from node A to B with a value V, there needs to be a path from node B to node A with a value V.
        '''
        graph = defaultdict(dict)

        '''
        for node in nodes:
            node=tuple(node)
            adjacency_list = dict(adjacent_lists[node]).keys()
            for i in range(0,len(adjacency_list)):
                graph[node][tuple(adjacency_list[i])] = adjacent_lists[node][i][1]
        '''

        for node in nodes:
            node=tuple(node)
            for i in range(0,len(adjacent_lists[node])):
                graph[node][tuple(adjacent_lists[node][i][0])] = adjacent_lists[node][i][1]

        '''
        graph.update(adjacent_lists)
        
        for node, edges in graph.items():
            for adjacent_node, value in edges.items():
                if graph[adjacent_node].get(node, False) == False:
                    graph[adjacent_node][node] = value
        '''

        return graph
    
    def get_nodes(self):
        "Returns the nodes of the graph."
        return self.nodes
    
    def get_outgoing_edges(self, node, adjacent_lists):

        node=tuple(node)
        connections_list = dict(adjacent_lists[node]).keys()
        return connections_list

        "Returns the neighbors of a node."
        connections = []
        for out_node in self.nodes:
            if self.graph[node].get(out_node, False) != False:
                connections.append(out_node)
        return connections
    
    def value(self, node1, node2):
        node1=tuple(node1)
        bode2=tuple(node2)
        "Returns the value of an edge between two nodes."
        return self.graph[node1][node2]

def dijkstra_algorithm(graph, start_node):
    start_node=tuple(start_node)
    unvisited_nodes = list(graph.get_nodes())
 
    # We'll use this dict to save the cost of visiting each node and update it as we move along the graph   
    shortest_path = {}
 
    # We'll use this dict to save the shortest known path to a node found so far
    previous_nodes = {}
 
    # We'll use max_value to initialize the "infinity" value of the unvisited nodes   
    max_value = sys.maxsize
    for node in unvisited_nodes:
        node=tuple(node)
        shortest_path[node] = max_value
    # However, we initialize the starting node's value with 0   
    shortest_path[start_node] = 0
    
    # The algorithm executes until we visit all nodes
    while unvisited_nodes:
        # The code block below finds the node with the lowest score
        current_min_node = None
        for node in unvisited_nodes: # Iterate over the nodes
            node=tuple(node)
            if current_min_node == None:
                current_min_node = node
            elif shortest_path[node] < shortest_path[current_min_node]:
                current_min_node = node
                
        # The code block below retrieves the current node's neighbors and updates their distances
        neighbors = graph.get_outgoing_edges(current_min_node, graph.links_dic)
        for neighbor in neighbors:
            neighbor=tuple(neighbor)
            tentative_value = shortest_path[current_min_node] + graph.value(current_min_node, neighbor)
            if tentative_value < shortest_path[neighbor]:
                shortest_path[neighbor] = tentative_value
                # We also update the best path to the current node
                previous_nodes[neighbor] = current_min_node
 
        # After visiting its neighbors, we mark the node as "visited"
        unvisited_nodes.remove(list(current_min_node))
    
    return previous_nodes, shortest_path