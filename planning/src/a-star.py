import osmnx as ox
import networkx as nx
import queue
import math
import priority_dict

map_graph = ox.graph_from_place('Berkeley, California', network_type='drive')
origin = ox.get_nearest_node(map_graph, (37.8743, -122.277))
destination = list(map_graph.nodes())[-1]


# Ground Truth
shortest_path = nx.shortest_path(map_graph, origin, destination, weight='length')

fig, ax = ox.plot_graph_route(map_graph, shortest_path)


# This function generates a path starting from origin_key
# to goal_key based on the predecessors 
def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]
    
    while (key != origin_key):
        key = predecessors[key]
        path.insert(0, key)
        
    return path


# Given current node (state_node), goal node (goal_key)
# and graph data this function returns the Heuristic 
# from current node to goal node

# Here Heuristic is found by computing the linear 
# distance between current node and goal node 
def distance_heuristic(state_key, goal_key, node_data):
    n1 = node_data[state_key]
    n2 = node_data[goal_key]

    # Get the longitude and latitude for each vertex.
    long1 = n1['x']*math.pi/180.0
    lat1 = n1['y']*math.pi/180.0
    long2 = n2['x']*math.pi/180.0
    lat2 = n2['y']*math.pi/180.0
    
    # Use a spherical approximation of the earth for
    # estimating the distance between two points.
    r = 6371000
    x1 = r*math.cos(lat1)*math.cos(long1)
    y1 = r*math.cos(lat1)*math.sin(long1)
    z1 = r*math.sin(lat1)

    x2 = r*math.cos(lat2)*math.cos(long2)
    y2 = r*math.cos(lat2)*math.sin(long2)
    z2 = r*math.sin(lat2)

    d = ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**0.5
    
    return d

# Given starting node (origin_key), destination node (goal_key)
# and graph it returns the shortest path from origin_key to 
# goal_key using A* Shortest Path Algorithm
def a_star_search(origin_key, goal_key, graph):
    # The priority queue/ min Heap of open vertices we've reached.
    # Keys are the vertex keys, vals are the accumulated
    # distances plus the heuristic estimates of the distance
    # to go.
    open_queue = priority_dict.priority_dict({})
    
    # The dictionary of closed vertices we've processed.
    closed_dict = {}
    
    # The dictionary of predecessors for each vertex.
    predecessors = {}
    
    # The dictionary that stores the best cost to reach each
    # vertex found so far.
    costs = {}
    
    # Get the spatial data for each vertex as a dictionary.
    node_data = graph.nodes(True)
    
    # Add the origin to the open queue and the costs dictionary.
    costs[origin_key] = 0.0
    open_queue[origin_key] = costs[origin_key] + distance_heuristic(origin_key, goal_key, node_data)

    goal_found = False
    

    # Iterating through the  graph until the open_queue is empty
    while(open_queue):
        
        curr_key,curr_dist = open_queue.pop_smallest()
        
        open_queue[curr_key] = curr_dist 
        
        if curr_key == goal_key:

            goal_found = True

            return get_path(origin_key, goal_key, predecessors) 
        
        
        for edge in graph.out_edges([curr_key], data=True):
            
            if edge[1] not in closed_dict.keys():
                
                open_queue[edge[1]] = edge[2]['length'] + costs[curr_key] + distance_heuristic(edge[1],goal_key,node_data)
                
                predecessors[edge[1]] = curr_key
                
                costs[edge[1]] = costs[curr_key] + edge[2]['length']
                    
            else:
                
                if edge[1] not in costs.keys():
                    
                    costs[edge[1]] = costs[curr_key] + edge[2]['length']

                
                if (edge[1] not in closed_dict.keys() and  
                    costs[edge[1]] + distance_heuristic(curr_key,goal_key,node_data) < open_queue[edge[1]]):
                    
                    open_queue[edge[1]] = costs[edge[1]] + distance_heuristic(curr_key,goal_key,node_data)
                    
                    predecessors[edge[1]] = curr_key
                    
                    costs[edge[1]] = costs[curr_key] + edge[2]['length'] 
                
        curr_key,curr_dist = open_queue.pop_smallest()  
        
        closed_dict[curr_key] = curr_dist
                   

    if not goal_found:
        raise ValueError("Goal not found in search.")
    
    # Construct the path from the predecessors dictionary.
    return get_path(origin_key, goal_key, predecessors)


path = a_star_search(origin, destination, map_graph)        
fig, ax = ox.plot_graph_route(map_graph, path)