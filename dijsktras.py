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


# Given starting node (origin_key), destination node (goal_key)
# and graph it returns the shortest path from origin_key to 
# goal_key using Dijkstras Shortest Path Algorithm
def dijkstras_search(origin_key, goal_key, graph):
    
    # The priority queue / min Heap of open vertices we've reached.
    # Keys are the vertex keys, vals are the distances.
    open_queue = priority_dict.priority_dict({})
    
    # The dictionary of closed vertices we've processed.
    closed_dict = {}
    
    # The dictionary of predecessors for each vertex.
    predecessors = {}
    
    # Add the origin to the open queue.
    open_queue[origin_key] = 0.0


    goal_found = False
    
    # Iteration through the graph till open_queue is empty
    while(open_queue):
        
        curr_key,curr_dist = open_queue.pop_smallest()

        open_queue[curr_key] = curr_dist

        # If goal found
        if curr_key == goal_key:
            
            goal_found = True
            
            return get_path(origin_key, goal_key, predecessors) 
        
        # Iteration through all the reachable adjacent vertices to curr_key
        for edge in graph.out_edges([curr_key], data=True):
            
            # If next node is not visited then initialize the dist, 
            # open_queue and predecessors for it
            if edge[1] not in closed_dict.keys():

                dist = edge[2]['length'] + curr_dist
                
                open_queue[edge[1]] = dist
                
                predecessors[edge[1]] = curr_key
                    
            else:
                
                dist = edge[2]['length'] + curr_dist

                # If next node is visited then update the parameters if the distance 
                # from origin is less as compared to the previous visit
                
                if edge[1] not in closed_dict.keys() and dist < open_queue[edge[1]]:
                    
                    open_queue[edge[1]] = dist
                    
                    predecessors[edge[1]] = curr_key
                    
                if edge[1] == goal_key:
                    
                    goal_found = True
                    
                    return get_path(origin_key, goal_key, predecessors)  
                
        curr_key,curr_dist = open_queue.pop_smallest()   

        # Make the current node as visited
        closed_dict[curr_key] = curr_dist
            
    
    # If we get through entire priority queue without finding the goal,
    # something is wrong.
    if not goal_found:
        raise ValueError("Goal not found in search.")
    
    # Construct the path from the predecessors dictionary.
    return get_path(origin_key, goal_key, predecessors)


# Print the Path
path = dijkstras_search(origin, destination, map_graph)      

fig, ax = ox.plot_graph_route(map_graph, path)


