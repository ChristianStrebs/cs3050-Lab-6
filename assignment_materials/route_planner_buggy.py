#!/usr/bin/env python3
"""
BUGGY VERSION - For Lab 6 : Bug Hunt Exercise
This implementation contains a subtle bug that produces incorrect results
under specific graph configurations. Students must find and fix it.
"""

import sys
import csv
import math
from typing import Dict, List, Tuple, Optional

EARTH_RADIUS = 6371.0

class Node:
    def __init__(self, node_id: int, lat: float, lon: float):
        self.id = node_id
        self.lat = lat
        self.lon = lon

class Edge:
    def __init__(self, to: int, weight: float):
        self.to = to
        self.weight = weight

class Graph:
    def __init__(self):
        self.nodes: Dict[int, Node] = {}
        self.adj_list: Dict[int, List[Edge]] = {}
    
    def add_node(self, node_id: int, lat: float, lon: float):
        self.nodes[node_id] = Node(node_id, lat, lon)
        if node_id not in self.adj_list:
            self.adj_list[node_id] = []
    
    def add_edge(self, from_id: int, to_id: int, weight: float):
        if from_id not in self.adj_list:
            self.adj_list[from_id] = []
        self.adj_list[from_id].append(Edge(to_id, weight))

def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return EARTH_RADIUS * c

def bellman_ford_buggy(graph: Graph, start: int, end: int) -> Tuple[Optional[Dict[int, float]], Optional[Dict[int, Optional[int]]], int]:
    dist = {node_id: float('inf') for node_id in graph.nodes}
    prev = {node_id: None for node_id in graph.nodes}
    dist[start] = 0
    
    nodes_explored = 0
    node_count = len(graph.nodes)
    
    # Relax edges |V| - 1 times
    for i in range(node_count - 1):
        updated_this_iteration = False
        consecutive_no_updates = 0 
        
        for u in graph.nodes:
            if dist[u] == float('inf'):
                continue
            
            node_updated = False
            for edge in graph.adj_list.get(u, []):
                v = edge.to
                if dist[u] + edge.weight < dist[v]:
                    dist[v] = dist[u] + edge.weight
                    prev[v] = u
                    updated_this_iteration = True
                    node_updated = True
            
            if not node_updated:
                consecutive_no_updates += 1
            else:
                consecutive_no_updates = 0
            
            if consecutive_no_updates >= 3:
                break
        
        nodes_explored += 1
        
        if not updated_this_iteration:
            break
    
    for u in graph.nodes:
        if dist[u] == float('inf'):
            continue
        
        for edge in graph.adj_list.get(u, []):
            v = edge.to
            if dist[u] + edge.weight < dist[v]:
                return None, None, 0
    
    return dist, prev, nodes_explored

def reconstruct_path(prev: Dict[int, Optional[int]], start: int, end: int) -> Optional[List[int]]:
    if prev[end] is None and start != end:
        return None
    
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = prev[current]
    
    path.reverse()
    return path

def print_path(graph: Graph, prev: Dict[int, Optional[int]], start: int, end: int, distance: float):
    path = reconstruct_path(prev, start, end)
    
    if path is None:
        print("No path found")
        return
    
    path_str = " -> ".join(str(node) for node in path)
    print(f"Path from {start} to {end}: {path_str}")
    print(f"Total distance: {distance:.2f} km")

def load_graph(nodes_file: str, edges_file: str) -> Graph:
    graph = Graph()
    
    with open(nodes_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            node_id = int(row['id'])
            lat = float(row['lat'])
            lon = float(row['lon'])
            graph.add_node(node_id, lat, lon)
    
    with open(edges_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            from_id = int(row['from'])
            to_id = int(row['to'])
            distance = float(row['distance'])
            graph.add_edge(from_id, to_id, distance)
    
    return graph

def main():
    if len(sys.argv) != 5:
        print(f"Usage: {sys.argv[0]} <nodes.csv> <edges.csv> <start_node> <end_node>")
        sys.exit(1)
    
    nodes_file = sys.argv[1]
    edges_file = sys.argv[2]
    start_node = int(sys.argv[3])
    end_node = int(sys.argv[4])
    
    graph = load_graph(nodes_file, edges_file)
    
    if start_node not in graph.nodes or end_node not in graph.nodes:
        print("Invalid start or end node")
        sys.exit(1)
    
    print("=== Bellman-Ford Algorithm (BUGGY VERSION) ===")
    dist, prev, nodes_explored = bellman_ford_buggy(graph, start_node, end_node)
    
    if dist is None:
        print("Negative cycle detected!")
        sys.exit(1)
    
    print_path(graph, prev, start_node, end_node, dist[end_node])
    print(f"Nodes explored: {nodes_explored}")

if __name__ == "__main__":
    main()