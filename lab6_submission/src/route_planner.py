#!/usr/bin/env python3
"""
CS3050 Lab 6: Advanced Route Planning - Time-Window Constrained Routing
Student Implementation Template

This file contains the structure for implementing time-window constrained routing
and priority-based multi-destination routing.

Author: Christian
Date: 2025-12-06
"""

import sys
import csv
import heapq
from typing import List, Dict, Tuple, Optional, Set
from dataclasses import dataclass
from math import radians, cos, sin, asin, sqrt


@dataclass
class Node:
    """Represents a node in the graph with optional time window constraints."""
    id: int
    lat: float
    lon: float
    earliest: float = 0.0  # Earliest allowable arrival time
    latest: float = float('inf')  # Latest allowable arrival time


@dataclass
class Edge:
    """Represents a directed edge in the graph."""
    from_node: int
    to_node: int
    distance: float


@dataclass
class State:
    """
    Represents a state in the search space.

    TODO: For time-window routing, you need to track more than just the node!
    Think about: What makes two visits to the same node different?

    Hint: Consider (node_id, arrival_time) as your state.
    """
    node_id: int
    arrival_time: float = 0.0
    distance: float = 0.0

    def __lt__(self, other):
        """For priority queue comparison."""
        return self.distance < other.distance


class Graph:
    """Graph representation for route planning."""

    def __init__(self):
        self.nodes: Dict[int, Node] = {}
        self.edges: Dict[int, List[Edge]] = {}  # Adjacency list

    def add_node(self, node: Node):
        """Add a node to the graph."""
        self.nodes[node.id] = node
        if node.id not in self.edges:
            self.edges[node.id] = []

    def add_edge(self, edge: Edge):
        """Add an edge to the graph."""
        if edge.from_node not in self.edges:
            self.edges[edge.from_node] = []
        self.edges[edge.from_node].append(edge)

    def get_neighbors(self, node_id: int) -> List[Edge]:
        """Get all outgoing edges from a node."""
        return self.edges.get(node_id, [])


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the great circle distance between two points on earth.
    Returns distance in kilometers.
    """
    # Convert to radians
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))

    # Radius of earth in kilometers
    r = 6371
    return c * r


def load_graph(nodes_file: str, edges_file: str) -> Graph:
    """
    Load graph from CSV files.

    Nodes file format: id,lat,lon[,earliest,latest]
    Edges file format: from,to,distance
    """
    graph = Graph()

    # Load nodes
    with open(nodes_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            node = Node(
                id=int(row['id']),
                lat=float(row['lat']),
                lon=float(row['lon']),
                earliest=float(row.get('earliest', 0.0)),
                latest=float(row.get('latest', float('inf')))
            )
            graph.add_node(node)

    # Load edges
    with open(edges_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            edge = Edge(
                from_node=int(row['from']),
                to_node=int(row['to']),
                distance=float(row['distance'])
            )
            graph.add_edge(edge)

    return graph


def dijkstra_with_time_windows(graph: Graph, start: int, end: int) -> Tuple[Optional[List[int]], float, int]:
    """
    Modified Dijkstra's algorithm to handle time window constraints.

    Returns: (path, total_distance, nodes_explored)

    Key implementation details:
    - State space: (node_id, arrival_time) instead of just node_id
    - Pruning: Reject states where arrival_time > latest[node]
    - Early arrival: Wait at node until earliest[node] if needed
    - Visited tracking: Track best arrival time per node to avoid reprocessing dominated states

    Implementation notes:
    The state space thing was tricky. Initially tried just tracking visited nodes like
    regular Dijkstra but that broke everything. Realized you need (node, time) pairs
    because arriving at the same node at different times can lead to different outcomes.

    The pruning is super important - without tracking best_arrival times, the algorithm
    was timing out even on small graphs. Basically if you've already gotten to a node
    earlier, there's no point exploring a path that gets there later (since you can
    always wait but can't go back in time).
    """

    # Priority queue: (distance, node_id, arrival_time)
    pq = []
    # Track best (earliest feasible) arrival time at each node
    # This pruning prevents state space explosion - was a huge performance issue before adding this
    best_arrival = {}
    # Parent tracking for path reconstruction: (node_id, arrival_time) -> (parent_node, parent_time)
    # Need to track both node AND time for proper reconstruction
    parent = {}
    nodes_explored = 0

    # Check start node constraints
    start_node = graph.nodes[start]
    if 0 > start_node.latest:
        return None, 0.0, 0

    # Adjust start time if needed
    start_time = max(0, start_node.earliest)

    # Initialize with start state
    heapq.heappush(pq, (0, start, start_time))
    parent[(start, start_time)] = None

    final_state = None

    while pq:
        current_dist, current_node, arrival_time = heapq.heappop(pq)

        # Skip if we've already found a better path to this node
        if current_node in best_arrival and arrival_time > best_arrival[current_node]:
            continue

        # Mark as explored
        nodes_explored += 1
        best_arrival[current_node] = arrival_time

        # Check if we've reached the goal
        if current_node == end:
            final_state = (current_node, arrival_time)
            break

        # Explore neighbors
        for edge in graph.get_neighbors(current_node):
            next_node = edge.to_node
            next_dist = current_dist + edge.distance
            # Assuming travel time equals distance (1 km = 1 time unit)
            next_arrival = arrival_time + edge.distance

            # Get next node's time window
            next_node_obj = graph.nodes[next_node]

            # Prune if arrival is too late
            if next_arrival > next_node_obj.latest:
                continue

            # Wait if arrival is too early
            # This took a while to get right - need to use actual_arrival (after waiting)
            # in the parent dict, not next_arrival. Had a bug where I was mixing these up
            # and the path reconstruction was totally broken
            actual_arrival = max(next_arrival, next_node_obj.earliest)

            # Skip if we've already visited this node with better arrival time
            if next_node in best_arrival and actual_arrival >= best_arrival[next_node]:
                continue

            # Add to queue
            heapq.heappush(pq, (next_dist, next_node, actual_arrival))
            parent[(next_node, actual_arrival)] = (current_node, arrival_time)

    # Reconstruct path if found
    if final_state is None:
        return None, 0.0, nodes_explored

    path = []
    current_state = final_state
    total_distance = 0.0

    while current_state is not None:
        path.append(current_state[0])
        parent_state = parent[current_state]

        if parent_state is not None:
            # Calculate distance of this edge
            for edge in graph.get_neighbors(parent_state[0]):
                if edge.to_node == current_state[0]:
                    total_distance += edge.distance
                    break

        current_state = parent_state

    path.reverse()
    return path, total_distance, nodes_explored


def priority_multi_destination_routing(
    graph: Graph,
    start: int,
    destinations: Dict[int, str],  # {node_id: priority_level}
    threshold: float = 0.2
) -> Tuple[List[int], float, List[str]]:
    """
    Find a route visiting multiple destinations with priority constraints.

    Args:
        graph: The graph
        start: Starting node
        destinations: Dict mapping node_id -> priority ("HIGH", "MEDIUM", "LOW")
        threshold: Allow path to be this much longer to maintain priority order

    Returns: (route, total_distance, priority_violations)

    Implementation approach:
    1. Greedy algorithm with priority-first ordering
    2. Within each priority level, visit nearest unvisited node
    3. Track violations when a lower priority is visited before higher priority
    4. Threshold allows some flexibility in distance vs priority trade-offs

    Using greedy nearest-neighbor within priority levels. Visit ALL high priority
    nodes before touching any medium priority ones, etc. The threshold parameter
    isn't really being used right now - just there as a placeholder for later.
    """

    def dijkstra_shortest_path(from_node: int, to_node: int) -> Tuple[Optional[List[int]], float]:
        """Helper: Find shortest path between two nodes.

        Just runs regular Dijkstra for each segment. Could've made this a separate
        function but keeping it here as a helper since it's only used in this context.
        """
        pq = []
        distances = {node_id: float('inf') for node_id in graph.nodes}
        parent = {}
        distances[from_node] = 0
        heapq.heappush(pq, (0, from_node))

        while pq:
            current_dist, current_node = heapq.heappop(pq)

            if current_dist > distances[current_node]:
                continue

            if current_node == to_node:
                break

            for edge in graph.get_neighbors(current_node):
                next_node = edge.to_node
                next_dist = current_dist + edge.distance

                if next_dist < distances[next_node]:
                    distances[next_node] = next_dist
                    parent[next_node] = current_node
                    heapq.heappush(pq, (next_dist, next_node))

        if distances[to_node] == float('inf'):
            return None, float('inf')

        # Reconstruct path
        path = []
        current = to_node
        while current != from_node:
            path.append(current)
            current = parent[current]
        path.reverse()

        return path, distances[to_node]

    # Group destinations by priority
    high_priority = [node for node, pri in destinations.items() if pri == "HIGH"]
    medium_priority = [node for node, pri in destinations.items() if pri == "MEDIUM"]
    low_priority = [node for node, pri in destinations.items() if pri == "LOW"]

    route = [start]
    total_distance = 0.0
    violations = []
    current_location = start
    visited = set()

    # Visit destinations in priority order
    # Strict ordering: ALL high, then ALL medium, then ALL low
    for priority_level, priority_group in [("HIGH", high_priority),
                                            ("MEDIUM", medium_priority),
                                            ("LOW", low_priority)]:
        # Visit all nodes in this priority level
        while priority_group:
            # Find nearest unvisited node in this priority group
            # This is the greedy part - just pick whichever one is closest right now
            best_node = None
            best_distance = float('inf')
            best_path = None

            for dest_node in priority_group:
                if dest_node not in visited:
                    path, dist = dijkstra_shortest_path(current_location, dest_node)
                    if path is not None and dist < best_distance:
                        best_node = dest_node
                        best_distance = dist
                        best_path = path

            if best_node is None:
                # No reachable nodes in this priority level
                violations.append(f"Could not reach all {priority_level} priority destinations")
                break

            # Add this segment to route
            route.extend(best_path)
            total_distance += best_distance
            current_location = best_node
            visited.add(best_node)
            # Had a bug here where I was modifying the list while iterating
            # Changed to remove() after finding the best one
            priority_group.remove(best_node)

    # Check for priority violations (lower priority visited before higher)
    # This is mainly for debugging - the greedy approach shouldn't create violations
    # but it's good to verify
    priority_order = {"HIGH": 0, "MEDIUM": 1, "LOW": 2}
    last_priority = -1

    for i in range(1, len(route)):  # Skip start node
        node = route[i]
        if node in destinations:
            current_priority_level = priority_order[destinations[node]]
            if current_priority_level < last_priority:
                violations.append(
                    f"Priority violation at node {node}: "
                    f"{destinations[node]} visited after lower priority"
                )
            last_priority = max(last_priority, current_priority_level)

    return route, total_distance, violations


def astar_with_time_windows(graph: Graph, start: int, end: int) -> Tuple[Optional[List[int]], float, int]:
    """
    A* algorithm with time window constraints and haversine heuristic.

    TODO: Optional - implement if you chose A* as your base algorithm.

    The heuristic h(n) = haversine_distance(n, goal) is admissible because:
    - It never overestimates the actual distance
    - Straight-line distance is always <= actual path distance

    Key consideration: How do time windows affect the heuristic?
    """

    print("ERROR: astar_with_time_windows not implemented!")
    return None, 0.0, 0


def find_infeasible_constraints(graph: Graph, start: int, end: int) -> List[str]:
    """
    When no feasible path exists, identify which constraints were violated.

    Approach:
    1. Run shortest path WITHOUT time constraints (standard Dijkstra)
    2. Simulate traversal and check which nodes violate their time windows
    3. Report specific violations

    This is way more helpful than just saying "nope can't do it" - it tells you
    exactly which nodes break the rules and by how much.
    """

    violations = []

    # Run standard Dijkstra without time constraints
    # Basically just copy-pasted regular Dijkstra and removed the time window checks
    pq = []
    distances = {node_id: float('inf') for node_id in graph.nodes}
    parent = {}
    distances[start] = 0
    heapq.heappush(pq, (0, start))

    while pq:
        current_dist, current_node = heapq.heappop(pq)

        if current_dist > distances[current_node]:
            continue

        for edge in graph.get_neighbors(current_node):
            next_node = edge.to_node
            next_dist = current_dist + edge.distance

            if next_dist < distances[next_node]:
                distances[next_node] = next_dist
                parent[next_node] = current_node
                heapq.heappush(pq, (next_dist, next_node))

    # Check if path exists at all
    if distances[end] == float('inf'):
        violations.append("No path exists between start and end nodes (connectivity issue)")
        return violations

    # Reconstruct shortest path
    path = []
    current = end
    while current != start:
        if current not in parent:
            violations.append("Path reconstruction failed")
            return violations
        path.append(current)
        current = parent[current]
    path.append(start)
    path.reverse()

    # Simulate traversal and check time window violations
    current_time = max(0, graph.nodes[start].earliest)

    for i in range(len(path)):
        node_id = path[i]
        node = graph.nodes[node_id]

        if i > 0:
            # Add travel time from previous node
            prev_node = path[i - 1]
            for edge in graph.get_neighbors(prev_node):
                if edge.to_node == node_id:
                    current_time += edge.distance
                    break

        # Check time window
        if current_time < node.earliest:
            violations.append(
                f"Node {node_id}: arrives at {current_time:.2f}, "
                f"but earliest allowed is {node.earliest:.2f} (too early by {node.earliest - current_time:.2f})"
            )
            current_time = node.earliest  # Wait

        if current_time > node.latest:
            violations.append(
                f"Node {node_id}: arrives at {current_time:.2f}, "
                f"but latest allowed is {node.latest:.2f} (too late by {current_time - node.latest:.2f})"
            )

    if not violations:
        violations.append("No specific constraint violations found, but no feasible path exists")

    return violations


def suggest_closest_path(graph: Graph, start: int, end: int) -> Tuple[List[int], float, List[str]]:
    """
    Find the path that minimizes constraint violations.

    Approach:
    1. Return the shortest distance path (ignoring time constraints)
    2. Calculate and report which constraints it violates
    3. This gives users the "closest" option when no feasible path exists

    Figured this would be more useful than just failing - at least shows what the
    best option is even if it doesn't perfectly work.
    """

    # Run standard Dijkstra to get shortest path
    pq = []
    distances = {node_id: float('inf') for node_id in graph.nodes}
    parent = {}
    distances[start] = 0
    heapq.heappush(pq, (0, start))

    while pq:
        current_dist, current_node = heapq.heappop(pq)

        if current_dist > distances[current_node]:
            continue

        if current_node == end:
            break

        for edge in graph.get_neighbors(current_node):
            next_node = edge.to_node
            next_dist = current_dist + edge.distance

            if next_dist < distances[next_node]:
                distances[next_node] = next_dist
                parent[next_node] = current_node
                heapq.heappush(pq, (next_dist, next_node))

    # Reconstruct path
    if distances[end] == float('inf'):
        return [], 0.0, ["No path exists between nodes"]

    path = []
    current = end
    while current != start:
        path.append(current)
        current = parent[current]
    path.append(start)
    path.reverse()

    total_distance = distances[end]

    # Check violations along this path
    violations = []
    current_time = max(0, graph.nodes[start].earliest)

    for i in range(len(path)):
        node_id = path[i]
        node = graph.nodes[node_id]

        if i > 0:
            # Add travel time
            prev_node = path[i - 1]
            for edge in graph.get_neighbors(prev_node):
                if edge.to_node == node_id:
                    current_time += edge.distance
                    break

        # Check violations
        if current_time < node.earliest:
            violations.append(f"Node {node_id}: too early by {node.earliest - current_time:.2f}")
            current_time = node.earliest
        elif current_time > node.latest:
            violations.append(f"Node {node_id}: too late by {current_time - node.latest:.2f}")

    if not violations:
        violations = ["No violations (this shouldn't happen if called after infeasible detection)"]

    return path, total_distance, violations


def print_results(path: Optional[List[int]], distance: float, nodes_explored: int, algorithm: str):
    """Print routing results in a formatted way."""

    print(f"\n=== {algorithm} ===")

    if path is None:
        print("No feasible path found satisfying time constraints")
        return

    print(f"Path: {' -> '.join(map(str, path))}")
    print(f"Total distance: {distance:.2f} km")
    print(f"Nodes explored: {nodes_explored}")


def main():
    """Main program entry point."""

    if len(sys.argv) < 5:
        print("Usage: python route_planner.py <nodes.csv> <edges.csv> <start> <end> [algorithm]")
        print("  algorithm: dijkstra (default), astar")
        sys.exit(1)

    nodes_file = sys.argv[1]
    edges_file = sys.argv[2]
    start = int(sys.argv[3])
    end = int(sys.argv[4])
    algorithm = sys.argv[5] if len(sys.argv) > 5 else "dijkstra"

    # Load graph
    print(f"Loading graph from {nodes_file} and {edges_file}...")
    graph = load_graph(nodes_file, edges_file)
    print(f"Loaded {len(graph.nodes)} nodes and {sum(len(edges) for edges in graph.edges.values())} edges")

    # Run algorithm
    if algorithm == "dijkstra":
        path, distance, explored = dijkstra_with_time_windows(graph, start, end)
        print_results(path, distance, explored, "Dijkstra with Time Windows")
    elif algorithm == "astar":
        path, distance, explored = astar_with_time_windows(graph, start, end)
        print_results(path, distance, explored, "A* with Time Windows")
    else:
        print(f"Unknown algorithm: {algorithm}")
        sys.exit(1)

    # If no path found, suggest alternatives
    if path is None:
        violations = find_infeasible_constraints(graph, start, end)
        print("\nConstraint violations:")
        for v in violations:
            print(f"  - {v}")

        print("\nSuggesting closest path...")
        closest_path, closest_dist, closest_violations = suggest_closest_path(graph, start, end)
        print(f"Closest path: {' -> '.join(map(str, closest_path))}")
        print(f"Distance: {closest_dist:.2f} km")
        print(f"Violations: {', '.join(closest_violations)}")


if __name__ == "__main__":
    main()
