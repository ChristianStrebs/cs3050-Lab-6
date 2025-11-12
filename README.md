# Route Planning and Navigation Lab

This lab guides students through building a basic route planning engine using shortest path algorithms like Dijkstra and A*. The lab is implemented in **C**, **Python**, and **Go** to show different implementation approaches side by side.

## Lab Overview

- **Bellman-Ford Algorithm**: Can handle negative edge weights and detect negative cycles
- **Algorithm Selection**: Choose between `dijkstra`, `astar`, or `bellman-ford` via command line
- **Unified Interface**: Consistent command-line interface across all three implementations

Students will:
- Load OpenStreetMap-style graph data from CSV files
- Implement Dijkstra's algorithm for finding shortest paths
- Implement A* algorithm for more efficient pathfinding
- Simulate traffic-aware routing by modifying edge weights
- Visualize or print routing results

## Repository Structure

```
cs3050-Lab-6/
├── assignment_materials/
│   ├── ASSIGNMENT.md        # Full Assignment
│   └── BUG_TEST_DATA.md        # Test data for debugging
│   ├── route_planner_buggy.py        # A route planner with a bug
│   └── STUDENT_HINTS.md        # Hints
├── data/
│   ├── nodes.csv        # Node coordinates (id, lat, lon)
│   └── edges.csv        # Edge connections (from, to, distance)
├── c/
│   ├── route_planner.c  # C implementation
│   ├── route_planner.h  # Header file
│   └── Makefile         # Build configuration
├── python/
│   └── route_planner.py # Python implementation
├── go/
│   ├── route_planner.go # Go implementation
│   └── go.mod           # Go module file
└── README.md
```

**Link to Assignment at the bottom**

## Getting Started: Familiarize Yourself

There are instructions for running each program, in Go, Python, and C. Each one has its own **"make file"** named `Makefile`, as one does. Examine the makefiles for patterns of how they are used to simplify compilation processes in a number of languages. 

### Data Format

#### nodes.csv
```csv
id,lat,lon
1,38.9072,-77.0369
2,38.9100,-77.0400
3,38.9050,-77.0300
```

#### edges.csv
```csv
from,to,distance
1,2,2.5
2,3,3.0
1,3,4.0
```

## Algorithm Comparison

| Algorithm | Time Complexity | Negative Weights | Use Case |
|-----------|----------------|------------------|----------|
| **Dijkstra** | O((V + E) log V) | ❌ No | Fast, general-purpose routing |
| **A*** | O(E log V) | ❌ No | Geographic routing with heuristic |
| **Bellman-Ford** | O(V × E) | ✅ Yes | Graphs with negative weights, cycle detection |

### When to Use Each Algorithm

- **Dijkstra**: Best for general routing with non-negative weights
- **A***: Most efficient for geographic routing (uses haversine distance heuristic)
- **Bellman-Ford**: When you need to handle negative weights or detect negative cycles

## Usage

All implementations require 5 command-line arguments:

```bash
<program> <nodes.csv> <edges.csv> <start_node> <end_node> <algorithm>
```

### C Implementation

**Build:**
```bash
cd c
make
```

**Run:**
```bash
# Using Dijkstra
./route_planner ../data/nodes.csv ../data/edges.csv 1 3 dijkstra

# Using A*
./route_planner ../data/nodes.csv ../data/edges.csv 1 3 astar

# Using Bellman-Ford
./route_planner ../data/nodes.csv ../data/edges.csv 1 3 bellman-ford
```

**Clean:**
```bash
make clean
```

### Python Implementation

**Run:**
```bash
cd python

# Using Dijkstra
python3 route_planner.py ../data/nodes.csv ../data/edges.csv 1 3 dijkstra

# Using A*
python3 route_planner.py ../data/nodes.csv ../data/edges.csv 1 3 astar

# Using Bellman-Ford
python3 route_planner.py ../data/nodes.csv ../data/edges.csv 1 3 bellman-ford
```

Or ,`make run` will run all three. 

### Go Implementation

**Run:**
```bash
cd go

# Using Dijkstra
go run route_planner.go ../data/nodes.csv ../data/edges.csv 1 3 dijkstra

# Using A*
go run route_planner.go ../data/nodes.csv ../data/edges.csv 1 3 astar

# Using Bellman-Ford
go run route_planner.go ../data/nodes.csv ../data/edges.csv 1 3 bellman-ford
```

Or, `make run` will run all three. 


##  Algorithms Implemented

### Dijkstra's Algorithm
- Finds shortest path in weighted graphs
- Guarantees optimal solution
- Works well for general routing problems
- Time complexity: O((V + E) log V)

### A* Algorithm
- Informed search algorithm
- Uses heuristic (haversine distance) to guide search
- More efficient than Dijkstra for geographic routing
- Guarantees optimal solution with admissible heuristic
- Time complexity: O(E log V) in practice

## Traffic-Aware Routing

All implementations support traffic simulation:
- Increase edge weights to simulate congestion
- Modify the CSV data or add traffic multipliers in code
- Routes will automatically adjust to avoid high-weight edges

## Example Output

```
Building route_planner.go...
go build -o bin/route_planner.go ./route_planner.go # Adjust path as needed
Running route_planner.go...
./bin/route_planner.go ../data/nodes.csv ../data/edges.csv 1 6 bellman-ford # Adjust arguments as needed
=== Bellman-Ford Algorithm ===
Path from 1 to 6: 1 -> 6
Total distance: 3.50 km
Nodes explored: 2
./bin/route_planner.go ../data/nodes.csv ../data/edges.csv 1 6 astar # Adjust arguments as needed
=== A* Algorithm ===
Path from 1 to 6: 1 -> 6
Total distance: 3.50 km
Nodes explored: 3
./bin/route_planner.go ../data/nodes.csv ../data/edges.csv 1 6 dijkstra # Adjust arguments as needed
=== Dijkstra's Algorithm ===
Path from 1 to 6: 1 -> 6
Total distance: 3.50 km
Nodes explored: 3
```

## Learning Objectives

These are the things for you to learn through this lab:
1. Understand graph representation and traversal
2. Implement priority queue-based algorithms
3. Compare uninformed vs informed search strategies
4. See how the same algorithm works across different languages
5. Learn about real-world applications of graph algorithms

## Additional Resources

- [Dijkstra's Algorithm - Wikipedia](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)
- [A* Search Algorithm - Wikipedia](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [OpenStreetMap](https://www.openstreetmap.org/) - Real map data source
- [Haversine Formula](https://en.wikipedia.org/wiki/Haversine_formula) - Geographic distance calculation

# Assignment 

[ASSIGNMENT.md](assignment_materials/ASSIGNMENT.md)