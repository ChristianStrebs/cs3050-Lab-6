package main

import (
	"container/heap"
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"strconv"
)

const earthRadius = 6371.0 // km

// Node represents a graph node
type Node struct {
	ID  int
	Lat float64
	Lon float64
}

// Edge represents a graph edge
type Edge struct {
	To     int
	Weight float64
}

// Graph represents the graph structure
type Graph struct {
	Nodes   map[int]*Node
	AdjList map[int][]Edge
}

// PQItem represents an item in the priority queue
type PQItem struct {
	Node     int
	Priority float64
	Index    int
}

// PriorityQueue implements heap.Interface
type PriorityQueue []*PQItem

func (pq PriorityQueue) Len() int { return len(pq) }

func (pq PriorityQueue) Less(i, j int) bool {
	return pq[i].Priority < pq[j].Priority
}

func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].Index = i
	pq[j].Index = j
}

func (pq *PriorityQueue) Push(x interface{}) {
	n := len(*pq)
	item := x.(*PQItem)
	item.Index = n
	*pq = append(*pq, item)
}

func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	item.Index = -1
	*pq = old[0 : n-1]
	return item
}

// NewGraph creates a new graph
func NewGraph() *Graph {
	return &Graph{
		Nodes:   make(map[int]*Node),
		AdjList: make(map[int][]Edge),
	}
}

// AddNode adds a node to the graph
func (g *Graph) AddNode(id int, lat, lon float64) {
	g.Nodes[id] = &Node{ID: id, Lat: lat, Lon: lon}
	if _, exists := g.AdjList[id]; !exists {
		g.AdjList[id] = []Edge{}
	}
}

// AddEdge adds an edge to the graph
func (g *Graph) AddEdge(from, to int, weight float64) {
	g.AdjList[from] = append(g.AdjList[from], Edge{To: to, Weight: weight})
}

// haversine calculates the distance between two points
func haversine(lat1, lon1, lat2, lon2 float64) float64 {
	dlat := (lat2 - lat1) * math.Pi / 180.0
	dlon := (lon2 - lon1) * math.Pi / 180.0
	lat1 = lat1 * math.Pi / 180.0
	lat2 = lat2 * math.Pi / 180.0

	a := math.Sin(dlat/2)*math.Sin(dlat/2) +
		math.Cos(lat1)*math.Cos(lat2)*math.Sin(dlon/2)*math.Sin(dlon/2)
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
	return earthRadius * c
}

// Dijkstra implements Dijkstra's algorithm
func (g *Graph) Dijkstra(start, end int) (map[int]float64, map[int]int, int) {
	dist := make(map[int]float64)
	prev := make(map[int]int)

	for id := range g.Nodes {
		dist[id] = math.Inf(1)
		prev[id] = -1
	}
	dist[start] = 0

	pq := make(PriorityQueue, 0)
	heap.Init(&pq)
	heap.Push(&pq, &PQItem{Node: start, Priority: 0})

	visited := make(map[int]bool)
	nodesExplored := 0

	for pq.Len() > 0 {
		item := heap.Pop(&pq).(*PQItem)
		u := item.Node

		if visited[u] {
			continue
		}
		visited[u] = true
		nodesExplored++

		if u == end {
			break
		}

		if item.Priority > dist[u] {
			continue
		}

		for _, edge := range g.AdjList[u] {
			v := edge.To
			alt := dist[u] + edge.Weight

			if alt < dist[v] {
				dist[v] = alt
				prev[v] = u
				heap.Push(&pq, &PQItem{Node: v, Priority: alt})
			}
		}
	}

	return dist, prev, nodesExplored
}

// AStar implements A* algorithm
func (g *Graph) AStar(start, end int) (map[int]float64, map[int]int, int) {
	dist := make(map[int]float64)
	prev := make(map[int]int)

	for id := range g.Nodes {
		dist[id] = math.Inf(1)
		prev[id] = -1
	}
	dist[start] = 0

	endNode := g.Nodes[end]
	heuristic := func(nodeID int) float64 {
		node := g.Nodes[nodeID]
		return haversine(node.Lat, node.Lon, endNode.Lat, endNode.Lon)
	}

	pq := make(PriorityQueue, 0)
	heap.Init(&pq)
	heap.Push(&pq, &PQItem{Node: start, Priority: heuristic(start)})

	visited := make(map[int]bool)
	nodesExplored := 0

	for pq.Len() > 0 {
		item := heap.Pop(&pq).(*PQItem)
		u := item.Node

		if visited[u] {
			continue
		}
		visited[u] = true
		nodesExplored++

		if u == end {
			break
		}

		for _, edge := range g.AdjList[u] {
			v := edge.To
			alt := dist[u] + edge.Weight

			if alt < dist[v] {
				dist[v] = alt
				prev[v] = u
				fScore := alt + heuristic(v)
				heap.Push(&pq, &PQItem{Node: v, Priority: fScore})
			}
		}
	}

	return dist, prev, nodesExplored
}

// BellmanFord implements Bellman-Ford algorithm
func (g *Graph) BellmanFord(start, end int) (map[int]float64, map[int]int, int, bool) {
	dist := make(map[int]float64)
	prev := make(map[int]int)

	for id := range g.Nodes {
		dist[id] = math.Inf(1)
		prev[id] = -1
	}
	dist[start] = 0

	nodeCount := len(g.Nodes)
	nodesExplored := 0

	// Relax edges |V| - 1 times
	for i := 0; i < nodeCount-1; i++ {
		updated := false
		for u := range g.Nodes {
			if math.IsInf(dist[u], 1) {
				continue
			}

			for _, edge := range g.AdjList[u] {
				v := edge.To
				if dist[u]+edge.Weight < dist[v] {
					dist[v] = dist[u] + edge.Weight
					prev[v] = u
					updated = true
				}
			}
		}
		nodesExplored++
		if !updated {
			break
		}
	}

	// Check for negative cycles
	for u := range g.Nodes {
		if math.IsInf(dist[u], 1) {
			continue
		}

		for _, edge := range g.AdjList[u] {
			v := edge.To
			if dist[u]+edge.Weight < dist[v] {
				return nil, nil, 0, false // Negative cycle detected
			}
		}
	}

	return dist, prev, nodesExplored, true
}

// reconstructPath builds the path from start to end
func reconstructPath(prev map[int]int, start, end int) []int {
	if prev[end] == -1 && start != end {
		return nil
	}

	path := []int{}
	current := end
	for current != -1 {
		path = append([]int{current}, path...)
		if current == start {
			break
		}
		current = prev[current]
	}

	return path
}

// printPath prints the path and distance
func printPath(g *Graph, prev map[int]int, start, end int, distance float64) {
	path := reconstructPath(prev, start, end)

	if path == nil {
		fmt.Println("No path found")
		return
	}

	fmt.Printf("Path from %d to %d: ", start, end)
	for i, node := range path {
		fmt.Printf("%d", node)
		if i < len(path)-1 {
			fmt.Print(" -> ")
		}
	}
	fmt.Println()
	fmt.Printf("Total distance: %.2f km\n", distance)
}

// loadGraph loads the graph from CSV files
func loadGraph(nodesFile, edgesFile string) (*Graph, error) {
	g := NewGraph()

	// Load nodes
	file, err := os.Open(nodesFile)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	reader := csv.NewReader(file)
	records, err := reader.ReadAll()
	if err != nil {
		return nil, err
	}

	for i, record := range records {
		if i == 0 { // Skip header
			continue
		}
		id, _ := strconv.Atoi(record[0])
		lat, _ := strconv.ParseFloat(record[1], 64)
		lon, _ := strconv.ParseFloat(record[2], 64)
		g.AddNode(id, lat, lon)
	}

	// Load edges
	file, err = os.Open(edgesFile)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	reader = csv.NewReader(file)
	records, err = reader.ReadAll()
	if err != nil {
		return nil, err
	}

	for i, record := range records {
		if i == 0 { // Skip header
			continue
		}
		from, _ := strconv.Atoi(record[0])
		to, _ := strconv.Atoi(record[1])
		distance, _ := strconv.ParseFloat(record[2], 64)
		g.AddEdge(from, to, distance)
	}

	return g, nil
}

func main() {
	if len(os.Args) != 6 {
		fmt.Printf("Usage: %s <nodes.csv> <edges.csv> <start_node> <end_node> <algorithm>\n", os.Args[0])
		fmt.Println("Algorithms: dijkstra, astar, bellman-ford")
		os.Exit(1)
	}

	nodesFile := os.Args[1]
	edgesFile := os.Args[2]
	startNode, _ := strconv.Atoi(os.Args[3])
	endNode, _ := strconv.Atoi(os.Args[4])
	algorithm := os.Args[5]

	// Load graph
	graph, err := loadGraph(nodesFile, edgesFile)
	if err != nil {
		fmt.Printf("Error loading graph: %v\n", err)
		os.Exit(1)
	}

	// Validate nodes
	if _, exists := graph.Nodes[startNode]; !exists {
		fmt.Println("Invalid start node")
		os.Exit(1)
	}
	if _, exists := graph.Nodes[endNode]; !exists {
		fmt.Println("Invalid end node")
		os.Exit(1)
	}

	// Run selected algorithm
	var dist map[int]float64
	var prev map[int]int
	var nodesExplored int

	switch algorithm {
	case "dijkstra":
		fmt.Println("=== Dijkstra's Algorithm ===")
		dist, prev, nodesExplored = graph.Dijkstra(startNode, endNode)
	case "astar":
		fmt.Println("=== A* Algorithm ===")
		dist, prev, nodesExplored = graph.AStar(startNode, endNode)
	case "bellman-ford":
		fmt.Println("=== Bellman-Ford Algorithm ===")
		var ok bool
		dist, prev, nodesExplored, ok = graph.BellmanFord(startNode, endNode)
		if !ok {
			fmt.Println("Negative cycle detected!")
			os.Exit(1)
		}
	default:
		fmt.Printf("Unknown algorithm: %s\n", algorithm)
		fmt.Println("Available algorithms: dijkstra, astar, bellman-ford")
		os.Exit(1)
	}

	// Print results
	printPath(graph, prev, startNode, endNode, dist[endNode])
	fmt.Printf("Nodes explored: %d\n", nodesExplored)
}
