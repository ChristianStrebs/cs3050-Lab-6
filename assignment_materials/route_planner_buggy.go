/*
 * BUGGY VERSION - For Lab 6 : Bug Hunt Exercise
 */

package main

import (
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"strconv"
)

const earthRadius = 6371.0

type Node struct {
	ID  int
	Lat float64
	Lon float64
}

type Edge struct {
	To     int
	Weight float64
}

type Graph struct {
	Nodes   map[int]*Node
	AdjList map[int][]Edge
}

func NewGraph() *Graph {
	return &Graph{
		Nodes:   make(map[int]*Node),
		AdjList: make(map[int][]Edge),
	}
}

func (g *Graph) AddNode(id int, lat, lon float64) {
	g.Nodes[id] = &Node{ID: id, Lat: lat, Lon: lon}
	if _, exists := g.AdjList[id]; !exists {
		g.AdjList[id] = []Edge{}
	}
}

func (g *Graph) AddEdge(from, to int, weight float64) {
	g.AdjList[from] = append(g.AdjList[from], Edge{To: to, Weight: weight})
}

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

/*
 * BUGGY Bellman-Ford implementation
 */
func (g *Graph) BellmanFordBuggy(start, end int) (map[int]float64, map[int]int, int, bool) {
	dist := make(map[int]float64)
	prev := make(map[int]int)

	for id := range g.Nodes {
		dist[id] = math.Inf(1)
		prev[id] = -1
	}
	dist[start] = 0

	nodeCount := len(g.Nodes)
	nodesExplored := 0

	for iter := 0; iter < nodeCount-1; iter++ {
		updatedThisIteration := false
		consecutiveNoUpdates := 0 // BUG: This shouldn't exist

		for u := range g.Nodes {
			if math.IsInf(dist[u], 1) {
				continue
			}

			nodeUpdated := false
			for _, edge := range g.AdjList[u] {
				v := edge.To
				if dist[u]+edge.Weight < dist[v] {
					dist[v] = dist[u] + edge.Weight
					prev[v] = u
					updatedThisIteration = true
					nodeUpdated = true
				}
			}

			if !nodeUpdated {
				consecutiveNoUpdates++
			} else {
				consecutiveNoUpdates = 0
			}

			if consecutiveNoUpdates >= 3 {
				break
			}
		}

		nodesExplored++

		if !updatedThisIteration {
			break
		}
	}

	for u := range g.Nodes {
		if math.IsInf(dist[u], 1) {
			continue
		}

		for _, edge := range g.AdjList[u] {
			v := edge.To
			if dist[u]+edge.Weight < dist[v] {
				return nil, nil, 0, false
			}
		}
	}

	return dist, prev, nodesExplored, true
}

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
	if len(os.Args) != 5 {
		fmt.Printf("Usage: %s <nodes.csv> <edges.csv> <start_node> <end_node>\n", os.Args[0])
		os.Exit(1)
	}

	nodesFile := os.Args[1]
	edgesFile := os.Args[2]
	startNode, _ := strconv.Atoi(os.Args[3])
	endNode, _ := strconv.Atoi(os.Args[4])

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

	fmt.Println("=== Bellman-Ford Algorithm (BUGGY VERSION) ===")
	dist, prev, nodesExplored, ok := graph.BellmanFordBuggy(startNode, endNode)
	if !ok {
		fmt.Println("Negative cycle detected!")
		os.Exit(1)
	}

	printPath(graph, prev, startNode, endNode, dist[endNode])
	fmt.Printf("Nodes explored: %d\n", nodesExplored)
}
