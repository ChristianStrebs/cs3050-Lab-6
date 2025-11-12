#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <float.h>
#include "route_planner.h"

#define MAX_NODES 10000
#define MAX_EDGES 50000
#define EARTH_RADIUS 6371.0

// Node structure
typedef struct {
    int id;
    double lat;
    double lon;
} Node;

// Edge structure
typedef struct Edge {
    int to;
    double weight;
    struct Edge* next;
} Edge;

// Graph structure
typedef struct {
    Node nodes[MAX_NODES];
    Edge* adj_list[MAX_NODES];
    int node_count;
    int node_ids[MAX_NODES];
} Graph;

// Priority queue node for Dijkstra and A*
typedef struct {
    int node;
    double priority;
} PQNode;

// Priority queue
typedef struct {
    PQNode heap[MAX_NODES];
    int size;
} PriorityQueue;

// Helper function to calculate haversine distance
double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    
    double a = sin(dlat/2) * sin(dlat/2) + 
               cos(lat1) * cos(lat2) * sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return EARTH_RADIUS * c;
}

// Priority queue functions
void pq_init(PriorityQueue* pq) {
    pq->size = 0;
}

void pq_push(PriorityQueue* pq, int node, double priority) {
    int i = pq->size++;
    pq->heap[i].node = node;
    pq->heap[i].priority = priority;
    
    // Bubble up
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (pq->heap[parent].priority <= pq->heap[i].priority) break;
        PQNode temp = pq->heap[parent];
        pq->heap[parent] = pq->heap[i];
        pq->heap[i] = temp;
        i = parent;
    }
}

PQNode pq_pop(PriorityQueue* pq) {
    PQNode result = pq->heap[0];
    pq->heap[0] = pq->heap[--pq->size];
    
    // Bubble down
    int i = 0;
    while (1) {
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        int smallest = i;
        
        if (left < pq->size && pq->heap[left].priority < pq->heap[smallest].priority)
            smallest = left;
        if (right < pq->size && pq->heap[right].priority < pq->heap[smallest].priority)
            smallest = right;
        
        if (smallest == i) break;
        
        PQNode temp = pq->heap[i];
        pq->heap[i] = pq->heap[smallest];
        pq->heap[smallest] = temp;
        i = smallest;
    }
    
    return result;
}

int pq_empty(PriorityQueue* pq) {
    return pq->size == 0;
}

// Graph functions
void graph_init(Graph* g) {
    g->node_count = 0;
    for (int i = 0; i < MAX_NODES; i++) {
        g->adj_list[i] = NULL;
        g->node_ids[i] = -1;
    }
}

int find_node_index(Graph* g, int node_id) {
    for (int i = 0; i < g->node_count; i++) {
        if (g->node_ids[i] == node_id) return i;
    }
    return -1;
}

void add_edge(Graph* g, int from, int to, double weight) {
    Edge* edge = (Edge*)malloc(sizeof(Edge));
    edge->to = to;
    edge->weight = weight;
    edge->next = g->adj_list[from];
    g->adj_list[from] = edge;
}

// Dijkstra's algorithm
void dijkstra(Graph* g, int start_idx, int end_idx, double* dist, int* prev, int* nodes_explored) {
    PriorityQueue pq;
    pq_init(&pq);
    
    for (int i = 0; i < g->node_count; i++) {
        dist[i] = DBL_MAX;
        prev[i] = -1;
    }
    
    dist[start_idx] = 0;
    pq_push(&pq, start_idx, 0);
    *nodes_explored = 0;
    
    while (!pq_empty(&pq)) {
        PQNode current = pq_pop(&pq);
        int u = current.node;
        (*nodes_explored)++;
        
        if (u == end_idx) break;
        
        if (current.priority > dist[u]) continue;
        
        Edge* edge = g->adj_list[u];
        while (edge != NULL) {
            int v = edge->to;
            double alt = dist[u] + edge->weight;
            
            if (alt < dist[v]) {
                dist[v] = alt;
                prev[v] = u;
                pq_push(&pq, v, alt);
            }
            edge = edge->next;
        }
    }
}

// A* algorithm
void astar(Graph* g, int start_idx, int end_idx, double* dist, int* prev, int* nodes_explored) {
    PriorityQueue pq;
    pq_init(&pq);
    
    for (int i = 0; i < g->node_count; i++) {
        dist[i] = DBL_MAX;
        prev[i] = -1;
    }
    
    dist[start_idx] = 0;
    double h = haversine(g->nodes[start_idx].lat, g->nodes[start_idx].lon,
                        g->nodes[end_idx].lat, g->nodes[end_idx].lon);
    pq_push(&pq, start_idx, h);
    *nodes_explored = 0;
    
    while (!pq_empty(&pq)) {
        PQNode current = pq_pop(&pq);
        int u = current.node;
        (*nodes_explored)++;
        
        if (u == end_idx) break;
        
        Edge* edge = g->adj_list[u];
        while (edge != NULL) {
            int v = edge->to;
            double alt = dist[u] + edge->weight;
            
            if (alt < dist[v]) {
                dist[v] = alt;
                prev[v] = u;
                double h = haversine(g->nodes[v].lat, g->nodes[v].lon,
                                    g->nodes[end_idx].lat, g->nodes[end_idx].lon);
                pq_push(&pq, v, alt + h);
            }
            edge = edge->next;
        }
    }
}

// Bellman-Ford algorithm
int bellman_ford(Graph* g, int start_idx, int end_idx, double* dist, int* prev, int* nodes_explored) {
    for (int i = 0; i < g->node_count; i++) {
        dist[i] = DBL_MAX;
        prev[i] = -1;
    }
    
    dist[start_idx] = 0;
    *nodes_explored = 0;
    
    // Relax edges |V| - 1 times
    for (int i = 0; i < g->node_count - 1; i++) {
        int updated = 0;
        for (int u = 0; u < g->node_count; u++) {
            if (dist[u] == DBL_MAX) continue;
            
            Edge* edge = g->adj_list[u];
            while (edge != NULL) {
                int v = edge->to;
                if (dist[u] + edge->weight < dist[v]) {
                    dist[v] = dist[u] + edge->weight;
                    prev[v] = u;
                    updated = 1;
                }
                edge = edge->next;
            }
        }
        (*nodes_explored)++;
        if (!updated) break;
    }
    
    // Check for negative cycles
    for (int u = 0; u < g->node_count; u++) {
        if (dist[u] == DBL_MAX) continue;
        
        Edge* edge = g->adj_list[u];
        while (edge != NULL) {
            int v = edge->to;
            if (dist[u] + edge->weight < dist[v]) {
                return 0; // Negative cycle detected
            }
            edge = edge->next;
        }
    }
    
    return 1; // No negative cycle
}

void print_path(Graph* g, int* prev, int start_idx, int end_idx, double distance) {
    if (prev[end_idx] == -1) {
        printf("No path found\n");
        return;
    }
    
    // Build path
    int path[MAX_NODES];
    int path_len = 0;
    int current = end_idx;
    
    while (current != -1) {
        path[path_len++] = current;
        current = prev[current];
    }
    
    printf("Path from %d to %d: ", g->node_ids[start_idx], g->node_ids[end_idx]);
    for (int i = path_len - 1; i >= 0; i--) {
        printf("%d", g->node_ids[path[i]]);
        if (i > 0) printf(" -> ");
    }
    printf("\nTotal distance: %.2f km\n", distance);
}

int main(int argc, char* argv[]) {
    if (argc != 6) {
        printf("Usage: %s <nodes.csv> <edges.csv> <start_node> <end_node> <algorithm>\n", argv[0]);
        printf("Algorithms: dijkstra, astar, bellman-ford\n");
        return 1;
    }
    
    char* nodes_file = argv[1];
    char* edges_file = argv[2];
    int start_node = atoi(argv[3]);
    int end_node = atoi(argv[4]);
    char* algorithm = argv[5];
    
    Graph g;
    graph_init(&g);
    
    // Load nodes
    FILE* fp = fopen(nodes_file, "r");
    if (!fp) {
        printf("Error opening nodes file\n");
        return 1;
    }
    
    char line[256];
    fgets(line, sizeof(line), fp); // Skip header
    
    while (fgets(line, sizeof(line), fp)) {
        int id;
        double lat, lon;
        if (sscanf(line, "%d,%lf,%lf", &id, &lat, &lon) == 3) {
            g.nodes[g.node_count].id = id;
            g.nodes[g.node_count].lat = lat;
            g.nodes[g.node_count].lon = lon;
            g.node_ids[g.node_count] = id;
            g.node_count++;
        }
    }
    fclose(fp);
    
    // Load edges
    fp = fopen(edges_file, "r");
    if (!fp) {
        printf("Error opening edges file\n");
        return 1;
    }
    
    fgets(line, sizeof(line), fp); // Skip header
    
    while (fgets(line, sizeof(line), fp)) {
        int from, to;
        double distance;
        if (sscanf(line, "%d,%d,%lf", &from, &to, &distance) == 3) {
            int from_idx = find_node_index(&g, from);
            int to_idx = find_node_index(&g, to);
            if (from_idx != -1 && to_idx != -1) {
                add_edge(&g, from_idx, to_idx, distance);
            }
        }
    }
    fclose(fp);
    
    // Find start and end indices
    int start_idx = find_node_index(&g, start_node);
    int end_idx = find_node_index(&g, end_node);
    
    if (start_idx == -1 || end_idx == -1) {
        printf("Invalid start or end node\n");
        return 1;
    }
    
    double dist[MAX_NODES];
    int prev[MAX_NODES];
    int nodes_explored = 0;
    
    // Run selected algorithm
    if (strcmp(algorithm, "dijkstra") == 0) {
        printf("=== Dijkstra's Algorithm ===\n");
        dijkstra(&g, start_idx, end_idx, dist, prev, &nodes_explored);
    } else if (strcmp(algorithm, "astar") == 0) {
        printf("=== A* Algorithm ===\n");
        astar(&g, start_idx, end_idx, dist, prev, &nodes_explored);
    } else if (strcmp(algorithm, "bellman-ford") == 0) {
        printf("=== Bellman-Ford Algorithm ===\n");
        if (!bellman_ford(&g, start_idx, end_idx, dist, prev, &nodes_explored)) {
            printf("Negative cycle detected!\n");
            return 1;
        }
    } else {
        printf("Unknown algorithm: %s\n", algorithm);
        printf("Available algorithms: dijkstra, astar, bellman-ford\n");
        return 1;
    }
    
    print_path(&g, prev, start_idx, end_idx, dist[end_idx]);
    printf("Nodes explored: %d\n", nodes_explored);
    
    // Cleanup
    for (int i = 0; i < g.node_count; i++) {
        Edge* edge = g.adj_list[i];
        while (edge != NULL) {
            Edge* next = edge->next;
            free(edge);
            edge = next;
        }
    }
    
    return 0;
}