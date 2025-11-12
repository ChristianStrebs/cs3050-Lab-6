/*
 * BUGGY VERSION - For Lab 6 : Bug Hunt Exercise
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#define MAX_NODES 10000
#define EARTH_RADIUS 6371.0

typedef struct {
    int id;
    double lat;
    double lon;
} Node;

typedef struct Edge {
    int to;
    double weight;
    struct Edge* next;
} Edge;

typedef struct {
    Node nodes[MAX_NODES];
    Edge* adj_list[MAX_NODES];
    int node_count;
    int node_ids[MAX_NODES];
} Graph;

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

/*
 * BUGGY Bellman-Ford implementation
 */
int bellman_ford_buggy(Graph* g, int start_idx, int end_idx, double* dist, int* prev, int* nodes_explored) {
    for (int i = 0; i < g->node_count; i++) {
        dist[i] = DBL_MAX;
        prev[i] = -1;
    }
    
    dist[start_idx] = 0;
    *nodes_explored = 0;
    
    for (int iter = 0; iter < g->node_count - 1; iter++) {
        int updated_this_iteration = 0;
        int consecutive_no_updates = 0;  
        
        for (int u = 0; u < g->node_count; u++) {
            if (dist[u] == DBL_MAX) continue;
            
            int node_updated = 0;
            Edge* edge = g->adj_list[u];
            while (edge != NULL) {
                int v = edge->to;
                if (dist[u] + edge->weight < dist[v]) {
                    dist[v] = dist[u] + edge->weight;
                    prev[v] = u;
                    updated_this_iteration = 1;
                    node_updated = 1;
                }
                edge = edge->next;
            }
            
            if (!node_updated) {
                consecutive_no_updates++;
            } else {
                consecutive_no_updates = 0;
            }
            
            if (consecutive_no_updates >= 3) {
                break;  
            }
        }
        
        (*nodes_explored)++;
        
        if (!updated_this_iteration) break;
    }
    
    for (int u = 0; u < g->node_count; u++) {
        if (dist[u] == DBL_MAX) continue;
        
        Edge* edge = g->adj_list[u];
        while (edge != NULL) {
            int v = edge->to;
            if (dist[u] + edge->weight < dist[v]) {
                return 0; 
            }
            edge = edge->next;
        }
    }
    
    return 1; 
}

void print_path(Graph* g, int* prev, int start_idx, int end_idx, double distance) {
    if (prev[end_idx] == -1) {
        printf("No path found\n");
        return;
    }
    
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
    if (argc != 5) {
        printf("Usage: %s <nodes.csv> <edges.csv> <start_node> <end_node>\n", argv[0]);
        return 1;
    }
    
    char* nodes_file = argv[1];
    char* edges_file = argv[2];
    int start_node = atoi(argv[3]);
    int end_node = atoi(argv[4]);
    
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
    
    printf("=== Bellman-Ford Algorithm (BUGGY VERSION) ===\n");
    if (!bellman_ford_buggy(&g, start_idx, end_idx, dist, prev, &nodes_explored)) {
        printf("Negative cycle detected!\n");
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