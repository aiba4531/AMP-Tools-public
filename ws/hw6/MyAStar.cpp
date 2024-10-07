#include "MyAStar.h"
#include <queue>
#include <utility>

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // Initialize the results object
    int start_node = problem.init_node;
    int goal_node = problem.goal_node;

    // Get the graph object
    std::shared_ptr<amp::Graph<double>> graph = problem.graph;

    // Get the number of nodes in graph
    int num_nodes = graph->nodes().size();
    
    // Priority queue (min-heap) to store nodes with their f_cost
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> open_set;

    // Vectors to store g_cost (distance from start), f_cost (g_cost + heuristic), and visited nodes
    std::vector<double> g_cost(num_nodes, std::numeric_limits<double>::infinity());
    std::vector<double> f_cost(num_nodes, std::numeric_limits<double>::infinity());
    std::vector<bool> closed_set(num_nodes, false);
    
    // Map to track parent nodes for path reconstruction
    std::unordered_map<int, int> parent_map;

    // Initialize start node costs
    g_cost[start_node] = 0.0;
    f_cost[start_node] = heuristic.operator()(start_node);
    open_set.push({f_cost[start_node], start_node});
    
    while (!open_set.empty()) {
        // Get the node with the smallest f_cost
        int current_node = open_set.top().second;
        open_set.pop();

        // If goal node is reached, reconstruct the path
        if (current_node == goal_node) {
            result.path_cost = g_cost[goal_node];
        
            // Reconstruct the path from goal to start using the parent map
            int node = goal_node;
            while (node != start_node) {
                result.node_path.push_back(node);
                node = parent_map[node];
            }
            result.node_path.push_back(start_node);
            std::reverse(result.node_path.begin(), result.node_path.end());

            result.print();
            return result;
        }

        // Mark the node as visited
        closed_set[current_node] = true;

        for (int n = 0 ; n < graph->outgoingEdges(current_node).size(); n++) {
            double edge_cost = graph->outgoingEdges(current_node)[n];
            std::cout << "Parent: " << current_node << " Child: " << n << " Edge Cost: " << edge_cost << std::endl;

            if (closed_set[n]) {
                continue; // Skip already visited nodes
            }

            // Calculate tentative g_cost for the neighbor
            double tentative_g_cost = g_cost[current_node] + edge_cost;

            // If a shorter path to the neighbor is found
            if (tentative_g_cost < g_cost[n]) {
                parent_map[n] = current_node;
                g_cost[n] = tentative_g_cost;
                f_cost[n] = g_cost[n] + heuristic.operator()(n);

                open_set.push({f_cost[n], n});
            }
        }
    }

    // If the open set is empty and the goal is not reached, return failure
    std::cout << "Failed to find a path." << std::endl;
    result.print();
    return result;
}