#include "MyAStar.h"
#include <queue>
#include <utility>

// Implement the search method for the A* algorithm
MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;
    GraphSearchResult result = {false, {}, 0.0}; // Initialize the results object
    
    // Initialize the start and goal nodes
    int start_node = problem.init_node;
    int goal_node = problem.goal_node;

    // Get the graph object
    std::shared_ptr<amp::Graph<double>> graph = problem.graph;

    // Get the number of nodes in graph
    int num_nodes = graph->nodes().size();
    
    // Priority queue (min-heap) to store nodes with their f_cost
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> open_set;
    std::vector<bool> closed_set(num_nodes, false);

    // Vectors to store g_cost (distance from start), f_cost (g_cost + heuristic), and visited nodes
    std::vector<double> g_cost(num_nodes, std::numeric_limits<double>::infinity());
    std::vector<double> f_cost(num_nodes, std::numeric_limits<double>::infinity());
    
    // Map to track parent nodes for path reconstruction
    std::unordered_map<int, int> parent_map;

    // Initialize start node costs
    g_cost[start_node] = 0.0;
    f_cost[start_node] = heuristic(start_node);
    open_set.push({f_cost[start_node], start_node});

    int numItr = 0;
    
    while (!open_set.empty()) {
        // Get the node with the smallest f_cost
        int current_node = open_set.top().second;
        open_set.pop();

        if (current_node < 0 || current_node >= num_nodes) {
        //     std::cerr << "Error: Invalid current node: " << current_node << std::endl;
        //     std::cerr << "Parent node: " << parent_map[current_node] << std::endl;
        //     std::cerr << "Number of nodes: " << num_nodes << std::endl;
        //     std::cerr << "Laste node index: " << graph->nodes().back() << std::endl;
            return result; // Exit early on error
        }

        // If goal node is reached, reconstruct the path
        if (current_node == goal_node) {
            result.success = true;
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
            std::cout << "Number of iterations: " << numItr << std::endl;
            return result;
        }

        // Mark the node as visited
        closed_set[current_node] = true;


        // Access neighboring nodes and corresponding edge costs
        const auto& neighbors = graph->children(current_node);  // Get neighbors of current_node
        const auto& edges = graph->outgoingEdges(current_node);   // Get edge costs of current_node

        // for (const auto& neighbor_node : neighbors) {
        //     double edge_cost = edges[neighbor_node];
        //     if (closed_set[neighbor_node]) {
        //         continue; // Skip already visited nodes
        //     }

        //     // Calculate tentative g_cost for the neighbor
        //     double tentative_g_cost = g_cost[current_node] + edge_cost;

        //     // If a shorter path to the neighbor is found
        //     if (tentative_g_cost < g_cost[neighbor_node]) {
        //         parent_map[neighbor_node] = current_node;
        //         g_cost[neighbor_node] = tentative_g_cost;
        //         f_cost[neighbor_node] = g_cost[neighbor_node] + heuristic(neighbor_node);
        //         open_set.push({f_cost[neighbor_node], neighbor_node});
        //     }
        // }
        

        // Iterate over the neighbors of the current node
        for (int k = 0; k < neighbors.size(); k++) {
            int neighbor_node = neighbors[k];
            double edge_cost = edges[k];

            if (closed_set[neighbor_node]) {
                continue; // Skip already visited nodes
            }


            // Calculate tentative g_cost for the neighbor
            double tentative_g_cost = g_cost[current_node] + edge_cost;

            // If a shorter path to the neighbor is found
            if (tentative_g_cost < g_cost[neighbor_node]) {
                parent_map[neighbor_node] = current_node;
                g_cost[neighbor_node] = tentative_g_cost;
                f_cost[neighbor_node] = g_cost[neighbor_node] + heuristic(neighbor_node);
                open_set.push({f_cost[neighbor_node], neighbor_node});
            }
        }
        numItr++;
    }    
    
    // If the open set is empty and the goal is not reached, return failure
    std::cout << "Failed to find a path." << std::endl;
    result.print();
    return result;
}