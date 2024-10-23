#include "MyMultiAgentPlanners.h"

// Example function declarations
bool robot_in_polygons(const Eigen::Vector2d& sample, double radius, const amp::MultiAgentProblem2D& problem);
bool robot_in_robot(const Eigen::Vector2d& sample, double radius, const Eigen::Vector2d& current_state, double radius2);
amp::Node find_closest_node(const std::map<amp::Node, std::vector<Eigen::Vector2d>>& nodes, const std::vector<Eigen::Vector2d>& samples);

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    
    // Create a 2D path for multiple agents
    amp::MultiAgentPath2D path;

    // Get the bounds of the workspace
    double x_max = problem.x_max;
    double x_min = problem.x_min;
    double y_max = problem.y_max;
    double y_min = problem.y_min;


    // Create a vector to store the initial states of the agents
    std::vector<std::vector< Eigen::Vector2d>> time_history_states;
    std::vector<Eigen::Vector2d> initial_states, current_states, goal_states, sampled_states; // vector of (x,y) coordinates for each agent
    Eigen::Vector2d sample, agent1, agent2; // Sampled point
    amp::Node curr_node;
    std::vector<bool> goal_reached; // Track if each agent has reached its goal

    std::vector<std::map<amp::Node, amp::Node>> parents_map;
    std::map<amp::Node, amp::Node> parent_map; // Store parent nodes for each 

    std::map<amp::Node, Eigen::Vector2d> nodes; // Store nodes for each agent
    std::vector<std::map<amp::Node, Eigen::Vector2d>> nodes_map; // Store nodes for each agent
    
    int num_agents = problem.numAgents(); // Number of agents

    for (const auto& agent : problem.agent_properties) {
        initial_states.push_back(agent.q_init);
        goal_states.push_back(agent.q_goal);
    }

    for (int i = 0; i < num_agents; i++) {
        goal_reached.push_back(false); // Initialize goal_reached for each agent
        nodes[0] = initial_states[i]; // Add the initial state to the nodes
        nodes_map.push_back(nodes); // Store the nodes for the first agent
    }


    // Create a tree with root at the start node
    // graphPtr = std::make_shared<amp::Graph<double>>();
    // nodes = std::map<amp::Node, std::vector<Eigen::Vector2d>>(); // node and full meta state of robot

    // Add the start point as a node
    nodes[0] = initial_states;
    parent_map[0] = -1; // Root node has no parent
    nodes_map.push_back(nodes); // Store the nodes for the first agent
    current_states = initial_states;
    time_history_states.push_back(current_states); // Store the initial state

    // Number of iterations
    int num_itr = 0; 
    double radius, radius2, x_rand, y_rand, distance;
    // While the goal is not reached or a number of iterations is not reached
    while (num_itr < max_itr) {
        // Sample the goal with probability goal_bias
        double prob = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        if (prob < goal_bias) {

            for (int i = 0; i < num_agents; i++) {
                if( goal_reached[i] ) {
                    sampled_states[i] = current_states[i]; // Keep the current state if the goal is reached
                } else {
                    sampled_states[i] = goal_states[i]; // Sample the goal state
                }
            }

        } else {
            // Sample a random point in the workspace
            for (int i = 0; i < num_agents; i++) {

                if( goal_reached[i] ) {
                    sampled_states[i] = current_states[i]; // Keep the current state if the goal is reached
                    continue; // Skip to the next agent
                }

                while (true) {
                    x_rand = x_min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (x_max - x_min)));
                    y_rand = y_min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (y_max - y_min)));
                    sample = Eigen::Vector2d(x_rand, y_rand);
                    radius = problem.agent_properties[i].radius;

                
                    // Check if the sample is in collision with any obstacles
                    if (robot_in_polygons(sample, radius, problem)) {
                        continue; // Skip this sample if it is in collision
                    }

                    for (int j = 0; j < num_agents; j++) {
                        if (i != j){
                            radius2 = problem.agent_properties[j].radius;
                            agent2 = current_states[j];

                            if (robot_in_robot(sample, radius, agent2, radius2)) {
                                continue; // Skip this sample if it is in collision with another robot
                            }
                        }
                    }

                    break; // Exit the loop if a valid sample is found
                }
                sampled_states[i] = sample; // Update the current state of the agent
            }
        }
    

        // Find closest configuration in the tree to current configuration
        // amp::Node cloest_node = find_closest_node(nodes, sampled_states);

        // Try to have all roboits move towards the sampled state
        for (int i = 0; i < num_agents; i++) {

            if(goal_reached[i] ) {
                current_states[i] = goal_states[i]; // Set the current state to the goal state
                continue; // Skip to the next agent if the goal is already reached
            }
            amp::Node cloest_node = find_closest_node(nodes, sampled_states);


            Eigen::Vector2d direction = (sampled_states[i] - nodes[cloest_node][i]).normalized();
            Eigen::Vector2d next_step = nodes[cloest_node][i] + direction * r;

            // Check for collisions with obstacles
            if (robot_in_polygons(next_step, problem.agent_properties[i].radius, problem)) {
                continue; // Skip this step if it is in collision
            }

            // Check for collisions with other robots
            for (int j = 0; j < num_agents; j++) {
                if (i != j) {
                    if (robot_in_robot(next_step, problem.agent_properties[i].radius, current_states[j], problem.agent_properties[j].radius)) {
                        continue; // Skip this step if it is in collision with another robot
                    }
                }
            }

            // If no collisions, update the node
            current_states[i] = next_step;
        }

        // Is ths goal reached for any agnets?
        bool all_goals_reached = true;
        for (int i = 0; i < num_agents; i++) {

            if(goal_reached[i] ) {
                continue; // Skip to the next agent if the goal is already reached
            }

            distance = (current_states[i] - goal_states[i]).norm();
            if (distance < epsilon) {
                std::cout << "Goal reached for agent " << i << std::endl;
                std::cout << "Current state: " << current_states[i].transpose() << std::endl;
                current_states[i] = goal_states[i]; // Set the current state to the goal state
                goal_reached[i] = true; // Mark the goal as reached
                
            }
            else {
                all_goals_reached = false; // At least one agent has not reached its goal
            }
        }

        // Add the new node to the graph
        num_itr++;
        curr_node = num_itr; // Create a new node index
        nodes[curr_node] = current_states; // Add the new node to the graph
        parent_map[curr_node] = cloest_node; // Store the parent of the new node
        time_history_states.push_back(current_states); // Store the history of states

        if (all_goals_reached) {
            std::cout << "Curent node: " << curr_node << std::endl;
            std::cout << "Parent node: " << parent_map[curr_node] << std::endl;
            std::cout << "All goals reached!" << std::endl;
            break; // Exit the loop if all goals are reached
        }
    }

    std::cout << "Number of iterations: " << num_itr << std::endl;

    // Create a path for each agent
    for (int i = 0; i < num_agents; i++) {
        amp::Path2D agent_path;
        agent_path.waypoints.push_back(goal_states[i]);
        // Reconstruct the path for each agent
        for (amp::Node j = curr_node; j != -1; j = parent_map[j]) {
            std::cout << "Cuurrent node: " << j << " with parent: " << parent_map[j];
            std::cout << " with waypoint for agent " << i << ": " << nodes[j][i].transpose() << std::endl;
            agent_path.waypoints.push_back(nodes[j][i]);
        }
        agent_path.waypoints.push_back(initial_states[i]); // Add the goal state as the last waypoint
        std::reverse(agent_path.waypoints.begin(), agent_path.waypoints.end()); // Reverse the path to get the correct order
        path.agent_paths.push_back(agent_path);
    }

    return path;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
        amp::Path2D agent_path;
        agent_path.waypoints = {agent.q_init, agent.q_goal};
        path.agent_paths.push_back(agent_path);
    }
    return path;
}


amp::Node find_closest_node(const std::map<amp::Node, std::vector<Eigen::Vector2d>>& nodes, const std::vector<Eigen::Vector2d>& samples) {
    amp::Node nearest_node = 0;
    double min_distance = std::numeric_limits<double>::max();
    double total_distance, distance = 0; // Initialize total distance

    for (const auto& [node, state] : nodes) {
        total_distance = 0; // Reset total distance for each node
        for (size_t i = 0; i < state.size(); ++i) {
            distance = (state[i] - samples[i]).norm(); // Calculate distance for each agent
            total_distance += distance; // Accumulate total distance
        }
        if (total_distance < min_distance) {
            min_distance = total_distance; // Update minimum distance
            nearest_node = node; // Update nearest node
        }
    }
    return nearest_node; // Return the nearest node
}  

amp::Node find_closest_node_per_robot(const std::map<amp::Node, std::vector<Eigen::Vector2d>>& nodes, const std::vector<Eigen::Vector2d>& samples) {
    amp::Node nearest_node = 0;
    double min_distance = std::numeric_limits<double>::max();
    double total_distance, distance = 0; // Initialize total distance

    for (const auto& [node, state] : nodes) {
        total_distance = 0; // Reset total distance for each node
        for (size_t i = 0; i < state.size(); ++i) {
            distance = (state[i] - samples[i]).norm(); // Calculate distance for each agent
            total_distance += distance; // Accumulate total distance
        }
        if (total_distance < min_distance) {
            min_distance = total_distance; // Update minimum distance
            nearest_node = node; // Update nearest node
        }
    }
    return nearest_node; // Return the nearest node
}  


// 2D distance from a point to a line segment
double distance_to_segment(const Eigen::Vector2d& circle_center, const Eigen::Vector2d& edge_start, const Eigen::Vector2d& edge_end) {
    // Calculate the length of the edge
    Eigen::Vector2d edge = edge_end - edge_start;
    double edge_length_squared = edge.squaredNorm();

    // Calculate the projection of the circle center onto the edge
    double t = ((circle_center - edge_start).dot(edge)) / edge_length_squared;

    // Clamp t to the range [0, 1] to find the closest point on the segment
    t = std::max(0.0, std::min(1.0, t));

    // Calculate the closest point on the segment
    Eigen::Vector2d closest_point = edge_start + t * edge;

    // Return the distance from the circle center to the closest point
    return (circle_center - closest_point).norm();
}

// Checking if a point is inside a polygon
bool point_in_polygons(Eigen::Vector2d point, const amp::MultiAgentProblem2D& problem) {   

    // For every obstacle in the problem
    for (int i = 0; i < problem.obstacles.size(); i++) {
        bool inside = false;
        // Get the vertices of the polygon
        std::vector<Eigen::Vector2d> polygon = problem.obstacles[i].verticesCW();
        int num_vertices = polygon.size();

        // Get the x and y coordinates of the point
        double x = point.x(), y = point.y();
    
        // Store the first point in the polygon and initialize the second point
        Eigen::Vector2d p1 = polygon[0], p2;
    
        // Loop through each edge in the polygon
        for (int i = 1; i <= num_vertices; i++) {
            // Get the next point in the polygon
            p2 = polygon[i % num_vertices];
    
            // Check if the point is above the minimum y coordinate of the edge
            if (y > std::min(p1.y(), p2.y())) {
                // Check if the point is below the maximum y coordinate of the edge
                if (y <= std::max(p1.y(), p2.y())) {
                    // Check if the point is to the left of the maximum x coordinate of the edge
                    if (x <= std::max(p1.x(), p2.x())) {
                        // Calculate the x-intersection of the line connecting the point to the edge
                        double x_intersection = (y - p1.y()) * (p2.x() - p1.x())/ (p2.y() - p1.y()) + p1.x();
    
                        // Check if the point is on the same line as the edge or to the left of the x-intersection
                        if (p1.x() == p2.x() || x <= x_intersection) {
                            // Return the point is inside this polygon
                            inside = !inside;
                        }
                    }
                }
            }
    
            // Store the current point as the first point for the next iteration
            p1 = p2;
        }

        // If the point is inside the polygon, return true
        if (inside) {
            return true;
        }
    
    }
    // Return the value of the inside flag
    return false;
}

// Robot Inside 2D Polygon
bool robot_in_polygons(const Eigen::Vector2d& robot_center, double radius, const amp::MultiAgentProblem2D& problem){
    // Check if the robot's center is inside the polygon
    if (point_in_polygons(robot_center, problem)) {
        return true;
    }

    // Check if the robot's radius intersects with the polygon edges
    for (const auto& obstacle : problem.obstacles) {
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCW();
        for (size_t i = 0; i < vertices.size(); ++i) {
            Eigen::Vector2d vertex1 = vertices[i];
            Eigen::Vector2d vertex2 = vertices[(i + 1) % vertices.size()];

            // Check if the distance from the robot center to the edge is less than the radius
            if (distance_to_segment(robot_center, vertex1, vertex2) < radius + 0.05) {
                return true;
            }
        }
    }

    return false;
}

// 2D Robot Collision Check
bool robot_in_robot( const Eigen::Vector2d& robot1_center, double robot1_radius, const Eigen::Vector2d& robot2_center, double robot2_radius) { 
    double distance = (robot1_center - robot2_center).norm();
    return distance < (robot1_radius + robot2_radius);
}