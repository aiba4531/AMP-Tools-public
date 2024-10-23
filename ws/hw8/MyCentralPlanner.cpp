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
    // std::vector<std::vector< Eigen::Vector2d>> time_history_states;
    std::vector<Eigen::Vector2d> initial_states, current_states, goal_states, sampled_states; // vector of (x,y) coordinates for each agent
    std::vector<bool> goal_reached; // Track if each agent has reached its goal
    std::map<amp::Node, amp::Node> parent_map; // Store parent nodes for meta agent
    std::map<amp::Node, std::vector<Eigen::Vector2d>> nodes; // Store nodes for meta agent

    int num_agents = problem.numAgents(); // Number of agents

    // Define the iniital states and goal states for each agent
    for (const auto& agent : problem.agent_properties) {
        initial_states.push_back(agent.q_init);
        current_states.push_back(agent.q_init);
        sampled_states.push_back(agent.q_init);
        goal_states.push_back(agent.q_goal);
        goal_reached.push_back(false); // Initialize goal_reached for each agent

        // Create an agent path and add it to the multi-agent path
        path.agent_paths.push_back(amp::Path2D());
    }

    // Number of iterations
    int num_itr = 0; 
    int inside_num_itr = 0; 
    double x_rand, y_rand, radius, radius2, distance;
    bool robot_robot_collision;
    Eigen::Vector2d sample;
    amp::Node curr_node;
    current_states = initial_states; // Set the current states to the initial states
    nodes[0] = initial_states; // Add the initial states to the nodes
    parent_map[0] = -1; // Set the parent of the initial node to -1

    // for (int i = 0; i < num_agents; i++) {
    //     path.agent_paths[i].waypoints.push_back(initial_states[i]); // Add the initial state to the path
    // }

    // While the goal is not reached or a number of iterations is not reached
    while (num_itr < max_itr) {

        // For each agent
        for (int i = 0; i < num_agents; i++) {

            if (goal_reached[i]) {
                sampled_states[i] = goal_states[i]; // If the goal is reached, sample the goal state
                continue; // Skip to the next agent
            }

            // Sample the goal with probability goal_bias
            double prob = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            if (prob < goal_bias) {
                sampled_states[i] = goal_states[i]; // Sample the goal state

            } else {
                // Sample a random point in the workspace
                while (inside_num_itr < 100) {
                    robot_robot_collision = false; // Check for robot-robot collision
                    x_rand = x_min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (x_max - x_min)));
                    y_rand = y_min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (y_max - y_min)));
                    sample = Eigen::Vector2d(x_rand, y_rand);
                    radius = problem.agent_properties[i].radius;

                    // Check if the sample is in collision with any obstacles
                    if (!robot_in_polygons(sample, radius, problem)) {
                        break; // Exit the loop if the sample is valid
                    }
                    inside_num_itr++;
                }
                sampled_states[i] = sample; // Update the current state of the agent
            }
        }
        
        // Now we have valid sampled states for the meta agent, connect the sampled states to the closest node in tree
        amp::Node closest_node = find_closest_node(nodes, sampled_states);
        std::vector<Eigen::Vector2d> closest_node_state = nodes[closest_node]; // Get the closest node state

        // Try to have all robots move towards the sampled state from closest node
        for (int i = 0; i < num_agents; i++) {

            robot_robot_collision = false; // Check for robot-robot collision
            if (goal_reached[i]) {
                current_states[i] = goal_states[i]; // If the goal is reached, set the current state to the goal state
                continue; // Skip to the next agent if the goal is already reached
            }

            // Calculate the direction to the sampled state
            Eigen::Vector2d direction = (sampled_states[i] - nodes[closest_node][i]).normalized();
            Eigen::Vector2d next_step = nodes[closest_node][i] + direction * r;

            // Check for collisions with obstacles
            if (robot_in_polygons(next_step, problem.agent_properties[i].radius, problem)) {
                current_states[i] = current_states[i]; // If in collision, stay at the current node
                continue; // Skip this step if it is in collision
            }

            // Check for collisions with other robots
            for (int j = 0; j < num_agents; j++) {
                if(i != j && robot_in_robot(next_step, problem.agent_properties[i].radius, current_states[j], problem.agent_properties[j].radius)) {
                    robot_robot_collision = true; // Set the collision flag
                    current_states[i] = current_states[i]; // If in collision, stay at the current node
                    break; // Skip this step if it is in collision with another robot
                }
            }

            if (!robot_robot_collision) {
                current_states[i] = next_step; // If no collisions, update the node
            }
        }

        // Is ths goal reached for any agnets?
        bool all_goals_reached = true;
        for (int i = 0; i < num_agents; i++) {

            if(goal_reached[i] ) {
                continue; // Skip to the next agent if the goal is already reached
            }

            distance = (current_states[i] - goal_states[i]).norm();
            if (distance <= epsilon) {
                std::cout << "Goal reached for agent " << i << std::endl;
                std::cout << "Current state: " << current_states[i].transpose() << std::endl;
                current_states[i] = goal_states[i]; // Set the current state to the goal state
                goal_reached[i] = true; // Mark the goal as reached
            }
            else {
                all_goals_reached = false; // At least one agent has not reached its goal
            }
        }

        
        // Now the current states are valid places to move all robots so store in the tree
        // Add the new node to the graph
        num_itr++;
        curr_node = num_itr; // Create a new node index
        nodes[curr_node] = current_states; // Add the new node to the graph
        parent_map[curr_node] = closest_node; // Store the parent of the new node


        if (all_goals_reached) {
            std::cout << "All goals reached!" << std::endl;
            break; // Exit the loop if all goals are reached
        }
    }

    std::cout << "Number of iterations: " << num_itr << std::endl;

    for (amp::Node j = curr_node; j != -1; j = parent_map[j]) {
        // Reconstruct the path for each agent
        for (int i = 0; i < num_agents; i++) {
            //std::cout << "Node: " << j << " Agent: " << i << " State: " << nodes[j][i].transpose() << std::endl;
            path.agent_paths[i].waypoints.insert(path.agent_paths[i].waypoints.begin(), nodes[j][i]);            
        }
    }

    

    if (num_itr == max_itr) {
        for (int i = 0; i < num_agents; i++) {
            path.agent_paths[i].waypoints.push_back(goal_states[i]); // Add the goal state as the last waypoint
        }
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
    std::vector<double> distances; // Store distances for each agent

    for (const auto& [node, state] : nodes) {
        total_distance = 0; // Reset total distance for each node
        distances.clear(); // Clear the distances vector for each node
        for (size_t i = 0; i < state.size(); i++) {
            distances.push_back((state[i] - samples[i]).norm()); // Calculate distance for each agent
            distance = (state[i] - samples[i]).norm(); // Calculate distance for each agent
            //distance = (state[i] - samples[i]).squaredNorm(); // Calculate squared distance for each agent
            total_distance += distance; // Accumulate total distance
        }

        double max_distance = *std::max_element(distances.begin(), distances.end()); // Find the maximum distance
        // std::cout << "Max distance: " << max_distance << std::endl; // Print max distance
        // std::cout << "Total distance: " << total_distance << std::endl; // Print total distance

        if (total_distance < min_distance) {
        // if (max_distance < min_distance) {
            min_distance = total_distance; // Update minimum distance
            nearest_node = node; // Update nearest node
        }
    }
    return nearest_node; // Return the nearest node
}  


// 2D distance from a point to a line segment
double distance_to_segment(const Eigen::Vector2d& circle_center, const Eigen::Vector2d& edge_start, const Eigen::Vector2d& edge_end) {
    constexpr double epsilon = 1e-9; // Tolerance for small floating-point errors
    
    // Calculate the length of the edge
    Eigen::Vector2d edge = edge_end - edge_start;
    double edge_length_squared = edge.squaredNorm();

    // if the edge length is zero, return the distance from the circle center to the edge start
    if (edge_length_squared < epsilon) {
        return (circle_center - edge_start).norm();
    }

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
bool point_in_polygons(const Eigen::Vector2d& point, const amp::MultiAgentProblem2D& problem) {   
    constexpr double epsilon = 1e-9; // Small tolerance for floating-point errors

    // For every obstacle in the problem
    for (const auto& obstacle : problem.obstacles) {
        bool inside = false;
        // Get the vertices of the polygon
        std::vector<Eigen::Vector2d> polygon = obstacle.verticesCW();
        int num_vertices = polygon.size();
    
        // Store the first point in the polygon and initialize the second point
        Eigen::Vector2d p1 = polygon[0], p2;
    
        // Loop through each edge in the polygon
        for (int i = 1; i <= num_vertices; i++) {
            // Get the next point in the polygon
            p2 = polygon[i % num_vertices];
    
            // Ray-casting algorithm
            if ((point.y() > std::min(p1.y(), p2.y()) - epsilon) && 
                (point.y() <= std::max(p1.y(), p2.y()) + epsilon) &&
                (point.x() <= std::max(p1.x(), p2.x()) + epsilon)) {
                
                // Check x intersection
                double x_intersection = (p2.y() - p1.y() != 0) ?
                    ((point.y() - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x()) : p1.x();

                if (p1.x() == p2.x() || point.x() <= x_intersection + epsilon) {
                    inside = !inside;
                }
            }
            p1 = p2;
        }

        if (inside) {
            return true;
        }
    }
    return false;
}

// Robot Inside 2D Polygon
bool robot_in_polygons(const Eigen::Vector2d& robot_center, double radius, const amp::MultiAgentProblem2D& problem){
    constexpr double epsilon = 1e-9;

    // Check if the robot's center is inside any polygon
    if (point_in_polygons(robot_center, problem)) {
        return true;
    }

    // Check if the robot's radius intersects with any polygon edges
    for (const auto& obstacle : problem.obstacles) {
        const std::vector<Eigen::Vector2d>& vertices = obstacle.verticesCW();
        size_t num_vertices = vertices.size();

        for (size_t i = 0; i < num_vertices; ++i) {
            Eigen::Vector2d vertex1 = vertices[i];
            Eigen::Vector2d vertex2 = vertices[(i + 1) % num_vertices];

            // Check if the distance from the robot center to the edge is less than the radius
            if (distance_to_segment(robot_center, vertex1, vertex2) <= radius + epsilon) {
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