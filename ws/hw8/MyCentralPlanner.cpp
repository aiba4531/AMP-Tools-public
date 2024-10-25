#include "MyMultiAgentPlanners.h"

// Example function declarations
bool robot_in_polygons(const Eigen::Vector2d& sample, double radius, const amp::MultiAgentProblem2D& problem);
bool robot_in_robot(const Eigen::Vector2d& sample, double radius, const Eigen::Vector2d& current_state, double radius2);
amp::Node find_closest_node(const std::map<amp::Node, std::vector<Eigen::Vector2d>>& nodes, const std::vector<Eigen::Vector2d>& samples);
amp::Node find_closest_state_node(const std::map<amp::Node, Eigen::Vector2d>& nodes, const Eigen::Vector2d& sample);
bool will_robots_collide(const Eigen::Vector2d& pos1_start, const Eigen::Vector2d& pos1_end, double radius1, const Eigen::Vector2d& pos2_start, const Eigen::Vector2d& pos2_end, double radius2);

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    
    // Create a 2D path for multiple agents
    amp::MultiAgentPath2D path;

    // Number of agents    
    int num_agents = problem.numAgents(); 
    std::vector<bool> agents_at_goal(num_agents, false); // Track agents that have reached their goal

    // Get the bounds of the workspace
    double x_max = problem.x_max;
    double x_min = problem.x_min;
    double y_max = problem.y_max;
    double y_min = problem.y_min;

    // Create a graph to store the nodes for each agent
    std::vector<std::map<amp::Node, Eigen::Vector2d>> nodes(num_agents);
    std::vector<std::map<amp::Node, amp::Node>> parent_map(num_agents);
    std::vector<Eigen::Vector2d> sampled_points(num_agents);
    std::vector<Eigen::Vector2d> current_states(num_agents);
    std::vector<Eigen::Vector2d> new_state(num_agents);

    // Initialize the nodes and parent map for each agent
    for (int i = 0; i < num_agents; i++) {
        current_states[i] = problem.agent_properties[i].q_init; // Initialize the current state
        nodes[i][0] = current_states[i]; // Initialize the first node with the initial state
        parent_map[i][0] = -1; // Set the parent of the first node to itself
        path.agent_paths.push_back(amp::Path2D()); // Initialize the path for each agent

        std::cout << "Agent " << i << " initialized at: " << current_states[i].transpose() << " moving to: " << problem.agent_properties[i].q_goal.transpose() << std::endl;
    }

    // Intialize a counter for the number of iterations
    int iterations = 0;
    amp::Node nearest_node, new_node;

    while(iterations < max_itr) {
        
        for (int i = 0; i < num_agents; i++){

            // Skip agents that have already reached their goal
            if (agents_at_goal[i]) {
                continue; // Skip to the next agent
            }

            // Randomly sample with goal bias probability
            Eigen::Vector2d sample;
            double rand_num = static_cast<double>(rand()) / RAND_MAX; // Random number between 0 and 1

            if (rand_num < goal_bias) {
                sample = problem.agent_properties[i].q_goal; // Sample the goal
            } else {

                while(true){
                    double x = (x_max - x_min) * static_cast<double>(rand()) / RAND_MAX + x_min; // Sample x
                    double y = (y_max - y_min) * static_cast<double>(rand()) / RAND_MAX + y_min; // Sample y
                    sample = Eigen::Vector2d(x, y); // Create the sample point
                
                    if(!robot_in_polygons(sample, problem.agent_properties[i].radius, problem)){
                        break; // Exit the loop if the sample is valid
                    }
                }
            }

            // Given this sample for this robot, find the closest node in its respective tree
            nearest_node = find_closest_state_node(nodes[i], sample);
            Eigen::Vector2d nearest_state = nodes[i][nearest_node];

            // Extend a branch towards the sample point
            Eigen::Vector2d direction = (sample - nearest_state).normalized(); // Normalize the direction
            new_state[i] = nearest_state + epsilon * direction; // Create a new state
        }
        
        std::vector<bool> collision(num_agents, false); // Track collisions for each agent
        for (int i = 0; i < num_agents; i++) {

            if (robot_in_polygons(new_state[i], problem.agent_properties[i].radius, problem)) {
                collision[i] = true; // No collision with the environment
                continue;
            }

            for (int j = 0; j < num_agents; j++) {
                if (j != i) {
                    if (will_robots_collide(current_states[i], new_state[i], problem.agent_properties[i].radius, current_states[j], new_state[j], problem.agent_properties[j].radius)) {
                        collision[i] = true; // Collision detected
                        break;
                    }
                }
            }
        }

        // Update the states and nodes for agents that are not in collision
        for (int i = 0; i < num_agents; i++) {
            if (!collision[i]) {
                current_states[i] = new_state[i];
            } 
            // Check if the agent has reached its goal
            if ((current_states[i] - problem.agent_properties[i].q_goal).norm() < epsilon && agents_at_goal[i] == false) {
                agents_at_goal[i] = true; // Mark the agent as at goal
                current_states[i] = problem.agent_properties[i].q_goal; // Set the current state to the goal
                std::cout << "Agent " << i << " has reached its goal." << std::endl;
            }

            // Add the new state to the tree
            nearest_node = find_closest_state_node(nodes[i], current_states[i]);
            new_node = nodes[i].size(); // Create a new node
            nodes[i][new_node] = current_states[i]; // Add the new state to the tree
            parent_map[i][new_node] = nearest_node; // Set the parent of the new node            
        }

        // Are agents in collision at the end of this iteration?
        for (int i = 0; i < num_agents; i++) {
            for (int j = i + 1; j < num_agents; j++) {
                if (robot_in_robot(current_states[i], problem.agent_properties[i].radius, current_states[j], problem.agent_properties[j].radius)) {
                    std::cout << "Collision detected between agents " << i << " and " << j << std::endl;
                }
            }
        }

        if (std::all_of(agents_at_goal.begin(), agents_at_goal.end(), [](bool at_goal) { return at_goal; })) {
            std::cout << "All agents have reached their goal." << std::endl;

            for (int i = 0; i < num_agents; i++) {
                nearest_node = find_closest_state_node(nodes[i], problem.agent_properties[i].q_goal);
                new_node = nodes[i].size(); // Create a new node
                nodes[i][new_node] = problem.agent_properties[i].q_goal; // Add the goal state to the tree
                parent_map[i][new_node] = nearest_node; // Set the parent of the new node
            }

            break; // Exit if all agents have reached their goal
        }
        
        iterations++;
    }

    // Construct the path for each agent using same nodes because they cant be in collision
    for (int i = 0; i < num_agents; i++) {
        amp::Node current_node = nodes[i].size() - 1; // Start from the last node (goal)

        std::cout << "Agent " << i << " has " << nodes[i].size() << " nodes in its tree." << std::endl;
        
        // Backtrack to construct the path
        while (current_node != -1) {
            std::cout << current_node << std::endl;
            path.agent_paths[i].waypoints.insert(path.agent_paths[i].waypoints.begin(), nodes[i][current_node]); // Add the current state to the path
            current_node = parent_map[i][current_node]; // Move to the parent node
        }
    }

    // Are the agents in collision after reconstruction
    for (int i = 0; i < num_agents; i++) {
        for (int j = i + 1; j < num_agents; j++) {
            for (size_t k = 0; k < path.agent_paths[i].waypoints.size(); k++) {
                for (size_t l = 0; l < path.agent_paths[j].waypoints.size(); l++) {
                    if (robot_in_robot(path.agent_paths[i].waypoints[k], problem.agent_properties[i].radius, path.agent_paths[j].waypoints[l], problem.agent_properties[j].radius)) {
                        std::cout << "Collision detected in the final path between agents " << i << " and " << j << std::endl;
                    }
                }
            }
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

    // Plan for every agent using RRT then resolve conflicts
    int num_agents = problem.numAgents();

    return path;
}

bool will_robots_collide(const Eigen::Vector2d& pos1_start, const Eigen::Vector2d& pos1_end, double radius1,
                         const Eigen::Vector2d& pos2_start, const Eigen::Vector2d& pos2_end, double radius2) {
    // Calculate the direction vectors for both robots
    Eigen::Vector2d direction1 = pos1_end - pos1_start;
    Eigen::Vector2d direction2 = pos2_end - pos2_start;
    
    // Calculate the relative velocity vector between the two robots
    Eigen::Vector2d relative_velocity = direction1 - direction2;
    
    // Calculate the initial distance between the two robots
    Eigen::Vector2d initial_distance = pos1_start - pos2_start;
    
    // Quadratic formula coefficients (relative velocity, initial distance, and radius sum)
    double a = relative_velocity.squaredNorm(); // Coefficient for t^2
    double b = 2 * initial_distance.dot(relative_velocity); // Coefficient for t
    double c = initial_distance.squaredNorm() - std::pow(radius1 + radius2, 2); // Constant term

    // Calculate the discriminant of the quadratic equation
    double discriminant = b * b - 4 * a * c;

    // If discriminant < 0, the robots don't collide
    if (discriminant < 0) {
        return false;
    }

    // Calculate the roots (times) when the robots collide
    double t1 = (-b - std::sqrt(discriminant)) / (2 * a);
    double t2 = (-b + std::sqrt(discriminant)) / (2 * a);

    // Check if any collision time is within the [0, 1] range (indicating a collision during the movement)
    if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) {
        return true;
    }

    // If no collision occurs within the movement timeframe, return false
    return false;
}



amp::Node find_closest_state_node(const std::map<amp::Node, Eigen::Vector2d>& nodes, const Eigen::Vector2d& sample) {
    amp::Node nearest_node = 0;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto& [node, state] : nodes) {
        double distance = (state - sample).norm();
        if (distance < min_distance) {
            min_distance = distance;
            nearest_node = node;
        }
    }
    return nearest_node;
}

// Find the closest node in the composed c-space
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
    constexpr double epsilon = 0.25;

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