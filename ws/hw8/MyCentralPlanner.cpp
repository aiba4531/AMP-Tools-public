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

    // Get the bounds of the workspace
    double x_max = problem.x_max;
    double x_min = problem.x_min;
    double y_max = problem.y_max;
    double y_min = problem.y_min;

    // Create a vector to store the initial states of the agents
    std::vector<Eigen::Vector2d> initial_states, current_states, goal_states, sampled_states; // vector of (x,y) coordinates for each agent
    std::vector<bool> goal_reached; // Track if each agent has reached its goal
    std::map<amp::Node, amp::Node> parent_map; // Store parent nodes for meta agent
    std::map<amp::Node, std::vector<Eigen::Vector2d>> nodes; // Store nodes for meta agent

    // Define the iniital states and goal states for each agent
    for (const auto& agent : problem.agent_properties) {
        initial_states.push_back(agent.q_init);
        current_states.push_back(agent.q_init);
        sampled_states.push_back(agent.q_init);
        goal_states.push_back(agent.q_goal);
        goal_reached.push_back(false); // Initialize goal_reached for each agent
        path.agent_paths.push_back(amp::Path2D());
    }

    // Number of iterations
    int num_itr = 0, inside_num_itr = 0; 
    double x_rand, y_rand, radius;
    bool robot_robot_collision;
    Eigen::Vector2d sample;
    amp::Node curr_node;

    current_states = initial_states; // Set the current states to the initial states
    nodes[0] = initial_states; // Add the initial states to the nodes
    parent_map[0] = -1; // Set the parent of the initial node to -1

    // While the goal is not reached or a number of iterations is not reached
    while (num_itr < max_itr) {

        // For each agent
        for (int i = 0; i < num_agents; i++) {

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

        std::vector<Eigen::Vector2d> next_states(num_agents); // Compute next states for each agent
        for (int i = 0 ; i < num_agents; i++){
            // Calculate the direction to the sampled state
            Eigen::Vector2d direction = (sampled_states[i] - closest_node_state[i]).normalized();
            Eigen::Vector2d next_step = closest_node_state[i] + direction * r;

            if (robot_in_polygons(next_step, problem.agent_properties[i].radius, problem)) {
                next_states[i] = current_states[i]; // If in collision, stay at the current node
            } else {
                next_states[i] = next_step; // If no collisions, update the node
            }
        }

        // Try to have all robots move towards the sampled state from closest node
        for (int i = 0; i < num_agents; i++) {
            robot_robot_collision = false; // Check for robot-robot collision
            if (goal_reached[i]) {
                continue; // Skip to the next agent if the goal is already reached
            }

            // Check for collisions with other robots
            for (int j = 0; j < num_agents; j++) {
                if(i != j) {
                    if(will_robots_collide(current_states[i], next_states[i], problem.agent_properties[i].radius, current_states[j], next_states[j], problem.agent_properties[j].radius)) {
                        robot_robot_collision = true; // Set the collision flag
                        next_states[i] = current_states[i]; // If in collision, stay at the current node
                        break; // Skip this step if it is in collision with another robot
                    }
                }
            }
        }

        for (int i = 0; i < num_agents; i++) {
            if (goal_reached[i]) {
                current_states[i] = current_states[i];
            } else{
                current_states[i] = next_states[i];
            }
        }

        
        // Is ths goal reached for any agnets?
        bool all_goals_reached = true;
        for (int i = 0; i < num_agents; i++) {
            if(goal_reached[i] ) {
                continue; // Skip to the next agent if the goal is already reached
            }

            double distance = (current_states[i] - goal_states[i]).norm();
            if (distance < epsilon) {
                std::cout << "Goal reached for agent " << i << " --> Current state: " << current_states[i].transpose() << std::endl;        
                goal_reached[i] = true; // Mark the goal as reached
            }
            else {
                all_goals_reached = false; // At least one agent has not reached its goal
            }
        }

        // Now the current states are valid places to move all robots so store in the tree
        num_itr++;
        curr_node = num_itr; // Create a new node index
        nodes[curr_node] = current_states; // Add the new node to the graph
        parent_map[curr_node] = closest_node; // Store the parent of the new node

        std::cout << "Adding Node " << curr_node << " with states: ";
        for (const auto& state : current_states) {
            std::cout << state.transpose() << " | ";
        }

        std::cout << " that is connected to Node " << closest_node << " with states: ";
        for (const auto& state : nodes[closest_node]) {
            std::cout << state.transpose() << " | ";
        }

        std::cout << std::endl;

        if (all_goals_reached) {
            std::cout << "All goals reached!" << std::endl;
            amp::Node new_node = num_itr+1; // Create a new node index
            for (int i = 0; i < num_agents; i++) {
                current_states[i] = goal_states[i]; // Set the current state to the goal state
            }
            nodes[new_node] = current_states; // Add the new node to the graph
            parent_map[new_node] = curr_node; // Store the parent of the new node
            curr_node = new_node; // Update the current node
            break;
        }

    }


    std::cout << "Number of iterations: " << num_itr << std::endl;
    for (amp::Node j = curr_node; j != -1; j = parent_map[j]) {
        // Reconstruct the path for each agent
        for (int i = 0; i < num_agents; i++) {
            path.agent_paths[i].waypoints.insert(path.agent_paths[i].waypoints.begin(), nodes[j][i]);          
        }
    }

    for (int i = 0; i < num_agents; i++) {
        for (int j = 0; j < path.agent_paths[i].waypoints.size(); j++) {
            std::cout << "Waypoint " << j << ": " << path.agent_paths[i].waypoints[j].transpose() << std::endl;
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