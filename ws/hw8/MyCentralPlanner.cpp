#include "MyMultiAgentPlanners.h"

// Example function declarations
bool robot_in_polygons(const Eigen::Vector2d& sample, double radius, const amp::MultiAgentProblem2D& problem);
bool robot_in_robot(const Eigen::Vector2d& sample, double radius, const Eigen::Vector2d& current_state, double radius2);
//amp::Node find_closest_node(const std::map<amp::Node, std::vector<Eigen::Vector2d>>& nodes, const std::vector<Eigen::Vector2d>& samples);
amp::Node find_closest_node(const std::map<amp::Node, Eigen::VectorXd> &nodes, const Eigen::VectorXd &samples);
amp::Node find_closest_state_node(const std::map<amp::Node, Eigen::Vector2d>& nodes, const Eigen::Vector2d& sample);
bool will_robots_collide(const Eigen::Vector2d& pos1_start, const Eigen::Vector2d& pos1_end, double radius1, const Eigen::Vector2d& pos2_start, const Eigen::Vector2d& pos2_end, double radius2);
bool is_in_goal(const Eigen::VectorXd& current_state, const Eigen::VectorXd& goal_state, double epsilon);
bool is_agent_in_goal(const Eigen::Vector2d& current_state, const Eigen::Vector2d& goal_state, double epsilon);

amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    
    // Create a 2D path for multiple agents
    amp::MultiAgentPath2D path;

    // Number of agents    
    int num_agents = problem.numAgents(); 
    int num_states = 2 * num_agents; // Each agent has a x and y state
    int itr = 0;


    // Get the bounds of the workspace
    double x_max = problem.x_max;
    double x_min = problem.x_min;
    double y_max = problem.y_max;
    double y_min = problem.y_min;

    // Create a graph to store the nodes for each agent
    std::map<amp::Node, Eigen::VectorXd> nodes; 
    std::map<amp::Node, amp::Node> parent_map;
    amp::Node current_node;

    // Create a vector for initial states, goal states, current states, next states, and current samples
    Eigen::VectorXd initial_state(num_states), goal_state(num_states), current_state(num_states), next_state(num_states), sample(num_states);
    
    // Define the inital and goal states for each agent
    for (int i = 0; i < num_agents; i++) {
        initial_state[i * 2] = problem.agent_properties[i].q_init.x();
        initial_state[i * 2 + 1] = problem.agent_properties[i].q_init.y();
        goal_state[i * 2] = problem.agent_properties[i].q_goal.x();
        goal_state[i * 2 + 1] = problem.agent_properties[i].q_goal.y();
        path.agent_paths.push_back(amp::Path2D());
    }

    // Let the current state be the initial state
    current_state = initial_state;

    // Define the first node to have initial state and parent of -1
    nodes[0] = current_state;
    parent_map[0] = -1;
    current_node = 1;

    // RRT algorithm
    while (itr < max_itr) {

        if (is_in_goal(current_state, goal_state, epsilon)) {
            std::cout << "Reached goal state!" << std::endl;

            for (int i = 0; i < num_agents; i++) {
                std::cout << "Agent " << i << "  State: " << current_state[i * 2] << ", " << current_state[i * 2 + 1] << std::endl;
                amp::Node itr_node = current_node;

                // Add the goal state to the path
                int dont_loop = 0;
                path.agent_paths[i].waypoints.push_back(goal_state.segment<2>(i * 2));
                while (itr_node != -1 & dont_loop < 500) {
                    path.agent_paths[i].waypoints.push_back(nodes[itr_node].segment<2>(i * 2));
                    itr_node = parent_map[itr_node];
                    dont_loop++;
                }
                // Add the initial state to the path
                path.agent_paths[i].waypoints.push_back(initial_state.segment<2>(i * 2));
                std::reverse(path.agent_paths[i].waypoints.begin(), path.agent_paths[i].waypoints.end());
            }
            break; // Exit if the current state is in the goal
        }
        
        // Generate a random number to decide if sample the goal
        double rand_num = static_cast<double>(rand()) / RAND_MAX;

        // If the rand_num < goal bias, sample the goal
        if (rand_num < goal_bias) {
            sample = goal_state;
        } else {
            // Sample a random state in the workspace
            int valid_sample_for_agent = 0;
            while(valid_sample_for_agent < num_agents){
                // Sample 2 random x,y points in workspace and check if they are valid
                double x_sample = x_min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX/(x_max - x_min)));
                double y_sample = y_min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX/(y_max - y_min)));
                Eigen::Vector2d sample_point(x_sample, y_sample);

                // Check if the sample point is valid
                if (!robot_in_polygons(sample_point, problem.agent_properties[valid_sample_for_agent].radius, problem)) {
                    sample[valid_sample_for_agent * 2] = x_sample;
                    sample[valid_sample_for_agent * 2 + 1] = y_sample;
                    valid_sample_for_agent++;
                }
            }
        }

        // Find the closest node in the graph to the sample
        amp::Node nearest_node = find_closest_node(nodes, sample);
        Eigen::VectorXd nearest_state = nodes[nearest_node];

        // Calculate the direction to the sample and normalize it
        Eigen::VectorXd direction = sample - nearest_state;
        direction.normalize();

        // Create a new state by moving towards the sample
        next_state = nearest_state + direction * r;
        
        // Check if the next state is valid for all agents
        bool valid_next_state = true;
        for (int i = 0; i < num_agents; i++) {
            // Print the 2D coordinates of the next state
            Eigen::Vector2d next_state_point(next_state[i * 2], next_state[i * 2 + 1]);

            // Check if the next state is valid by first checking if it is inside a polygon
            if (robot_in_polygons(next_state_point, problem.agent_properties[i].radius, problem)) {
                valid_next_state = false;
                break;
            }

            // Check for collisions with other agents
            for (int j = 0; j < i; j++) {
                if (i != j) {
                    Eigen::Vector2d other_agent_state(next_state[j * 2], next_state[j * 2 + 1]);
                    if (will_robots_collide(current_state.segment<2>(i * 2), next_state.segment<2>(i * 2), problem.agent_properties[i].radius,
                                            current_state.segment<2>(j * 2), next_state.segment<2>(j * 2), problem.agent_properties[j].radius)) {
                        valid_next_state = false;
                        break;
                    }
                }
            }

            if (!valid_next_state) {
                break;
            }
        }

        // If the next state is NOT valid then skip to the next iteration
        if (!valid_next_state) {
            continue;
        }

        current_node  = nodes.size(); // Start from the last node
        current_state = next_state; // Update the current state
        nodes[current_node] = current_state; // Add the new state to the graph
        parent_map[current_node] = nearest_node; // Set the parent of the new node
        itr++;

        // Print everything about new update
        // std::cout << "Iteration: " << itr << std::endl;
        // for (int i = 0; i < num_agents; i++) {
        //     std::cout << "Agent " << i << "  State: " << current_state[i * 2] << ", " << current_state[i * 2 + 1] << std::endl;
        // }
        // std::cout << "Current Node: " << current_node << " Parent Node: " << parent_map[current_node - 1] << std::endl;

    } // End of RRT loop

    return path;
}

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
     // Create a 2D path for multiple agents
    amp::MultiAgentPath2D path;

    // Number of agents    
    int num_agents = problem.numAgents(); 

    // Get the bounds of the workspace
    double x_max = problem.x_max;
    double x_min = problem.x_min;
    double y_max = problem.y_max;
    double y_min = problem.y_min;

    // Create a vector of graphs that have the paths for all nodes
    std::vector<std::map<amp::Node, Eigen::Vector2d>> nodes(num_agents);
    std::vector<std::map<amp::Node, amp::Node>> parent_map(num_agents);
    std::vector<amp::Node> current_node(num_agents);

    // Create a vector for initial states, goal states, current states, next states, and current samples
    Eigen::Vector2d initial_state, goal_state, current_state, next_state, sample; // Each agent has a x and y state
    int stop = 0;
    for (int i = 0; i < num_agents; i++) {

        if(stop == num_agents){
            break;
        }
        stop++;

        initial_state = problem.agent_properties[i].q_init;
        goal_state = problem.agent_properties[i].q_goal;
        path.agent_paths.push_back(amp::Path2D());

        // Let the current state be the initial state
        current_state = initial_state;

        std::cout << "Planning for agent " << i << " with initial state: " << current_state.transpose() << " and goal state: " << goal_state.transpose() << std::endl;

        // Define the first node to have initial state and parent of -1
        nodes[i][0] = current_state;
        parent_map[i][0] = -1;

        current_node[i] = 1; // Set the current node for each agent

        // RRT algorithm for each agent
        int itr = 0;
        while (itr < max_itr) {

            if (is_agent_in_goal(current_state, goal_state, epsilon)) {
                std::cout << "Agent " << i << " Reached goal state!" << std::endl;
                amp::Node itr_node = current_node[i];
                path.agent_paths[i].waypoints.push_back(goal_state);
                while (itr_node != -1) {
                    path.agent_paths[i].waypoints.push_back(nodes[i][itr_node]);
                    itr_node = parent_map[i][itr_node];
                }
                std::reverse(path.agent_paths[i].waypoints.begin(), path.agent_paths[i].waypoints.end());
                break; // Exit if the current state is in the goal
            }

            // Generate a random number to decide if sample the goal
            double rand_num = static_cast<double>(rand()) / RAND_MAX;

            // If the rand_num < goal bias, sample the goal
            if (rand_num < goal_bias) {
                sample = goal_state;
            }
            else if (rand_num > goal_bias && rand_num < 0.1) {
                // Sample a left corner
                sample = Eigen::Vector2d(x_min, y_min);
            }
            else if(rand_num > 0.1 && rand_num < 0.2){
                // Sample a right corner
                sample = Eigen::Vector2d(x_max, y_min);
            }
            else if(rand_num > 0.2 && rand_num < 0.3){
                // Sample a top left corner
                sample = Eigen::Vector2d(x_min, y_max);
            }
            else if(rand_num > 0.3 && rand_num < 0.4){
                // Sample a top right corner
                sample = Eigen::Vector2d(x_max, y_max);
            }
            else {
                while(true){
                    // Sample a random state in the workspace
                    double x_sample = x_min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX/(x_max - x_min)));
                    double y_sample = y_min + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX/(y_max - y_min)));
                    sample = Eigen::Vector2d(x_sample, y_sample);

                    // Check if the sample point is valid
                    if (!robot_in_polygons(sample, problem.agent_properties[i].radius, problem)) {
                        break; // Exit the loop if the sample is valid
                    }
                }
            }

            amp::Node nearest_node = find_closest_state_node(nodes[i], sample);
            Eigen::Vector2d nearest_state = nodes[i][nearest_node];

            // Calculate the direction to the sample and normalize it
            Eigen::Vector2d direction = sample - nearest_state;
            direction.normalize();

            // Create a new state by moving towards the sample
            next_state = nearest_state + direction * r;

            // Check if the next state is valid for the agent
            if (robot_in_polygons(next_state, problem.agent_properties[i].radius, problem)) {
                continue; // Skip to the next iteration if the next state is invalid
            }

            // Check for collisions with other agents
            bool valid_next_state = true;
            for (int j = 0; j < num_agents; j++) {
                if (i != j) {
                    // Check if the next state is in collision with other robot start or goal
                    if (robot_in_robot(next_state, problem.agent_properties[i].radius, problem.agent_properties[j].q_init, problem.agent_properties[j].radius) ||
                        robot_in_robot(next_state, problem.agent_properties[i].radius, problem.agent_properties[j].q_goal, problem.agent_properties[j].radius)) {
                        valid_next_state = false;
                        break;
                    }

                    if (j < i) {
                        // Check along the entire path for collisions
                        for (int k = 0; k < path.agent_paths[j].waypoints.size(); k++) {
                            if (will_robots_collide(current_state, next_state, problem.agent_properties[i].radius,
                                                    path.agent_paths[j].waypoints[k], path.agent_paths[j].waypoints[k], problem.agent_properties[j].radius)) {
                                valid_next_state = false;
                                break;
                            }
                        }
                    }
                }

                if (!valid_next_state) {
                    break;
                }
            }
            // If the next state is NOT valid then skip to the next iteration
            if (!valid_next_state) {
                itr++;
                continue;
            }

            current_node[i] = nodes[i].size(); // Start from the last node
            current_state = next_state; // Update the current state
            nodes[i][current_node[i]] = current_state; // Add the new state to the graph
            parent_map[i][current_node[i]] = nearest_node; // Set the parent of the new node
            itr++;
        }

        if (itr == max_itr) {
            std::cout << "Agent " << i << " FAILED to reach goal after " << max_itr << " iterations." << std::endl;
            path.agent_paths[i].waypoints.push_back(initial_state); 
            path.agent_paths[i].waypoints.push_back(goal_state);
            break;
        } else {
            std::cout << "Agent " << i << " completed planning in " << itr << " iterations." << std::endl; 
        }
    }

    std::cout << "Finished planning for all agents!" << std::endl;

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

// Is robt in goal
bool is_agent_in_goal(const Eigen::Vector2d& current_state, const Eigen::Vector2d& goal_state, double epsilon) {
    return (current_state - goal_state).norm() < epsilon;
}  // End of is_in_goal function


// Is System in goal
bool is_in_goal(const Eigen::VectorXd& current_state, const Eigen::VectorXd& goal_state, double epsilon) {
    bool in_goal = true;

    for (int i = 0; i < current_state.size(); i++) {
        if (std::abs(current_state[i] - goal_state[i]) > epsilon) {
            in_goal = false;
            break;
        }
    }
    return in_goal;
}  // End of is_in_goal function

// 2D distance from a point to a line segment
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

// Finds the closest node in the composed c-space
amp::Node find_closest_node(const std::map<amp::Node, Eigen::VectorXd> &nodes, const Eigen::VectorXd &samples) {
    amp::Node nearest_node = 0;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto& [node, state] : nodes) {
        double distance = (state - samples).norm();
        if (distance < min_distance) {
            min_distance = distance;
            nearest_node = node;
        }
    }
    return nearest_node; // Return the nearest node
}

// Find the closest node in the composed c-space
amp::Node find_closest_node_orig(const std::map<amp::Node, std::vector<Eigen::Vector2d>>& nodes, const std::vector<Eigen::Vector2d>& samples) {
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