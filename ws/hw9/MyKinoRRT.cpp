#include "MyKinoRRT.h"
bool point_in_polygons(const Eigen::Vector2d& point, const amp::KinodynamicProblem2D& problem);
amp::Node find_closest_node(const std::map<amp::Node, Eigen::VectorXd> &nodes, const Eigen::VectorXd &samples);
std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> get_all_primitives(const amp::KinodynamicProblem2D& problem);
bool line_segment_in_polygon(const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step);

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
};

void MyFirstOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state[0] += dt * control[0] * std::cos(state[2]);
    state[1] += dt * control[0] * std::sin(state[2]);
    state[2] += dt * control[1];
};

// amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
//     amp::KinoPath path;
//     Eigen::VectorXd state = problem.q_init;
//     path.waypoints.push_back(state);
//     for (int i = 0; i < 10; i++) {
//         Eigen::VectorXd control = Eigen::VectorXd::Random(problem.q_init.size());
//         agent.propagate(state, control, 1.0);
//         path.waypoints.push_back(state);
//         path.controls.push_back(control);
//         path.durations.push_back(1.0);
//     }
//     path.valid = true;
//     return path;
// }

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    amp::KinoPath path;

    // Get the bounds of the workspace
    double x_min = problem.x_min;
    double x_max = problem.x_max;
    double y_min = problem.y_min;
    double y_max = problem.y_max;

    // Get all linear primitives
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives = get_all_primitives(problem);

    // Get the bounds of the control space [u_min, u_max]
    std::vector<std::pair<double, double>> control_space = problem.u_bounds;
    double num_control_inputs = control_space.size();

    // Get the bounds of the time space [t_min, t_max]
    std::pair<double, double> time_space = problem.dt_bounds;
    double dt = time_space.second;

    // Create a tree to store the nodes
    potential_states = std::map<amp::Node, Eigen::VectorXd>();
    potential_controls = std::map<amp::Node, Eigen::VectorXd>();
    potential_duration = std::map<amp::Node, double>();
    parent_map = std::map<amp::Node, amp::Node>();
    amp::Node new_node;

    // Create a vector for initial states
    Eigen::VectorXd initial_state = problem.q_init;
    int number_of_states = initial_state.size();

    // Create a vector for the goal state
    std::vector<std::pair<double, double>> goal_vector = problem.q_goal;
    Eigen::VectorXd goal_state = Eigen::VectorXd::Zero(number_of_states);
    for (int i = 0; i < number_of_states; i++){
        goal_state[i] = goal_vector[i].first;
    }

    // Add the initial state to the potential states and set its parent
    potential_states[0] = initial_state;
    //potential_controls[0] = Eigen::VectorXd::Zero(num_control_inputs);
    //potential_duration[0] = 0;
    parent_map[0] = -1;
    

    int current_itr = 0;
    Eigen::VectorXd sample = Eigen::VectorXd::Zero(number_of_states);
    Eigen::Vector2d robot_position;


    while (current_itr < max_itr){

       // Sample a valid ranodm state in the workspace
       while(true){
            // Sample a random state
            double sample_x = x_min + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(x_max - x_min)));
            double sample_y = y_min + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(y_max - y_min)));
            sample[0] = sample_x;
            sample[1] = sample_y;

            // Check if the sample point is not in an obstacle
            robot_position = sample.segment<2>(0);

            if (!point_in_polygons(robot_position, problem)){
                    break;
            }
        }

        // Find the nearest node to the sample
        amp::Node nearest_node = find_closest_node(potential_states, sample);

        // Generate Local Trajectory from nearest state to sample using random controls
        Eigen::VectorXd nearest_state = potential_states[nearest_node];

        // Loop 10 times through the control space
        int itr = 0;
        std::vector<Eigen::VectorXd> random_potential_states;
        std::vector<Eigen::VectorXd> random_potential_controls;
        Eigen::VectorXd control = Eigen::VectorXd::Zero(num_control_inputs);
        while (itr < 10){
            // Sample a random control
            double control_0 = control_space[0].first + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(control_space[0].second - control_space[0].first)));
            double control_1 = control_space[1].first + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(control_space[1].second - control_space[1].first)));
            control[0] = control_0;
            control[1] = control_1;
            Eigen::VectorXd new_state = nearest_state;

            agent.propagate(new_state, control, dt);

            Eigen::Vector2d new_robot_position = new_state.segment<2>(0);
            Eigen::Vector2d previous_robot_position = nearest_state.segment<2>(0);

            // Check if the new state is valid
            if (point_in_polygons(new_state.segment<2>(0), problem)){
                continue;
            }

            // Check if the new state is in collision with the obstacles
            std::tuple<Eigen::Vector2d, Eigen::Vector2d> line_segment = std::make_tuple(previous_robot_position, new_robot_position - previous_robot_position);
            if (line_segment_in_polygon(all_primitives, line_segment)){
                continue;
            }

            // Add the new state to the potential states
            random_potential_states.push_back(new_state);
            random_potential_controls.push_back(control);
            itr++;
        }

        // Find the best state from the potential states
        double min_distance = std::numeric_limits<double>::max();
        Eigen::VectorXd best_state;
        Eigen::VectorXd best_control;
        for (int i = 0; i < random_potential_states.size(); i++){
            double distance = (random_potential_states[i] - sample).norm();
            if (distance < min_distance){
                min_distance = distance;
                best_state = random_potential_states[i];
                best_control = random_potential_controls[i];
            }
        }

        // Add the best state to the nodes
        // new_node = potential_states.size();
        // potential_states[new_node] = best_state;
        // potential_controls[new_node] = best_control;
        // potential_duration[new_node] = potential_duration[nearest_node] + dt;
        // parent_map[new_node] = nearest_node;
        // current_itr++;

        new_node = potential_states.size();
        potential_states[new_node] = best_state;
        potential_controls[nearest_node] = best_control;
        potential_duration[new_node] = dt;
        parent_map[new_node] = nearest_node;
        current_itr++;


        // Check if the best state is close to the goal state
        if ((best_state - goal_state).norm() < epsilon){

            // Get the previous node
            amp::Node prev_node = new_node;
            
            // Get the new node
            new_node = potential_states.size();

            // Define a vector equal to the best state
            Eigen::VectorXd final_state = best_state;

            // Get the goal control
            Eigen::VectorXd goal_control = (goal_state - final_state) * 1/dt;

            // Propagate the agent to the goal state
            agent.propagate(final_state, goal_control, dt);

            potential_states[new_node] = final_state;
            potential_controls[prev_node] = goal_control;
            potential_duration[new_node] = dt;
            parent_map[new_node] = prev_node;
            potential_controls[new_node] = Eigen::VectorXd::Zero(num_control_inputs);

            std::cout << "GOAL! At iteration " << current_itr << " the final state is: " << final_state[0] << ", " << final_state[1] << " with control: " << goal_control[0] << ", " << goal_control[1] << std::endl;
            break;
        }

        std::cout << "At iteration " << current_itr << " the best state is: " << best_state[0] << ", " << best_state[1] << " with control: " << best_control[0] << ", " << best_control[1] << std::endl;
    }

    // Construct the path
    amp::Node current_node = new_node;
    while (current_node != -1){
        path.waypoints.push_back(potential_states[current_node]);
        path.controls.push_back(potential_controls[current_node]);
        path.durations.push_back(potential_duration[current_node]);
        current_node = parent_map[current_node];
    }

    // Reverse the path
    std::reverse(path.waypoints.begin(), path.waypoints.end());
    std::reverse(path.controls.begin(), path.controls.end());
    std::reverse(path.durations.begin(), path.durations.end());

    // Print the path and controls
    for (int i = 0; i < path.waypoints.size(); i++) {
        std::cout << std::fixed << std::setprecision(3); // Set fixed-point notation and precision to 3 digits
        std::cout << "Waypoint " << i << " at " << path.waypoints[i][0] << ", " << path.waypoints[i][1]
                << " with control: " << path.controls[i][0] << ", " << path.controls[i][1] << " at time " << path.durations[i] << std::endl;
    }

    // for (int i = 0; i < path.waypoints.size(); i++) {
    //     std::cout << std::fixed << std::setprecision(3); // Set fixed-point notation and precision to 3 digits
    //     std::cout << i << ", " << path.waypoints[i][0] << ", " << path.waypoints[i][1]
    //             << ", " << path.controls[i][0] << ", " << path.controls[i][1] << std::endl;
    // }



    path.valid = true;
    return path;
}

// Line segment polygon intersection algorithm
bool line_segment_in_polygon(const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step){
    
    // Check for collisions with all obstacle primitives
    for (int i = 0; i < all_primitives.size(); i++) {
        for (int j = 0; j < all_primitives[i].size(); j++) {

            std::tuple<Eigen::Vector2d, Eigen::Vector2d> primitive = all_primitives[i][j];
            // Get intersection of two line segments
            // Get the two vectors
            Eigen::Vector2d vector1 = std::get<1>(primitive);
            Eigen::Vector2d vector2 = std::get<1>(next_step);

            // Get the two points
            Eigen::Vector2d point1 = std::get<0>(primitive);
            Eigen::Vector2d point2 = std::get<0>(next_step);

            // Get the intersection point
            Eigen::Vector2d intersection = point1 - point2;

            // Get the determinant
            double det = vector1[0]*vector2[1] - vector1[1]*vector2[0];

            // Get the intersection point
            double t = (vector2[0]*intersection[1] - vector2[1]*intersection[0]) / det;
            double u = (vector1[0]*intersection[1] - vector1[1]*intersection[0]) / det;


            // If the intersection point is between 0 and 1, then the vectors intersect
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
                return true;
            }
        }
    }
    return false;
}

// Define all linear primitives for each obstacle
std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> get_all_primitives(const amp::KinodynamicProblem2D& problem) {
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives;
    for (int i = 0; i < problem.obstacles.size(); i++) {
        std::vector<Eigen::Vector2d> verticies = problem.obstacles[i].verticesCW();
        std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>> primitives;
        
        for (int j = 0; j < verticies.size(); j++) {
            if (j == verticies.size() - 1) {
                primitives.push_back(std::make_tuple(verticies[j], verticies[0]-verticies[j]));
                continue;
            }
            primitives.push_back(std::make_tuple(verticies[j], verticies[j+1]-verticies[j]));
        }
        all_primitives.push_back(primitives);
    }
    return all_primitives;
}


// Finds the closest node to the sampled state
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


// Checking if a point is inside a polygon
bool point_in_polygons(const Eigen::Vector2d& point, const amp::KinodynamicProblem2D& problem) {   
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
