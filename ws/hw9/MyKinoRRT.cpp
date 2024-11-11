#include "MyKinoRRT.h"
bool point_in_polygons(const Eigen::Vector2d& point, const amp::KinodynamicProblem2D& problem);
amp::Node find_closest_node(const std::map<amp::Node, Eigen::VectorXd> &nodes, const Eigen::VectorXd &samples);
std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> get_all_primitives(const amp::KinodynamicProblem2D& problem, double expansion_factor);
bool line_segment_in_polygon(const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step);
void propagate_agent(const amp::KinodynamicProblem2D& problem, Eigen::VectorXd& state, Eigen::VectorXd& control, double dt);
bool line_segment_with_rotation_in_polygon(const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives, const Eigen::VectorXd& rear_axle_center_position, double robot_angle, const amp::KinodynamicProblem2D& problem);
bool next_step_or_control_out_of_bounds(const Eigen::VectorXd new_state, const Eigen::Vector2d& next_step, Eigen::VectorXd& control ,const amp::KinodynamicProblem2D& problem, double scaling_factor = 1.0);
bool car_next_step_or_control_out_of_bounds(const Eigen::VectorXd new_state, const Eigen::Vector2d& rear_axle_center_position, double robot_angle, Eigen::VectorXd& control ,const amp::KinodynamicProblem2D& problem, double scaling_factor);

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
};

void MyFirstOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // State is [x, y, theta]
    // Control is [angular velocity, rotational velcoity]
    const double r = 0.25;

    double num_steps = 200;
    double new_dt = dt / num_steps;

    for (int i = 0; i < num_steps; i++) {
        state[0] += new_dt * control[0] * r * std::cos(state[2]);
        state[1] += new_dt * control[0] * r * std::sin(state[2]);
        state[2] += new_dt * control[1];
    }

};

void MySecondOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    // State is [x, y, theta, v, w]
    // Control is [a, alpha]
    const double r = 0.25;

    double num_steps = 200;
    double new_dt = dt / num_steps;

    for (int i = 0; i < num_steps; i++) {
        state[0] += new_dt * state[3] * r * std::cos(state[2]);
        state[1] += new_dt * state[3] * r * std::sin(state[2]);
        state[2] += new_dt * state[4];
        state[3] += new_dt * control[0];
        state[4] += new_dt * control[1];
    }
};

void MySimpleCar::propagate_car(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt, const amp::KinodynamicProblem2D& problem) {
    // State is [x, y, theta, v, phi]
    // Control is [v-dot, phi-dot]
    double L = 5;//get_length(problem);
    double num_steps = 200;
    double new_dt = dt / num_steps;

    for (int i = 0; i < num_steps; i++) {
        state[0] += new_dt * state[3] * std::cos(state[2]);
        state[1] += new_dt * state[3] * std::sin(state[2]);
        state[2] += new_dt * state[3] * std::tan(state[4]) / L;
        state[3] += new_dt * control[0];
        state[4] += new_dt * control[1];
    }
};
double MySimpleCar::get_length(const amp::KinodynamicProblem2D& problem){
    return problem.agent_dim.length;
}



// Implement the plan function
amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    amp::KinoPath path;

    std::vector<std::pair<double, double>> state_space_bounds = problem.q_bounds;
    double num_states = state_space_bounds.size();

    // Get all linear primitives
    double expansion_factor = 0.0875; // set this to expand the obstacle verticies
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives = get_all_primitives(problem, expansion_factor);

    // Get the bounds of the control space [u1_min, u1_max]
    std::vector<std::pair<double, double>> control_space = problem.u_bounds;
    double num_control_inputs = control_space.size();
    

    // Get the bounds of the time space [t_min, t_max]
    std::pair<double, double> time_space = problem.dt_bounds;
    // double dt = time_space.second;

    // Get the bounds of the goal space [x1_goal_min, x1_goal_min]
    std::vector<std::pair<double, double>> goal_space = problem.q_goal;

    // Create a 3 trees and parent map to store the nodes
    potential_states = std::map<amp::Node, Eigen::VectorXd>();
    potential_controls = std::map<amp::Node, Eigen::VectorXd>();
    potential_duration = std::map<amp::Node, double>();
    parent_map = std::map<amp::Node, amp::Node>();
    
    // Define a new node to access anywhere
    amp::Node new_node;

    // Create a vector for initial states
    Eigen::VectorXd initial_state = problem.q_init;

    // Add the potential states to be the initial state with no control over 0 duration
    potential_states[0] = initial_state;
    potential_duration[0] = 0.0;
    potential_controls[0] = Eigen::VectorXd::Zero(num_control_inputs);

    // Add the parent map to be -1
    parent_map[0] = -1;
    
    // Define a vector to hold a random sampled state
    Eigen::VectorXd sample = Eigen::VectorXd::Zero(num_states);
    
    // 2D Vector to check if point in polygon
    Eigen::Vector2d robot_position;

    int current_itr = 0;
    while (current_itr < max_itr){
        // Generate a valid sample
        double sample_goal = amp::RNG::randf(0.0, 1.0);

        if (sample_goal < goal_bias){
            // Sample the goal state
            for (int i = 0; i < num_states; i++){
                sample[i] = amp::RNG::randf(goal_space[i].first, goal_space[i].second);
            }
        }
        else {
            // Sample a valid ranodm state in the workspace
            int num_sampled_points = 0;
            int max_sampled_points = 1000;
            while(num_sampled_points < max_sampled_points){
                // Sample a random state
                for (int i = 0; i < num_states; i++){
                    sample[i] = amp::RNG::randf(state_space_bounds[i].first, state_space_bounds[i].second);
                }            

                // Check if the sample point is not in an obstacle
                robot_position = sample.segment<2>(0);

                if (!point_in_polygons(robot_position, problem)){ // Found a valid position to go towards
                        break;
                }
                num_sampled_points++;
            }
        }
        // Find the nearest node to the sample
        amp::Node nearest_node = find_closest_node(potential_states, sample);                            

        // Generate Local Trajectory from nearest state to sample using random controls
        Eigen::VectorXd nearest_state = potential_states[nearest_node];

        // Create a vector to store the potential states from the random controls
        std::vector<Eigen::VectorXd> random_potential_states;
        std::vector<Eigen::VectorXd> random_potential_controls;

        // Create a vector to each new state and control
        Eigen::VectorXd new_state;
        Eigen::Vector2d new_robot_position, previous_robot_position;
        double new_robot_angle, previous_robot_angle;

        // Create a vector to store the potential random controls
        Eigen::VectorXd control = Eigen::VectorXd::Zero(num_control_inputs);
        int num_sampled_controls = 0;
        //int max_sampled_controls = 5;
        while (num_sampled_controls < max_sampled_controls){
            // Sample a random control
            for (int i = 0; i < num_control_inputs; i++){
                control[i] = amp::RNG::randf(control_space[i].first, control_space[i].second);
            }

            // Propagate the agent
            new_state = nearest_state;
            propagate_agent(problem, new_state, control, dt);

            // Check if the new state is valid
            new_robot_position = new_state.segment<2>(0);
            new_robot_angle = new_state[2];
            previous_robot_position = nearest_state.segment<2>(0);
            double scaling_factor = 0.99; // set this to scale the bounds
            
            if (problem.isPointAgent){ // If the robot is a point agent

                // If center of robot is in the polygon, find a new control
                if (point_in_polygons(new_robot_position, problem)){
                    num_sampled_controls++;
                    continue;
                }

                // If the next step or control is out of bounds, find a new control
                if (next_step_or_control_out_of_bounds(new_state, new_robot_position, control, problem, scaling_factor)){
                    num_sampled_controls++;
                    continue;
                }          
                // Check if the line between the previous and new state is in collision with the obstacles
                std::tuple<Eigen::Vector2d, Eigen::Vector2d> line_segment = std::make_tuple(previous_robot_position, new_robot_position - previous_robot_position);
                if (line_segment_in_polygon(all_primitives, line_segment)){
                    num_sampled_controls++;
                    continue;
                }
            }
            else { // The robot is a car agent and new robot position is center of rear axle

                // If center of robot is in the polygon, find a new control
                if (point_in_polygons(new_robot_position, problem)){
                    num_sampled_controls++;
                    continue;
                }

                // If the next step or control is out of bounds, find a new control
                if (car_next_step_or_control_out_of_bounds(new_state, new_robot_position, new_robot_angle, control, problem, scaling_factor)){
                    num_sampled_controls++;
                    continue;
                }

                // If the line between the previous and new state is in collision with the obstacles, find a new control
                if (line_segment_in_polygon(all_primitives, std::make_tuple(previous_robot_position, new_robot_position - previous_robot_position))){
                    num_sampled_controls++;
                    continue;
                }

                // Check if the line segments that define the new car position are in collision with the obstacles
                if (line_segment_with_rotation_in_polygon(all_primitives, new_robot_position, new_robot_angle, problem)){                    
                    num_sampled_controls++;
                    continue;
                }
            }
            // If it made it here, then the new state and controls are valid
            random_potential_states.push_back(new_state);
            random_potential_controls.push_back(control);
            num_sampled_controls++;
        }

        // After sampling sufficient random controls, find the best state and control
        if (random_potential_states.size() > 0){
            // Find the best state from the potential states
            double min_distance = std::numeric_limits<double>::max();
            Eigen::VectorXd best_state;
            Eigen::VectorXd best_control;
            for (int i = 0; i < random_potential_states.size(); i++){
                // Grab the n-Dim distance
                double distance = (random_potential_states[i] - sample).norm();
                if (distance < min_distance){
                    min_distance = distance;
                    best_state = random_potential_states[i];
                    best_control = random_potential_controls[i];
                }
            }

            // Add the best sampled state to the potential nodes
            new_node = potential_states.size();
            potential_states[new_node] = best_state;
            potential_controls[new_node] = best_control;
            potential_duration[new_node] = dt;
            parent_map[new_node] = nearest_node;
            current_itr++;
            
            // Does the new state reach the goal?
            bool goal_reached = true;
            for (int i = 0; i < num_states; i++) {
                if (best_state[i] < goal_space[i].first || best_state[i] > goal_space[i].second) {
                    goal_reached = false;
                    break;
                }
            }

            if (goal_reached){ // flag was never set to false, so inside goal region
                amp::Node previous_node = new_node;
                new_node = potential_states.size();

                // Apply zero control over zero duration to keep robot from moving
                potential_controls[new_node] = Eigen::VectorXd::Zero(num_control_inputs);
                potential_duration[new_node] = 0.0;
                parent_map[new_node] = previous_node;
                break;
            }
        }
        else { // No valid states were found try to sample a new point          
            current_itr++;
        }
    }

    if (current_itr == max_itr){
        std::cout << "Max iterations reached!" << std::endl;
        path.valid = false;
        return path;
    }

    // Construct the path
    std::cout << "After " << current_itr << " iterations, received a valid path!" << std::endl;
    amp::Node current_node = new_node;
    while (current_node != 0){
        path.controls.push_back(potential_controls[current_node]);
        path.durations.push_back(potential_duration[current_node]);
        current_node = parent_map[current_node];
    }

    // Reverse the path
    std::reverse(path.waypoints.begin(), path.waypoints.end());
    std::reverse(path.controls.begin(), path.controls.end());
    std::reverse(path.durations.begin(), path.durations.end());

    // Pretend like we start at initial point and propagate the controls
    Eigen::VectorXd current_state = problem.q_init;
    Eigen::VectorXd previous_state = current_state;

    // Propagate the agent along the path
    path.waypoints.push_back(current_state);
    for (int i = 0; i < path.controls.size(); i++){
        previous_state = current_state;
        propagate_agent(problem, current_state, path.controls[i], path.durations[i]);
        path.waypoints.push_back(current_state);
        path_length += (current_state.segment<2>(0) - previous_state.segment<2>(0)).norm();
    }
    //std::cout << "Predicted end point of path: " << current_state[0] << ", " << current_state[1] << " with path length: " << path_length << std::endl;
    path.valid = true;
    return path;
}

// Write a function to check what the agent type is and call the correct propagate function
void propagate_agent(const amp::KinodynamicProblem2D& problem, Eigen::VectorXd& state, Eigen::VectorXd& control, double dt){
    if (problem.agent_type == amp::AgentType::SingleIntegrator){
        MySingleIntegrator agent;
        agent.propagate(state, control, dt);
    }
    else if (problem.agent_type == amp::AgentType::FirstOrderUnicycle){
        MyFirstOrderUnicycle agent;
        agent.propagate(state, control, dt);
    }
    else if (problem.agent_type == amp::AgentType::SecondOrderUnicycle){
        MySecondOrderUnicycle agent;
        agent.propagate(state, control, dt);
    }
    else if (problem.agent_type == amp::AgentType::SimpleCar){
        MySimpleCar agent;
        agent.propagate_car(state, control, dt, problem);
    }
}


// Check if any of the car line segments are in collision with the obstacles
bool line_segment_with_rotation_in_polygon(const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives, const Eigen::VectorXd& rear_axle_center_position, double robot_angle, const amp::KinodynamicProblem2D& problem) {
    
    // Check if the car vertices are in bounds
    double half_length = problem.agent_dim.length/2;  // Half of the robot's length
    double half_width = problem.agent_dim.width/2;    // Half of the robot's width
    

    // Define the four corners of the rectangle in the robot's local frame
    std::array<Eigen::Vector2d, 4> corners_local = {
        Eigen::Vector2d(0, -half_width),
        Eigen::Vector2d(0, half_width),
        Eigen::Vector2d(2*half_length, half_width),
        Eigen::Vector2d(2*half_length, -half_width)
    };

    // Rotate corners into the global frame and create line segments for each side
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> car_segments;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector2d corner1 = corners_local[i];
        Eigen::Vector2d corner2 = corners_local[(i + 1) % 4];

        // Rotate and translate corners into the global frame
        Eigen::Vector2d point1(
            std::cos(robot_angle) * corner1.x() - std::sin(robot_angle) * corner1.y() + rear_axle_center_position.x(),
            std::sin(robot_angle) * corner1.x() + std::cos(robot_angle) * corner1.y() + rear_axle_center_position.y()
        );
        Eigen::Vector2d point2(
            std::cos(robot_angle) * corner2.x() - std::sin(robot_angle) * corner2.y() + rear_axle_center_position.x(),
            std::sin(robot_angle) * corner2.x() + std::cos(robot_angle) * corner2.y() + rear_axle_center_position.y()
        );

        car_segments.push_back({point1, point2});
    }

    // Check each car segment against all obstacle primitives
    for (const auto& car_segment : car_segments) {
        Eigen::Vector2d car_start = car_segment.first;
        Eigen::Vector2d car_end = car_segment.second;
        Eigen::Vector2d car_vec = car_end - car_start;

        // std::cout << "Car Segment: " << car_start.x() << ", " << car_start.y() << " with Car Vector: " << car_vec.x() << ", " << car_vec.y() << std::endl;

        for (int i = 0; i < all_primitives.size(); i++) {
            for (int j = 0; j < all_primitives[i].size(); j++) {

                std::tuple<Eigen::Vector2d, Eigen::Vector2d> primitive = all_primitives[i][j];
                Eigen::Vector2d vector1 = std::get<1>(primitive);
                Eigen::Vector2d vector2 = car_vec;

                Eigen::Vector2d point1 = std::get<0>(primitive);
                Eigen::Vector2d point2 = car_start;

                Eigen::Vector2d intersection = point1 - point2;

                double det = vector1[0] * vector2[1] - vector1[1] * vector2[0];

                double t = (vector2[0] * intersection[1] - vector2[1] * intersection[0]) / det;
                double u = (vector1[0] * intersection[1] - vector1[1] * intersection[0]) / det;

                if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
                    // std::cout << "Collision detected with obstacle primitive: " <<  point1.x() << ", " << point1.y() << " with vector " << vector1.x() << ", " << vector1.y() << std::endl;
                    return true;
                }
            }
        }
    }

    return false;
}


// Check if the next step or control is out of bounds
bool next_step_or_control_out_of_bounds(const Eigen::VectorXd new_state, const Eigen::Vector2d& next_step, Eigen::VectorXd& control ,const amp::KinodynamicProblem2D& problem, double scaling_factor) {
    // Check if the next step is out of bounds
    for (int i = 0; i < new_state.size(); i++) {
        if (new_state[i] < scaling_factor*problem.q_bounds[i].first || new_state[i] > scaling_factor*problem.q_bounds[i].second) {
            return true;
        }
    }

    for (int i = 0; i < next_step.size(); i++) {
        if (next_step[i] < scaling_factor*problem.q_bounds[i].first || next_step[i] > scaling_factor*problem.q_bounds[i].second) {
            return true;
        }
    }


    for (int i = 0; i < control.size(); i++) {
        if (control[i] < scaling_factor*problem.u_bounds[i].first || control[i] > scaling_factor*problem.u_bounds[i].second) {
            return true;
        }
    }
    return false;
}


// Check if next step or control is out of bounds
bool car_next_step_or_control_out_of_bounds(const Eigen::VectorXd new_state, const Eigen::Vector2d& rear_axle_center_position, double robot_angle, Eigen::VectorXd& control, const amp::KinodynamicProblem2D& problem, double scaling_factor) {
    // Check if the next step is out of bounds
    for (int i = 0 ; i < new_state.size(); i++){
        if (new_state[i] < scaling_factor*problem.q_bounds[i].first || new_state[i] > scaling_factor*problem.q_bounds[i].second){
            return true;
        }
    }


    for (int i = 0; i < rear_axle_center_position.size(); i++) {
        if (rear_axle_center_position[i] < scaling_factor*problem.q_bounds[i].first || rear_axle_center_position[i] > scaling_factor*problem.q_bounds[i].second) {
            return true;
        }
    }

    // Check if the control is out of bounds
    for (int i = 0; i < control.size(); i++) {
        if (control[i] < scaling_factor*problem.u_bounds[i].first || control[i] > scaling_factor*problem.u_bounds[i].second) {
            return true;
        }
    }

    // Check if the car vertices are in bounds
    double half_length = problem.agent_dim.length/2;  // Half of the robot's length
    double half_width = problem.agent_dim.width/2;    // Half of the robot's width
    
    // Define the four corners of the rectangle in the robot's local frame which is center rear axel
    std::array<Eigen::Vector2d, 4> corners_local = {
        Eigen::Vector2d(0, -half_width),
        Eigen::Vector2d(0, half_width),
        Eigen::Vector2d(2*half_length, half_width),
        Eigen::Vector2d(2*half_length, -half_width)
    };

    // Rotate corners into the global frame and create line segments for each side
    for (int i = 0; i < 4; i++) {
        Eigen::Vector2d corner1 = corners_local[i];

        // Rotate and translate corners into the global frame
        Eigen::Vector2d point1(
            std::cos(robot_angle) * corner1.x() - std::sin(robot_angle) * corner1.y() + rear_axle_center_position.x(),
            std::sin(robot_angle) * corner1.x() + std::cos(robot_angle) * corner1.y() + rear_axle_center_position.y()
        );

        if (point1.x() < scaling_factor*problem.q_bounds[0].first || point1.x() > scaling_factor*problem.q_bounds[0].second || point1.y() < scaling_factor*problem.q_bounds[1].first || point1.y() > scaling_factor*problem.q_bounds[1].second) {
            return true;
        }
 
    }
    
    return false;
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


// Get all the primitives for the obstacles
std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> get_all_primitives(const amp::KinodynamicProblem2D& problem, double expansion_factor) {
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives;
    for (const auto& obstacle : problem.obstacles) {
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCW();
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();

        // Calculate the centroid
        for (const auto& vertex : vertices) {
            centroid += vertex;
        }
        centroid /= vertices.size();

        // Expand each vertex outward from the centroid
        for (auto& vertex : vertices) {
            Eigen::Vector2d direction = vertex - centroid;
            vertex += direction * expansion_factor;
        }

        std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>> primitives;
        for (size_t j = 0; j < vertices.size(); ++j) {
            Eigen::Vector2d start = vertices[j];
            Eigen::Vector2d end = (j == vertices.size() - 1) ? vertices[0] : vertices[j + 1];
            primitives.push_back(std::make_tuple(start, end - start));
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
    constexpr double epsilon = 0;//1e-9; // Small tolerance for floating-point errors

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
