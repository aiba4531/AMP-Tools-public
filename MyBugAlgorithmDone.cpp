#include "MyBugAlgorithm.h"
// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;

    // Start with the initial point
    path.waypoints.push_back(problem.q_init);

    // Set the step size
    double step_size = 0.25;
    
    // Get a vector that holds the linear primitives for every obsticle
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives = get_all_primitives(problem);
    
    // Create vectors to store hit and leave points
    std::vector<std::tuple<Eigen::Vector2d, double>> leave_points;
    std::vector<std::tuple<Eigen::Vector2d, double>> hit_points;

    Eigen::Vector2d current_point, rel_next_point, next_point;

    // Get the original goal vector and distance
    Eigen::Vector2d goalVector = (problem.q_goal - path.waypoints.back()).normalized();
    double orig_distance = (problem.q_goal - path.waypoints.back()).norm();

    int max_itr, max_itr2, max_itr3 = 0;
    while (max_itr < 10000) {
    
    // Jump Statement if goal can be reached after any iteration
    Move_to_goal:
        if (path.waypoints.back() == problem.q_goal) {
            break;
        }

        // Get the current point
        current_point = path.waypoints.back();
        
        // Try to move towards the goal from current point
        goalVector = (problem.q_goal - current_point).normalized();
        rel_next_point = goalVector * step_size;
        next_point = current_point + rel_next_point;

        // Define a vector to hold all edges that could intersect with the movement to next point
        std::vector<std::tuple<bool, std::tuple<int, int>>> intersecting_primitive_tuple_vector;
        
        // Define the tuple that holds the last intersecting primitive :: CHECK THIS 
        std::tuple<bool, std::tuple<int, int>> intersecting_primitive_tuple;
        std::tuple<int, int> intersecting_primitive;
        bool safe_next_point = true;

        // Does this next step intersect with any primitive?
        intersecting_primitive_tuple_vector = next_step_collision(all_primitives, std::make_tuple(current_point, rel_next_point));
        
        // Get the last inertsecting primitive
        intersecting_primitive_tuple = intersecting_primitive_tuple_vector.back();

        // If the next step intersects with a primitive, get the primitive it instersects with
        if (std::get<0>(intersecting_primitive_tuple) == false) {
            safe_next_point = false;
            intersecting_primitive = std::get<1>(intersecting_primitive_tuple);
        }
       
        // If safe, move to the next point
        if (safe_next_point) {
            path.waypoints.push_back(next_point);
        }
        // Otherwise, start wall-following 
        else {    
            // Define the hit point
            Eigen::Vector2d hit_point = current_point;

            // Define the distance from the hit point to the goal
            double distance_hit_point_to_goal = (problem.q_goal - hit_point).norm();
            
            // Define the current Distance to goal
            double current_distance_goal = distance_hit_point_to_goal;

            // Define the intersecting obstacle index
            int i = std::get<0>(intersecting_primitive); 
            
            // Define the intersecting edge index
            int j = std::get<1>(intersecting_primitive); 

            // Define the next vertex index
            int k;        
            
            // Define a variable to use a flag to jump to goal
            bool goToGoal = false;
            
            // Define the number of vertices in the obstacle
            int numVertices = all_primitives[i].size() + 1; //+1 to loop back to the hit point
            int visitedVertices = 0; // define a count to make sure we visit every vertex

            Eigen::Vector2d prev_vertex, next_vertex, edge_direction, current_point_prev_vertex_vector;
            double orig_distance_between_verticies, orig_distance_between_current_and_prev_vertex_along_the_edge, distance_traveled;
            

            // After a collision, follow the wall unless you can move straight to goal
            do {
                // Can we go to goal from current point without any intersections?
                // goToGoal = jump_to_goal(current_point, problem.q_goal, all_primitives, path);
                // if(goToGoal){
                //     goto Move_to_goal; // label to jump to top of while loop
                // }

                // Otherwise move towards the CCW vertex from hit point
                k = (j + 1) % all_primitives[i].size(); // get the next vertex index
                prev_vertex = std::get<0>(all_primitives[i][j]); // get the previous vertex location
                next_vertex = std::get<0>(all_primitives[i][k]); // get the next vertex location 
                edge_direction = std::get<1>(all_primitives[i][j]).normalized(); // direction of the edge from previous vertex to next vertex
                current_point_prev_vertex_vector = current_point - prev_vertex; // vector from prev vertex to current point
                
                // Get the original distance to the next vertex
                orig_distance_between_verticies = (next_vertex - prev_vertex).norm();
                
                // Get the original distance from the current point to the previous vertex along the edge we are going to follow
                orig_distance_between_current_and_prev_vertex_along_the_edge = (current_point_prev_vertex_vector.dot(edge_direction)/(edge_direction.dot(edge_direction))*edge_direction).dot(edge_direction);
                distance_traveled = 0; // we will follow until: distance_traveled + orig_distance_between_current_and_prev_vertex_along_the_edge < orig_distance_between_verticies

                // Follow the edge until the next vertex is reached
                do
                {
                    // Can we go to goal from current point without any intersections?
                    // goToGoal = jump_to_goal(current_point, problem.q_goal, all_primitives, path);
                    // if(goToGoal){
                    //     goto Move_to_goal;
                    // }

                    // Otherwise move along the edge direciton of the current edge
                    edge_direction = std::get<1>(all_primitives[i][j]).normalized(); // direction of the edge
                    
                    // Does movement along the edge intersect with any primitive?
                    safe_next_point = true;
                    rel_next_point = edge_direction * step_size;
                    intersecting_primitive_tuple_vector = next_step_collision(all_primitives, std::make_tuple(current_point, rel_next_point));
                    
                    intersecting_primitive_tuple = intersecting_primitive_tuple_vector.back();
                    safe_next_point = std::get<0>(intersecting_primitive_tuple);
                    
                    if (safe_next_point == false) { // if movement along the edge intersects with a primitive
       
                        // New hit point! Reset the hit point and distance to goal
                        //hit_point = current_point;
                        //distance_hit_point_to_goal = (problem.q_goal - hit_point).norm();
                        current_distance_goal = distance_hit_point_to_goal;
                        
                        // Get the intersecting primitive
                        intersecting_primitive = std::get<1>(intersecting_primitive_tuple);
                        
                        // Get the new intersecting obstacle index and edge index
                        i = std::get<0>(intersecting_primitive);
                        j = std::get<1>(intersecting_primitive);
                        k = (j + 1) % all_primitives[i].size(); // get the next vertex index to travel towards
                        
                        prev_vertex = std::get<0>(all_primitives[i][j]); // get the new previous vertex
                        next_vertex = std::get<0>(all_primitives[i][k]); // get the new next vertex

                        // Move along the new edge until the next vertex is reached
                        edge_direction = std::get<1>(all_primitives[i][j]).normalized(); // direction of the edge
                        current_point_prev_vertex_vector = current_point - prev_vertex; // vector from current point to prev vertex

                        // Get the original distance to the next vertex
                        orig_distance_between_verticies = (next_vertex - prev_vertex).norm();
                        orig_distance_between_current_and_prev_vertex_along_the_edge = (current_point_prev_vertex_vector.dot(edge_direction)/(edge_direction.dot(edge_direction))*edge_direction).dot(edge_direction);
                        distance_traveled = 0;

                         // Get the number of vertices in the new obstacle
                        numVertices = all_primitives[i].size() + 1; //to loop back to the hit point
                        visitedVertices = 0;
                        
                    }
                    
                    // Either the original step is safe or are moving along a different safe edge because edge_direction is safe 
                    current_point = path.waypoints.back() + edge_direction * step_size; // new point along edge
                    current_distance_goal = (problem.q_goal - current_point).norm();  
                    distance_traveled += step_size;
                    
                    // Add the new point to the path
                    path.waypoints.push_back(current_point);

                    // Add the new point as a potential new leave point 
                    leave_points.push_back(std::make_tuple(current_point, current_distance_goal));
                } 
                while (orig_distance_between_current_and_prev_vertex_along_the_edge + distance_traveled < orig_distance_between_verticies && max_itr3 < 10000);
                max_itr3 = 0;

                // Now I have traveled at least far enough to be outside the next vertex I was traveling towards
                j = k; // update the current edge index to the next vertex I just reached
                visitedVertices++; // I have visited a vertex
                max_itr2++; // increment to avoid inf loops
            } 
            while(visitedVertices < numVertices && max_itr2 < 50); // if I have visited every vertex, or if I have been in the loop for too long, break out of the loop
            max_itr2 = 0; // reset the max itr variable

            // Determine the closest leave point I found to resume towards the goal
            double min_distance = 1000;
            Eigen::Vector2d closest_point;

            for (const auto& leave_point : leave_points) { // for every leave point
                if (std::get<1>(leave_point) < min_distance) { // if the distance to the goal is less than the current min distance
                    min_distance = std::get<1>(leave_point);
                    closest_point = std::get<0>(leave_point);
                }
            }
        
            // Navigate back to the closest leave point
            int p = 0; 

            // Find the current point in the leave points
            p = find(leave_points.begin(), leave_points.end(), std::make_tuple(current_point, current_distance_goal)) - leave_points.begin();
            while ((current_point - closest_point).norm() > step_size && p < leave_points.size()) {
                current_point = std::get<0>(leave_points[p]); // go back through every leave point until I reach the closest one
                path.waypoints.push_back(current_point);
                p++;
            }
        
            leave_points.clear(); // clear the leave points for the next collision
        }
        
        // Now I am ready to move towards the goal again
        max_itr++;
    }

    // Add the goal to the final path
    path.waypoints.push_back(problem.q_goal);
    return path;
}

std::tuple<Eigen::Vector2d, Eigen::Vector2d> MyBugAlgorithm::getPrimitive(const Eigen::Vector2d vertex1, const Eigen::Vector2d vertex2) {
    // Get primitive from two vertices
    // Primitive is a vector from vertex1 to vertex2
    return std::make_tuple(vertex1, vertex2 - vertex1);
}

std::vector<std::tuple<bool, std::tuple<int, int>>> MyBugAlgorithm::next_step_collision(const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step){
     // Check for collisions with all obstacle primitives
    std::vector<std::tuple<bool, std::tuple<int, int>>> all_intersecting_primitives;
    all_intersecting_primitives.push_back(std::make_tuple(true, std::make_tuple(-1, -1)));

    std::tuple<int, int> intersecting_primitive;
    bool safe_next_point = true;

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
                intersecting_primitive = std::make_tuple(i, j);
                all_intersecting_primitives.push_back(std::make_tuple(false, intersecting_primitive));
            }
        }
    }
    return all_intersecting_primitives;
}


std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> MyBugAlgorithm::get_all_primitives(const amp::Problem2D& problem) {
    // Get all primitives for all obstacles
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives;
    
    // For each obstacle
    for (int i = 0; i < problem.obstacles.size(); i++) { 
        // Get the vertices of the obstacle
        std::vector<Eigen::Vector2d> verticies = problem.obstacles[i].verticesCW();
        std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>> primitives;

        // Compute the centroid of the obstacle to expand
        for (int j = 0; j < verticies.size(); j++) { 
            
            // If the last vertex is reached, create a primitive from the last vertex to the first vertex
            if (j == verticies.size() - 1) {
                primitives.push_back(getPrimitive(verticies[j], verticies[0]));
                continue;
            }
            // Create a primitive from the current vertex to the next vertex               
            primitives.push_back(getPrimitive(verticies[j], verticies[j+1]));
        }
        all_primitives.push_back(primitives);
    }
    return all_primitives;
}

 bool MyBugAlgorithm::jump_to_goal(const Eigen::Vector2d& current_point, const Eigen::Vector2d& goal, const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>>& all_primitives, amp::Path2D& path) {
    // Try to move towards goal without intersecting
    Eigen::Vector2d ideal_next_point = (goal - current_point);
    std::vector<std::tuple<bool, std::tuple<int, int>>> intersecting_primitive_tuple_vector = next_step_collision(all_primitives, std::make_tuple(current_point, ideal_next_point));
    std::tuple<bool, std::tuple<int, int>> intersecting_primitive_tuple = intersecting_primitive_tuple_vector.back();
    if (std::get<0>(intersecting_primitive_tuple) == true) {
        //std::cout << "I can move towards the goal without intersecting" << std::endl;
        path.waypoints.push_back(current_point + ideal_next_point);
        return true;
    }
    return false;
}