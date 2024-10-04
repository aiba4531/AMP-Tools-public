#include "MyBugAlgorithm.h"
// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    double step_size = 0.1;
    
    // Get a vector that holds the linear primitives for every obsticle
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives = get_all_primitives(problem);
    std::vector<std::tuple<Eigen::Vector2d, double>> leave_points;
    std::vector<std::tuple<Eigen::Vector2d, double>> hit_points;

    Eigen::Vector2d goalVector = (problem.q_goal - path.waypoints.back()).normalized();
    double orig_distance = (problem.q_goal - path.waypoints.back()).norm();

    int idx = 0;
    while (idx < 100) {
    
        if (path.waypoints.back() == problem.q_goal) {
            break;
        }
    Move_to_goal:
        Eigen::Vector2d current_point = path.waypoints.back();
        goalVector = (problem.q_goal - current_point).normalized();
        Eigen::Vector2d rel_next_point = goalVector * step_size;
        Eigen::Vector2d next_point = current_point + rel_next_point;

        std::vector<std::tuple<bool, std::tuple<int, int>>> intersecting_primitive_tuple_vector;
        std::tuple<bool, std::tuple<int, int>> intersecting_primitive_tuple;
        std::tuple<int, int> intersecting_primitive;
        bool safe_next_point = true;

        intersecting_primitive_tuple_vector = next_step_collision(all_primitives, std::make_tuple(current_point, rel_next_point));
        intersecting_primitive_tuple = intersecting_primitive_tuple_vector.back();

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
            // Method: Wall-following
            // 1. Store the current point as a hit point
            // 2. Move along the edge until the end of the primitive
            // 3. Move along the next edge until end of primitive
            // 4. Store every point traveled as a leave point w distance to goal
            // 5. Once the original hit point is reached, navigate back to the closest leave point


            Eigen::Vector2d hit_point = current_point;
            //std::cout << "I found hit point: " << hit_point[0] << " " << hit_point[1] << std::endl;

            double distance_hit_point_to_goal = (problem.q_goal - hit_point).norm();
            double current_distance_goal = distance_hit_point_to_goal;
            int numVertices = all_primitives[std::get<0>(intersecting_primitive)].size() + 1; //to loop back to the hit point
            int visitedVertices = 0;

            //std::cout << "I hit a wall at: " << hit_point[0] << " " << hit_point[1] << std::endl;
            int i = std::get<0>(intersecting_primitive); // obstacle index
            int j = std::get<1>(intersecting_primitive); // edge index
            int k;
            int m = 0;

            do {
                // The next vertex must be counter clockwise from hit point
                k = (j + 1) % all_primitives[i].size(); // get the next vertex index
                Eigen::Vector2d prev_vertex = std::get<0>(all_primitives[i][j]); // get the previous vertex
                Eigen::Vector2d next_vertex = std::get<0>(all_primitives[i][k]); // get the next vertex

                //# Move along the edge until the next vertex is reached #
                Eigen::Vector2d edge_direction = std::get<1>(all_primitives[i][j]).normalized(); // direction of the edge
                Eigen::Vector2d current_point_prev_vertex_vector = current_point - prev_vertex; // vector from current point to prev vertex

                // Get the original distance to the next vertex
                double orig_distance_between_verticies = (next_vertex - prev_vertex).norm();
                double orig_distance_between_current_and_prev_vertex_along_the_edge = (current_point_prev_vertex_vector.dot(edge_direction)/(edge_direction.dot(edge_direction))*edge_direction).dot(edge_direction);
                double distance_traveled = 0;

            
                do
                {
                    edge_direction = std::get<1>(all_primitives[i][j]).normalized(); // direction of the edge
                    
                    // Check for collisions with all obstacle primitives
                    intersecting_primitive_tuple_vector = next_step_collision(all_primitives, std::make_tuple(current_point, edge_direction * step_size));
                    intersecting_primitive_tuple = intersecting_primitive_tuple_vector.back();
                    if (std::get<0>(intersecting_primitive_tuple) == false) {
                        // Try to move towards goal without intersecting
                        Eigen::Vector2d ideal_direction = (problem.q_goal - current_point).normalized();
                        //std::cout << "Ideal direction: " << ideal_direction[0] << " " << ideal_direction[1] << std::endl;
                        Eigen::Vector2d ideal_next_point = ideal_direction * step_size;
                        intersecting_primitive_tuple_vector = next_step_collision(all_primitives, std::make_tuple(current_point, ideal_next_point));
                        intersecting_primitive_tuple = intersecting_primitive_tuple_vector.back();
                        if (std::get<0>(intersecting_primitive_tuple) == true) {
                            //std::cout << "I can move towards the goal without intersecting" << std::endl;
                            path.waypoints.push_back(current_point + ideal_next_point);
                            goto Move_to_goal;
                        }

                        // Reset Everything before we continue
                        hit_point = current_point;
                        distance_hit_point_to_goal = (problem.q_goal - hit_point).norm();
                        current_distance_goal = distance_hit_point_to_goal;
                        intersecting_primitive = std::get<1>(intersecting_primitive_tuple);
                        i = std::get<0>(intersecting_primitive);
                        j = std::get<1>(intersecting_primitive);
                        k = (j + 1) % all_primitives[i].size(); // get the next vertex index
                        numVertices = all_primitives[i].size() + 1; //to loop back to the hit point
                        visitedVertices = 0;
                        prev_vertex = std::get<0>(all_primitives[i][j]); // get the previous vertex
                        next_vertex = std::get<0>(all_primitives[i][k]); // get the next vertex

                        //# Move along the edge until the next vertex is reached #
                        edge_direction = std::get<1>(all_primitives[i][j]).normalized(); // direction of the edge
                        current_point_prev_vertex_vector = current_point - prev_vertex; // vector from current point to prev vertex

                        // Get the original distance to the next vertex
                        orig_distance_between_verticies = (next_vertex - prev_vertex).norm();
                        orig_distance_between_current_and_prev_vertex_along_the_edge = (current_point_prev_vertex_vector.dot(edge_direction)/(edge_direction.dot(edge_direction))*edge_direction).dot(edge_direction);
                        distance_traveled = 0;
                    }
                    
                    current_point = path.waypoints.back() + edge_direction * step_size; // new point along edge
                    current_distance_goal = (problem.q_goal - current_point).norm();  
                    distance_traveled += step_size;
                    
                    path.waypoints.push_back(current_point);  //// TODO: Check if add hit point to struct
                    leave_points.push_back(std::make_tuple(current_point, current_distance_goal));
                    
                } 
                while (orig_distance_between_current_and_prev_vertex_along_the_edge + distance_traveled < orig_distance_between_verticies);
                
            // Now I have reached the next vertex so I want to update next vertex
                j = k;
                visitedVertices++;
                //std::cout << "I made it to the next vertex: " << next_vertex[0] << " " << next_vertex[1] << std::endl;
                m++;
                //std::cout << "On obstacle: " << i << " I have visited " << visitedVertices << " vertices" << std::endl;
            } 
            while(visitedVertices < numVertices && m < 200);

            // Determine the closest leave point to resume towards the goal
            double min_distance = 1000;
            Eigen::Vector2d closest_point;

            for (const auto& leave_point : leave_points) { // for every leave point
                if (std::get<1>(leave_point) < min_distance) { // if the distance to the goal is less than the current min distance
                    min_distance = std::get<1>(leave_point);
                    closest_point = std::get<0>(leave_point);
                }
            }
            //std::cout << "I found the closest point is: " << closest_point[0] << " " << closest_point[1] << std::endl;

            // Navigate back to the closest leave point
            int p = 0;
            while ((current_point - closest_point).norm() > step_size && p < leave_points.size()) {
                current_point = std::get<0>(leave_points[p]);
                path.waypoints.push_back(current_point);
                p++;
            }
            //std::cout << "I made it back to the closest point: " << closest_point[0] << " " << closest_point[1] << std::endl;
        }
        idx++;
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

Eigen::Vector2d MyBugAlgorithm::calculateCentroid(const std::vector<Eigen::Vector2d>& vertices) {
    Eigen::Vector2d centroid(0.0, 0.0);
    for (const auto& vertex : vertices) {
        centroid += vertex;
    }
    centroid /= vertices.size();
    return centroid;
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
        Eigen::Vector2d centroid = calculateCentroid(verticies);

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


