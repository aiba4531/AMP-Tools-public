#include "MyBugAlgorithm.h"
// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    double step_size = 0.1;
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives = get_all_primitives(problem);
    std::vector<std::tuple<Eigen::Vector2d, double>> leave_points;
    std::vector<std::tuple<Eigen::Vector2d, double>> hit_points;

    int idx = 0;
    Eigen::Vector2d orig_goalVector = (problem.q_goal - path.waypoints.back()).normalized();
    double orig_distance = (problem.q_goal - path.waypoints.back()).norm();

    while (idx < 100) {
        if (path.waypoints.back() == problem.q_goal) {
            break;
        }

        orig_goalVector = (problem.q_goal - path.waypoints.back()).normalized();

        Eigen::Vector2d current_point = path.waypoints.back();
        Eigen::Vector2d rel_next_point = orig_goalVector * step_size;
        Eigen::Vector2d next_point = current_point + rel_next_point;

        std::tuple<int, int> intersecting_primitive;
        bool safe_next_point = true;
        bool exit_outer_loop = false;

        // Check for collisions with all obstacle primitives
        for (int i = 0; i < all_primitives.size(); i++) {
            for (int j = 0; j < all_primitives[i].size(); j++) {
                safe_next_point = next_step_collision(all_primitives[i][j], std::make_tuple(current_point, rel_next_point));
                
                if (!safe_next_point) {
                    intersecting_primitive = std::make_tuple(i, j);
                    exit_outer_loop = true;
                    break;
                }
            }
            if (exit_outer_loop) {
                break;
            }
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

            //std::cout << "I hit a wall at: " << hit_point[0] << " " << hit_point[1] << std::endl;
            int i = std::get<0>(intersecting_primitive); // obstacle index
            int j = std::get<1>(intersecting_primitive); // edge index


            Eigen::Vector2d edge_direction = std::get<1>(all_primitives[i][j]).normalized(); // direction of the edge
            Eigen::Vector2d new_point = path.waypoints.back() + edge_direction * step_size; // new point along edge
            double new_distance = (problem.q_goal - new_point).norm();  
            
            
            path.waypoints.push_back(new_point);  //// TODO: Check if add hit point to struct
            leave_points.push_back(std::make_tuple(new_point, new_distance));

            
            new_point = path.waypoints.back() + edge_direction * step_size;
            path.waypoints.push_back(new_point);  //// TODO: Check if add hit point to struct
            
            new_distance = (problem.q_goal - new_point).norm();
            leave_points.push_back(std::make_tuple(new_point, new_distance));

            int m = 0, n = 0; // counters to avoid infinite loops

            // Move along the obsticle until the original hit point is reached
            while ((path.waypoints.back() - hit_point).norm() > step_size) {

                int k = (j + 1) % all_primitives[i].size(); // get the next vertex index
                Eigen::Vector2d next_vertex = std::get<0>(all_primitives[i][k]); // get the next vertex
                Eigen::Vector2d direction = (next_vertex - path.waypoints.back()).normalized();
                //std::cout << "Looking for the next vertex: " << next_vertex[0] << " " << next_vertex[1] << std::endl;

                ///// Check if the next point is a shared edge

                
                // Move along the edge until the next vertex is reached
                while ((path.waypoints.back() - next_vertex).norm() >  step_size ) {
                //while ((direction.dot(next_vertex - path.waypoints.back()) > 0.0) && m < 150) {

                    edge_direction = std::get<1>(all_primitives[i][j]).normalized(); // current edge direction
                    new_point = path.waypoints.back() + edge_direction * step_size; 
                    new_distance = (problem.q_goal - new_point).norm();
                    path.waypoints.push_back(new_point);
                    leave_points.push_back(std::make_tuple(new_point, new_distance));


                    new_point = path.waypoints.back() + edge_direction * step_size; 
                    new_distance = (problem.q_goal - new_point).norm();
                    path.waypoints.push_back(new_point);
                    leave_points.push_back(std::make_tuple(new_point, new_distance));


                    m++;
                }
                //std::cout << "I made it to the next vertex: " << next_vertex[0] << " " << next_vertex[1] << std::endl;
                m = 0;
                j = k;  // update the edge index to the current vertex I just reached
                n++;  
            }
            std::cout << "I made it back to the hit point: " << hit_point[0] << " " << hit_point[1] << std::endl;
            vertices_passed++;


            // Determine the closest leave point to resume towards the goal
            double min_distance = 1000;
            Eigen::Vector2d closest_point;

            for (const auto& leave_point : leave_points) { // for every leave point
                if (std::get<1>(leave_point) < min_distance) { // if the distance to the goal is less than the current min distance
                    min_distance = std::get<1>(leave_point);
                    closest_point = std::get<0>(leave_point);
                }
            }
            std::cout << "I found the closest point is: " << closest_point[0] << " " << closest_point[1] << std::endl;

            // Navigate back to the closest leave point
            int p = 0;
            while ((path.waypoints.back() - closest_point).norm() > 0.5 * step_size && p < leave_points.size()) {
                new_point = std::get<0>(leave_points[p]);
                path.waypoints.push_back(new_point);
                p++;
            }
            std::cout << "I made it back to the Closest point: " << closest_point[0] << " " << closest_point[1] << std::endl;
        }
        idx++;
    }

    // Add the goal to the final path
    path.waypoints.push_back(problem.q_goal);
    return path;
}

std::tuple<Eigen::Vector2d, Eigen::Vector2d> MyBugAlgorithm::getPrimitive(Eigen::Vector2d vertex1, Eigen::Vector2d vertex2, const Eigen::Vector2d& centroid) {
    
    double expansion_distance = 0.0000001;
    // Calculate direction vectors from the centroid to each vertex
    Eigen::Vector2d direction1 = (vertex1 - centroid).normalized();
    Eigen::Vector2d direction2 = (vertex2 - centroid).normalized();

    // Expand the vertices outward by the expansion distance
    Eigen::Vector2d expanded_vertex1 = vertex1 + direction1 * expansion_distance;
    Eigen::Vector2d expanded_vertex2 = vertex2 + direction2 * expansion_distance;

    //std::cout << "Expanded vertex 1: " << expanded_vertex1[0] << " " << expanded_vertex1[1] << std::endl;
    // Return the expanded primitive
    return std::make_tuple(expanded_vertex1, expanded_vertex2 - expanded_vertex1);
}

Eigen::Vector2d MyBugAlgorithm::calculateCentroid(const std::vector<Eigen::Vector2d>& vertices) {
    Eigen::Vector2d centroid(0.0, 0.0);
    for (const auto& vertex : vertices) {
        centroid += vertex;
    }
    centroid /= vertices.size();
    return centroid;
}

bool MyBugAlgorithm::next_step_collision(const std::tuple<Eigen::Vector2d, Eigen::Vector2d> primitive, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step){
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
        return false;
    }

    return true;
}


std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> MyBugAlgorithm::get_all_primitives(const amp::Problem2D& problem) {
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives;
    for (int i = 0; i < problem.obstacles.size(); i++) { 

        // Get the vertices of the obstacle
        std::vector<Eigen::Vector2d> verticies = problem.obstacles[i].verticesCCW();
        
        // Define a vector to hold a tuple of (vertex,  linear primitive) for each edge of the obstacle
        std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>> primitives;

        // Compute the centroid of the obstacle
        Eigen::Vector2d centroid = calculateCentroid(verticies);

        for (int j = 0; j < verticies.size(); j++) { // for each pair of vertices create a linear primitive
            
            // If the last vertex is reached, create a primitive from the last vertex to the first vertex
            if (j == verticies.size() - 1) {
                primitives.push_back(getPrimitive(verticies[j], verticies[0], centroid));
                continue;
            }
            // Create a primitive from the current vertex to the next vertex               
            primitives.push_back(getPrimitive(verticies[j], verticies[j+1], centroid));
        }
        all_primitives.push_back(primitives);
    }
    return all_primitives;
}


