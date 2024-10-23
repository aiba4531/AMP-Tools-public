# include "MySamplingBasedPlanners.h"

// // Implement your PRM algorithm here
// amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
amp::Path2D MyPRM::plan(const amp::Problem2D& problem){
    // Create a path
    amp::Path2D path;

    // Get the bounds of the workspace
    // double x_max = problem.x_max;
    // double x_min = problem.x_min;
    // double y_max = problem.y_max;
    // double y_min = problem.y_min;

    // Problem 1 bounds
    double x_max = 11;
    double x_min = -1;
    double y_max = 3;
    double y_min = -3;

    // Get all linear primitives
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives = get_all_primitives(problem);
    
    // Get the number of samples
    std::size_t idx = 0;

    // Add the start point as a node
    nodes[idx] = problem.q_init;
    idx++;

    // Add the goal point as a node
    nodes[idx] = problem.q_goal;
    idx++;
    
    // Generate n random samples in the c-space
    for (int i = 0; i < n; i++) {
        double x = x_min + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (x_max - x_min)));
        double y = y_min + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (y_max - y_min)));
        Eigen::Vector2d sample(x, y);

        // If this sample is not in collision, add it to the list of samples
        if (!point_in_polygons(sample, problem)) {
            nodes[idx] = sample;
            idx++;
        }
    }
    
    // Connect the nodes that are within a certain distance r of each other
    amp::LookupSearchHeuristic heuristic;
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges;

   // For every node in the graph 
    for (amp::Node i = 0; i < nodes.size(); i++) {
        for (amp::Node j = 0; j < nodes.size(); j++) {
            if (i != j) {

                // Get the distance between the two nodes
                double distance = (nodes[i] - nodes[j]).norm();

                // If the distance is less than r, check if the edge is collision free
                if (distance < r) {
                    // Line segment between the two nodes [point, relative vector]
                    std::tuple<Eigen::Vector2d, Eigen::Vector2d> line_segment = std::make_tuple(nodes[i], nodes[j]-nodes[i]);
                    if (!line_segment_in_polygon(all_primitives, line_segment)) {
                        edges.push_back(std::make_tuple(i, j, distance));
                    }
                }
            }
        }
    }

    graphPtr = std::make_shared<amp::Graph<double>>();
    for (const auto& [from, to, weight] : edges) {
        graphPtr->connect(from, to, weight);
    }

    // Set the heuristic values for A*
    for (amp::Node i = 0; i < nodes.size(); i++) {
        heuristic.heuristic_values[i] = (nodes[i] - problem.q_goal).norm();
    }


    // amp::Visualizer::makeFigure(problem, path, *graphPtr, nodes);
    // amp::Visualizer::showFigures();

    // Run A* to find the shortest path
    MyAStarAlgo aStar;
    amp::ShortestPathProblem shortestPathProblem;
    shortestPathProblem.graph = graphPtr;
    shortestPathProblem.init_node = 0;
    shortestPathProblem.goal_node = 1;
    
    MyAStarAlgo::GraphSearchResult result = aStar.search(shortestPathProblem, heuristic);

    if (!result.success) {
        return path;
    }
    
    // Reconstruct the path from goal to start using the parent map
    while (!result.node_path.empty()) {
        path.waypoints.push_back(nodes[result.node_path.front()]);
        result.node_path.pop_front();
    }

    // Run Path smoothing
    path = path_smoothing(path, problem, all_primitives);

    return path;
}


// // Implement your RRT algorithm here
// amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {

//     // Create a path
//     amp::Path2D path;

//      // Get the bounds of the workspace
//     double x_max = problem.x_max;
//     double x_min = problem.x_min;
//     double y_max = problem.y_max;
//     double y_min = problem.y_min;

//     double x_max = 11;
//     double x_min = -1;
//     double y_max = 3;
//     double y_min = -3;

//     // Create a tree with root at the start node
//     graphPtr = std::make_shared<amp::Graph<double>>();
//     nodes = std::map<amp::Node, Eigen::Vector2d>();

//     // Get all linear primitives
//     std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives = get_all_primitives(problem);

//     // Get the distance for convergence
//     amp::LookupSearchHeuristic heuristic;

//     // Define an index to update nodes 
//     std::size_t idx = 0;
//     int num_itr = 0;

//     // Add the start point as a node
//     nodes[idx] = problem.q_init;
//     heuristic.heuristic_values[idx] =  0;
//     idx++;

//     // Distant to goal
//     double distance;
//     Eigen::Vector2d sample;

//     // While the goal is not reached or a mumber of iterations is not reached
//     while (num_itr < n) {

//         // Sample the goal with probability goal_bias
//         double prob = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        
//         if (prob < MyRRT::goal_bias) {
//             sample = problem.q_goal;


//             amp::Node nearest_node = 0;
//             double min_distance = std::numeric_limits<double>::max();
//             double curr_distance = 0;
//             for (const auto& [node, point] : nodes) {
//                 curr_distance = (point - sample).norm();
//                 if (curr_distance < min_distance) {
//                     min_distance = curr_distance;
//                     nearest_node = node;
//                 }
//             }

//             // Get the direction vector from the nearest node to the sample
//             Eigen::Vector2d direction = (sample - nodes[nearest_node]).normalized();

//             // Step size
//             double r = MyRRT::r;

//             // Get the next step in the direction of the sample
//             Eigen::Vector2d next_step = nodes[nearest_node] + direction*r;

//             // Get the relative vector between nearest node and next step
//             std::tuple<Eigen::Vector2d, Eigen::Vector2d> line_segment = std::make_tuple(nodes[nearest_node], next_step-nodes[nearest_node]);

//             if (!line_segment_in_polygon(all_primitives, line_segment)) {
                
//                 // Check if the goal is reached
//                 double dist_to_goal = (next_step - problem.q_goal).norm();
//                 if (dist_to_goal < epsilon) {
//                     nodes[idx] = next_step;
//                     graphPtr->connect(nearest_node, idx, r);
//                     heuristic.heuristic_values[idx] =  0;
//                     idx++;

//                     nodes[idx] = problem.q_goal;
//                     graphPtr->connect(idx-1, idx, epsilon);
//                     heuristic.heuristic_values[idx] =  0;
//                     break;
//                 }
                
//                 nodes[idx] = next_step;
//                 graphPtr->connect(nearest_node, idx, (nodes[nearest_node] - next_step).norm());
//                 heuristic.heuristic_values[idx] =  0;
//                 idx++;

//                 continue;
//             }
//             continue;


//         }
//         else {
//             // Generate a random sample in the c-space
//             double x = x_min + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (x_max - x_min)));
//             double y = y_min + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (y_max - y_min)));
//             sample = Eigen::Vector2d(x, y);

//             if (point_in_polygons(sample, problem)) {
//                 continue;
//             }
//         }

//         //std::cout << "Itr: " << num_itr << " sampling: " << sample.transpose();


//         // Find the nearest node in the tree to the sample
//         amp::Node nearest_node = 0;
//         double min_distance = std::numeric_limits<double>::max();
//         double curr_distance = 0;
        
//         for (const auto& [node, point] : nodes) {
//             curr_distance = (point - sample).norm();
//             if (curr_distance < min_distance) {
//                 min_distance = curr_distance;
//                 nearest_node = node;
//             }
//         }
//         //std::cout << " where the nearest node is " << nearest_node << " located at " << nodes[nearest_node].transpose() << std::endl;

//         // Get the direction vector from the nearest node to the sample
//         Eigen::Vector2d direction = (sample - nodes[nearest_node]).normalized();

//         // Step size
//         r = MyRRT::r;

//         // Get the next step in the direction of the sample
//         Eigen::Vector2d next_step = nodes[nearest_node] + direction*r;

//         // Get the relative vector between nearest node and next step
//         std::tuple<Eigen::Vector2d, Eigen::Vector2d> line_segment = std::make_tuple(nodes[nearest_node], next_step-nodes[nearest_node]);

//         if (line_segment_in_polygon(all_primitives, line_segment)) {
//             continue;
//         }


//         // Check if the goal is reached
//         double dist_to_goal = (next_step - problem.q_goal).norm();
//         if (dist_to_goal < epsilon) {
//             nodes[idx] = next_step;
//             graphPtr->connect(nearest_node, idx, r);
//             heuristic.heuristic_values[idx] =  0;
//             idx++;

//             nodes[idx] = problem.q_goal;
//             graphPtr->connect(idx-1, idx, epsilon);
//             heuristic.heuristic_values[idx] =  0;
//             break;
//         }
        
//         nodes[idx] = next_step;
//         graphPtr->connect(nearest_node, idx, r);
//         heuristic.heuristic_values[idx] =  0;//(nodes[idx] - problem.q_goal).norm();
//         idx++;
    
//         num_itr++;
//     }

//     if (graphPtr->nodes().size() == 0) {
//         return path;
//     }
    
//     // graphPtr->print();

//     // amp::Visualizer::makeFigure(problem, path, *graphPtr, nodes);
//     // amp::Visualizer::showFigures();

    
//     MyAStarAlgo aStar;
//     amp::ShortestPathProblem shortestPathProblem;
//     shortestPathProblem.graph = graphPtr;
//     shortestPathProblem.init_node = 0;
//     shortestPathProblem.goal_node = idx;
    
//     MyAStarAlgo::GraphSearchResult result = aStar.search(shortestPathProblem, heuristic);

//     if (!result.success) {
//         return path;
//     }
    
//     // Reconstruct the path from goal to start using the parent map
//     while (!result.node_path.empty()) {
//         path.waypoints.push_back(nodes[result.node_path.front()]);
//         result.node_path.pop_front();
//     }


//     return path;
// }

// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {

    // Create a path
    amp::Path2D path;

    // Get the bounds of the workspace
    double x_max = problem.x_max;
    double x_min = problem.x_min;
    double y_max = problem.y_max;
    double y_min = problem.y_min;

    // double x_max = 11;
    // double x_min = -1;
    // double y_max = 3;
    // double y_min = -3;

    // Create a tree with root at the start node
    graphPtr = std::make_shared<amp::Graph<double>>();
    nodes = std::map<amp::Node, Eigen::Vector2d>();

    // Get all linear primitives
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives = get_all_primitives(problem);

    // Define an index to update nodes 
    std::size_t idx = 0;
    int num_itr = 0;

    // Add the start point as a node
    nodes[idx] = problem.q_init;
    idx++;

    // Distant to goal
    double distance;
    Eigen::Vector2d sample;

    // While the goal is not reached or a mumber of iterations is not reached
    while (num_itr < n) {

        // Sample the goal with probability goal_bias
        double prob = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
        
        if (prob < MyRRT::goal_bias) {
            sample = problem.q_goal;

            amp::Node nearest_node = closest_node(nodes, sample);

            // Get the direction vector from the nearest node to the sample
            Eigen::Vector2d direction = (sample - nodes[nearest_node]).normalized();

            // Get the next step in the direction of the sample
            Eigen::Vector2d next_step = nodes[nearest_node] + direction*r;

            // Get the relative vector between nearest node and next step
            std::tuple<Eigen::Vector2d, Eigen::Vector2d> line_segment = std::make_tuple(nodes[nearest_node], next_step-nodes[nearest_node]);

            if (line_segment_in_polygon(all_primitives, line_segment)) {
                continue;
            }

             // Check if the goal can be reached
             double dist_to_goal = (next_step - problem.q_goal).norm();
             if (dist_to_goal < epsilon) {
                 nodes[idx] = problem.q_goal;
                 graphPtr->connect(nearest_node, idx, r + epsilon);
                 break;
             }
             
             // Otherwise step in the direction of the goal
             nodes[idx] = next_step;
             graphPtr->connect(nearest_node, idx, r);
             idx++;
             continue;
        }

         // Generate a random sample in the c-space
         double x = x_min + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (x_max - x_min)));
         double y = y_min + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (y_max - y_min)));
         sample = Eigen::Vector2d(x, y);

         if (point_in_polygons(sample, problem)) {
             continue;
         }

        // Find the nearest node in the tree to the sample
        amp::Node nearest_node = closest_node(nodes, sample);

        // Get the direction vector from the nearest node to the sample
        Eigen::Vector2d direction = (sample - nodes[nearest_node]).normalized();

        // Get the next step in the direction of the sample
        Eigen::Vector2d next_step = nodes[nearest_node] + direction*r;

        // Get the relative vector between nearest node and next step
        std::tuple<Eigen::Vector2d, Eigen::Vector2d> line_segment = std::make_tuple(nodes[nearest_node], next_step-nodes[nearest_node]);

        if (line_segment_in_polygon(all_primitives, line_segment)) {
            continue;
        }

        // Check if the goal is reached
        double dist_to_goal = (next_step - problem.q_goal).norm();
        if (dist_to_goal < epsilon) {
            nodes[idx] = problem.q_goal;
            graphPtr->connect(nearest_node, idx, r + epsilon);
            break;
        }
        
        nodes[idx] = next_step;
        graphPtr->connect(nearest_node, idx, r);
        idx++;
    
        num_itr++;
    }

    // Assign heursitics
    amp::LookupSearchHeuristic heuristic;
    for (amp::Node i = 0; i < nodes.size(); i++) {
        heuristic.heuristic_values[i] = (nodes[i] - problem.q_goal).norm();
    }
    
    // Run A* to find the shortest path
    MyAStarAlgo aStar;
    amp::ShortestPathProblem shortestPathProblem;
    shortestPathProblem.graph = graphPtr;
    shortestPathProblem.init_node = 0;
    shortestPathProblem.goal_node = idx;
    
    MyAStarAlgo::GraphSearchResult result = aStar.search(shortestPathProblem, heuristic);

    if (!result.success) {
        return path;
    }
    
    // Reconstruct the path from goal to start using the parent map
    while (!result.node_path.empty()) {
        path.waypoints.push_back(nodes[result.node_path.front()]);
        result.node_path.pop_front();
    }

    return path;
}


// Closest node to 
amp::Node closest_node(const std::map<amp::Node, Eigen::Vector2d>& nodes, const Eigen::Vector2d& sample) {
    amp::Node nearest_node = 0;
    double min_distance = std::numeric_limits<double>::max();
    double curr_distance = 0;

    for (const auto& [node, point] : nodes) {
        curr_distance = (point - sample).norm();
        if (curr_distance < min_distance) {
            min_distance = curr_distance;
            nearest_node = node;
        }
    }
    return nearest_node;
}

// Implement your path smoothing algorithm here
amp::Path2D path_smoothing(amp::Path2D path, const amp::Problem2D& problem, std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives){
    // Using the path, if any two configurations can be conncted, remove the middle point

    int max_iterations = 1000;
    int iterations = 0;

    while(iterations < max_iterations){
        // Randomly selecct two points in the path
        
        int idx1 = rand() % path.waypoints.size();
        int idx2 = rand() % path.waypoints.size();

        // Ensure idx1 is less than idx2
        if (idx1 > idx2) {
            std::swap(idx1, idx2);
        }

        if (idx2 - idx1 < 2) {
            iterations++;
            continue; // Skip if there are not enough points between idx1 and idx2
        }

        if (idx1 == 0 || idx2 == path.waypoints.size() - 1) {
            iterations++;
            continue; // Skip if the points are at the start or end of the path
        }

        // Check if the two points can be connected
        Eigen::Vector2d point1 = path.waypoints[idx1];
        Eigen::Vector2d point2 = path.waypoints[idx2];
        std::tuple<Eigen::Vector2d, Eigen::Vector2d> line_segment = std::make_tuple(point1, point2 - point1);

        if (!line_segment_in_polygon(all_primitives, line_segment)) {
            // If they can be connected, remove the middle points
            path.waypoints.erase(path.waypoints.begin() + idx1 + 1, path.waypoints.begin() + idx2);
        }
        iterations++;
    }

    return path;

}

// Checking if a point is inside a polygon
bool point_in_polygons(Eigen::Vector2d point, const amp::Problem2D& problem) {   

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
std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> get_all_primitives(const amp::Environment2D& problem) {
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