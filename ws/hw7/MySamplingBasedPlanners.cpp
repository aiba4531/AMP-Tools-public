# include "MySamplingBasedPlanners.h"

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    
    // Create a shared pointer to the graph
    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::Vector2d> nodes;

    // Add the start node to graph
    nodes[0] = {0, problem.q_init};

    // Get the bounds of the workspace
    double x_max = problem.x_max;
    double x_min = problem.x_min;
    double y_max = problem.y_max;
    double y_min = problem.y_min;

    // Get all linear primitives
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives = get_all_primitives(problem);
    
    // Get the number of samples
    int n = MyPRM::n;
    
    // Take n random samples of points in the c-space
    std::vector<Eigen::Vector2d> samples;

    // Generate n random samples in the c-space
    for (int i = 0; i < n; i++) {
        double x = x_min + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (x_max - x_min)));
        double y = y_min + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (y_max - y_min)));
        Eigen::Vector2d sample(x, y);

        // If this sample is not in collision, add it to the list of samples
        if (!point_in_polygons(sample, problem)) {
            samples.push_back(sample);
        }
    }

    // Check if the samples are collision free
    std::vector<Eigen::Vector2d> collision_free_samples;
    bool in_collision;
    for (int i = 0; i < n; i++) {
        in_collision = point_in_polygons(samples[i], problem);
        if (in_collision == false) {
            collision_free_samples.push_back(samples[i]);
        }
    }

    // Connect the nodes that are within a certain distance r of each other
    double r = MyPRM::r;
    std::vector<std::tuple<amp::Node, amp::Node, double>> edges;
    for (amp::Node i = 0; i < collision_free_samples.size(); i++) {
        for (amp::Node j = 0; j < collision_free_samples.size(); j++) {
            if (i != j) {
                double distance = (collision_free_samples[i] - collision_free_samples[j]).norm();
                if (distance < r) {
                    if (line_segment_in_polygon(all_primitives, std::make_tuple(collision_free_samples[i], collision_free_samples[j])) == false) {
                        edges.push_back(std::make_tuple(i,j, distance));
                    }
                }
            }
        }
    }

    // Run A* on the graph to find the shortest path
    std::map<amp::Node, Eigen::Vector2d> nodes;
    
    for (int i = 0; i < collision_free_samples.size(); i++) {
        nodes[i] = collision_free_samples[i];
    }

    for (const auto& [from, to, weight] : edges) {
        graphPtr->connect(from, to, weight);
    }

    graphPtr->print();

    MyAStarAlgo aStar;
    amp::ShortestPathProblem shortestPathProblem;
    shortestPathProblem.graph = graphPtr;
    shortestPathProblem.goal_node = nodes[nodes.size() - 2];
    shortestPathProblem.init_node = nodes[nodes.size() - 1];
    shortestPathProblem.heuristic = MyAStarAlgo::heuristic;
    amp::GraphSearchResult result = aStar.search(shortestPathProblem, MyAStarAlgo::heuristic);
    
    // std::vector<Eigen::Vector2d> points = {{3, 3}, {4, 5}, {5, 3}, {6, 5}, {5, 7}, {7, 3}}; // Points to add to the graph
    // for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map
    // std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {{0, 4, 1}, {0, 5, 1}, {4, 5, 1}, {1, 2, 1}, {1, 3, 1}, {2, 3, 1}}; // Edges to connect
    // for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight); // Connect the edges in the graph
    // //graphPtr->print();


    

    return path;


    // Psuedo-Code
    // 1. Sample n random points in the c-space
    // 2. For every sample, check if it is collision free
    // 3. If it is collision free, add it to the graph
    // 4. After adding all the samples to the graph, 
    //   connect the nodes that are within a certain distance r of each other
    // 5. Check if the connected nodes are collision free
    // 6. If they are collision free, add the edge to the graph
    // 7. Run A* on the graph to find the shortest path
}


// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);
    return path;
}

// Checking if a point is inside a polygon
bool point_in_polygons(Eigen::Vector2d point, const amp::Problem2D& problem) {   

    // For every obstacle in the problem
    for (int i = 0; i < problem.obstacles.size(); i++) {

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
                            return true;
                        }
                    }
                }
            }
    
            // Store the current point as the first point for the next iteration
            p1 = p2;
        }
    }
    // Return the value of the inside flag
    return false;
}

// Line segment polygon intersection algorithm
bool line_segment_in_polygon(const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step){
    
    // Check for collisions with all obstacle primitives
    std::tuple<int, int> intersecting_primitive;
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
std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> get_all_primitives(const amp::Environment2D& env) {
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives;
    for (int i = 0; i < env.obstacles.size(); i++) {
        std::vector<Eigen::Vector2d> verticies = env.obstacles[i].verticesCW();
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