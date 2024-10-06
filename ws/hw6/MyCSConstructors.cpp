#include "MyCSConstructors.h"
#include <queue>

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    double x0_min, x0_max, x1_min, x1_max, x0_step, x1_step, x0_diff, x1_diff;
    std::size_t x0_cell, x1_cell;


    // Get the bounds of the configuration space
    std::tie(x0_min, x0_max) = x0Bounds();
    std::tie(x1_min, x1_max) = x1Bounds();
    std::tie(x0_cell, x1_cell) = size();

    // Calculate the step size for each cell
    x0_step = (x0_max - x0_min) / x0_cell;
    x1_step = (x1_max - x1_min) / x1_cell;
    
    // Compute the difference between the point and the minimum bounds
    x0_diff = x0 - x0_min;
    x1_diff = x1 - x1_min;

    // Compute the cell index for each dimension using floor
    std::size_t cell_x = x0_diff / x0_step; // x index of cell
    std::size_t cell_y = x1_diff / x1_step; // x index of cell    
    
    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0, 2*M_PI, 0, 2*M_PI);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for manipulator" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class

     // Define all linear primitives for each obstacle
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

    // A link maniupaltor has two links length 1.0 each. The bounds of the cspace are the angles between links 0 and 1, and links 1 and the end effector.
    // Therefore we will loop through the joint angles use inverse kinematics to get the end effector position
    // Then check if that is in collision with an obstacle in the environment. If it is, set the cell to true, otherwise false.
    
    // Define 2D Manipulator State
    amp::ManipulatorState state = Eigen::VectorXd::Zero(manipulator.nLinks());

    // Define the bounds of the cspace
    double x0_min = 0;
    double x0_max = 2 * M_PI;
    double x1_min = 0;
    double x1_max = 2 * M_PI;
    double step = 2 * M_PI / m_cells_per_dim;

    // Iterate over each cell in the cspace
    for (int i = 0; i < m_cells_per_dim; i++) {
        for (int j = 0; j < m_cells_per_dim; j++) {
            // Get the bounds of the cell
            double x0 = x0_min + i * step;
            double x1 = x1_min + j * step;

            state << x0, x1; // this is the cell of space i a, checking if it is in collision with any obstacle
            bool in_collision = false;

            // Check if any link intersects with any obstacle in the environment
            for (int m = 0; m < manipulator.nLinks(); m++) {

                in_collision = false;
                
                Eigen::Vector2d joint = manipulator.getJointLocation(state, m);
                Eigen::Vector2d nextJoint = manipulator.getJointLocation(state, m+1);
                Eigen::Vector2d rel_nextJoint = nextJoint - joint;
                
                // Make a link tuple
                std::tuple<Eigen::Vector2d, Eigen::Vector2d> link = std::make_tuple(joint, rel_nextJoint);

                // Check if the link is in collision with the environment
                std::vector<std::tuple<bool, std::tuple<int, int>>> intersecting_primitive_tuple_vector;
                intersecting_primitive_tuple_vector = cspace.next_step_collision(all_primitives, link);


                std::tuple<bool, std::tuple<int, int>> intersecting_primitive_tuple = intersecting_primitive_tuple_vector.back();
                in_collision = std::get<0>(intersecting_primitive_tuple);

                //std::cout << "Link: " << m << " is in collision: " << in_collision << std::endl;

                if (in_collision == true) {
                    cspace(i,j) = true;
                    break;
                }
            }
            if (in_collision == false) {
                // std::cout << "Cell: " << i << ", " << j << " is not in collision: " << std::endl;
                cspace(i,j) = false;
            }

        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}


std::vector<std::tuple<bool, std::tuple<int, int>>> MyGridCSpace2D::next_step_collision(const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step){
     // Check for collisions with all obstacle primitives

    std::vector<std::tuple<bool, std::tuple<int, int>>> all_intersecting_primitives;

    // Assume there is no collision
    all_intersecting_primitives.push_back(std::make_tuple(false, std::make_tuple(-1, -1)));

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
                intersecting_primitive = std::make_tuple(i, j);

                // There is at least one collision with an obstacle
                all_intersecting_primitives.push_back(std::make_tuple(true, intersecting_primitive));

            }
        }
    }
    return all_intersecting_primitives;
}


// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {

    // Psuedo-Code:
    // 1. For every obstacle
    // 2. For every vertex in the obstacle
    // 3. Get the linear primitive from next vertex to current vertex
    // 4. Get every cell that the linear primitive intersects and set to true

    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class

    // Get the step size for each cell
    double step = (env.x_max - env.x_min) / m_cells_per_dim;

    // For a point agent, the C-space is going to be the cells that contain an obstacle
    for (auto& obstacle : env.obstacles) {
        for (int v = 0; v < obstacle.verticesCW().size(); v++) {
            Eigen::Vector2d vertex1, vertex2, current_point_along_the_primitive;
            
            if (v == obstacle.verticesCW().size() - 1) {
                vertex1 = obstacle.verticesCW()[v];
                vertex2 = obstacle.verticesCW()[0];
            } else {
                vertex1 = obstacle.verticesCW()[v];
                vertex2 = obstacle.verticesCW()[v+1];
            }

            // Get the linear primitive from the two vertices
            Eigen::Vector2d primitive = vertex2 - vertex1;
            Eigen::Vector2d primitive_unit_vector = primitive.normalized();

            // Find the cells that bound this current linear primitive
            std::pair<std::size_t, std::size_t> vertex1_cell = cspace.getCellFromPoint(vertex1[0], vertex1[1]);
            std::pair<std::size_t, std::size_t> vertex2_cell = cspace.getCellFromPoint(vertex2[0], vertex2[1]);

            // Set the current point along the primitive to the first vertex
            current_point_along_the_primitive = vertex1;
            double original_distance = (vertex2 - vertex1).norm();
            double distance_traveled = 0;

            while (distance_traveled < original_distance) {
                // Get the cell that the current point along the primitive is in
                std::pair<std::size_t, std::size_t> current_point_cell = cspace.getCellFromPoint(current_point_along_the_primitive[0], current_point_along_the_primitive[1]);

                // Set the cell to true
                cspace(current_point_cell.first, current_point_cell.second) = true;

                // Move the current point along the primitive
                current_point_along_the_primitive += primitive_unit_vector*step;
                distance_traveled += step;

            }
        }
            
        // Fill the interior of the obstacle
        for (std::size_t x = 0; x < m_cells_per_dim; ++x) {
            bool inside = false;
            for (std::size_t y = 0; y < m_cells_per_dim; ++y) {
                if (cspace(x, y)) { // I have hit an edge
                    // Move upwards until I hit another edge
                    y++;
                    inside = true;//!inside;
                    while (inside && y < m_cells_per_dim) {
                        if (cspace(x, y) == true) {
                            inside = false;
                        } else {
                            cspace(x, y) = true;
                        }
                        y++;
                    }
                }
            }
        }
                    
    }
    

    



    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}


amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
    // Implement your WaveFront algorithm here
    amp::Path2D path;
    path.waypoints.push_back(q_init);


    std::cout << "Planning from " << q_init[0] << ", " << q_init[1] << " to " << q_goal[0] << ", " << q_goal[1] << std::endl;

    // Set the goal cell to 2
    std::pair<std::size_t, std::size_t> q_goal_cell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
    std::vector<std::vector<int>> distance(grid_cspace.size().first, std::vector<int>(grid_cspace.size().second, -1));
    distance[q_goal_cell.first][q_goal_cell.second] = 2;


    // Set every cell that is in collision with a value of 1
    for (int i = 0; i < grid_cspace.size().first; i++) {
        for (int j = 0; j < grid_cspace.size().second; j++) {
            if (grid_cspace(i, j)) {
                distance[i][j] = 1;
            }
        }
    }
    
    // Starting from goal, set the distance of each neighboring cell to the current cell + 1
    // Any cell can have at most 4 neighbors, so check if any of those 4 are an obstacle, OW set them +=1
    // Given i,j, neighbors are (i-1, j), (i+1, j), (i, j-1), (i, j+1)
    
    int itr = 0;
    int i = q_goal_cell.first;
    int j = q_goal_cell.second;

    // Define a queue to store the cells that need to be checked
    std::queue<std::pair<std::size_t, std::size_t>> queue;
    queue.push(q_goal_cell);
    
    while (!queue.empty() && itr < 20000) {
        auto [i, j] = queue.front();
        queue.pop();
        int current_distance = distance[i][j];

        // Check the 4 neighbors (up, down, left, right)
        std::vector<std::pair<int, int>> neighbors = {{i-1, j}, {i+1, j}, {i, j-1}, {i, j+1}};
        for (const auto& [x, y] : neighbors) {
            if (x >= 0 && x < grid_cspace.size().first && y >= 0 && y < grid_cspace.size().second) {
                if (distance[x][y] == -1) { // If the cell is uninitialized
                    distance[x][y] = current_distance + 1;
                    //std::cout << "Setting cell " << x << ", " << y << " to " << current_distance + 1 << std::endl;
                    queue.push({x, y});
                }
            }
        }
        itr++;
    }

    std::cout << "Finished wavefront with " << itr << " iterations" << std::endl;
    
    std::pair<std::size_t, std::size_t> q_init_cell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    std::pair<std::size_t, std::size_t> current_cell = q_init_cell;

    std::cout << "Starting from cell " << current_cell.first << ", " << current_cell.second << " with distance " << distance[current_cell.first][current_cell.second] << " with waypoint " << q_init[0] << ", " << q_init[1] << std::endl;
    std::cout << "Goal cell is " << q_goal_cell.first << ", " << q_goal_cell.second << " with distance " << distance[q_goal_cell.first][q_goal_cell.second] << " with waypoint " << q_goal[0] << ", " << q_goal[1] << std::endl;
    itr = 0;
    // Move towards the goal by following the gradient of decreasing distance
    while (current_cell != q_goal_cell && itr < 5000) {
        int i = current_cell.first;
        int j = current_cell.second;
        int current_distance = distance[i][j];
        
        // Find the neighbor with the smallest distance value
        std::vector<std::pair<int, int>> neighbors = {{i-1, j}, {i+1, j}, {i, j-1}, {i, j+1}};
        std::pair<int, int> next_cell = current_cell;
        int min_distance = current_distance;

        for (const auto& [x, y] : neighbors) {
            if (x >= 0 && x < grid_cspace.size().first && y >= 0 && y < grid_cspace.size().second) {
                if (distance[x][y] > 1 && distance[x][y] < min_distance) {  // Follow decreasing distance
                    min_distance = distance[x][y];
                    next_cell = {x, y};
                }
            }
        }
        
        current_cell = next_cell;
        i = current_cell.first;
        j = current_cell.second;

        //std::cout << "Moving to cell " << current_cell.first << ", " << current_cell.second  << " because it has distance " << min_distance << " which is less than " << current_distance;

        // Convert grid cell to world coordinates and add to the path
        double x0_min, x0_max, x1_min, x1_max, x0_step, x1_step;
        std::size_t x0_cell, x1_cell;

        // Get the bounds of the configuration space
        std::tie(x0_min, x0_max) = grid_cspace.x0Bounds();
        std::tie(x1_min, x1_max) = grid_cspace.x1Bounds();
        std::tie(x0_cell, x1_cell) = grid_cspace.size();

        // Calculate the step size for each cell
        x0_step = (x0_max - x0_min) / x0_cell;
        x1_step = (x1_max - x1_min) / x1_cell;

        // Compute the point from the cell index
        double x0 = x0_min + i * x0_step;
        double x1 = x1_min + j * x1_step;
        Eigen::Vector2d waypoint = Eigen::Vector2d(x0, x1);
        path.waypoints.push_back(waypoint);
        //std::cout << " and adding waypoint " << waypoint[0] << ", " << waypoint[1] << std::endl;
        itr++;
    }

    std::cout << "Finished path with " << itr << " iterations" << std::endl;


    path.waypoints.push_back(q_goal);
    return path;
}