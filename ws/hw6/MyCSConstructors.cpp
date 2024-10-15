#include "MyCSConstructors.h"
#include <queue>
#include <iomanip>
#include <ctime>
#include <sstream>

////////////////////// THIS IS FROM HW4 //////////////////////

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/

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

    // Clamp the cell indices to the bounds of the cspace
    cell_x = std::min(cell_x, x0_cell - 1);
    cell_y = std::min(cell_y, x1_cell - 1);    
    
    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0, 2*M_PI, 0, 2*M_PI);
    //std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, -M_PI, M_PI, -M_PI, M_PI);
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

    // // Define the bounds of the cspace
    double x0_min = 0;
    double x0_max = 2 * M_PI;
    double x1_min = 0;
    double x1_max = 2 * M_PI;
    double step = 2 * M_PI / m_cells_per_dim;

    // double x0_min = -M_PI;
    // double x0_max = M_PI;
    // double x1_min = -M_PI;
    // double x1_max = M_PI;
    // double step = 2 * M_PI / m_cells_per_dim;

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

                // Check if the link is in collision with any obstacle
                in_collision = cspace.next_step_collision(all_primitives, link);

                // If the link is in collision, break out of the loop this since is invalid configuration
                if (in_collision == true) {
                    cspace(i,j) = true;
                    break;
                }
            }
            if (in_collision == false) {
                cspace(i,j) = false;
            }

        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}


// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {

    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;

    // Define the expansion factor for the C-space
    double expansion_factor = 0.1;
    
    // For a point agent, the C-space is going to be the cells that contain an obstacle
    for (auto& obstacle : env.obstacles) {
        std::vector<Eigen::Vector2d> expanded_vertices(obstacle.verticesCW().size());

        for (int v = 0; v < obstacle.verticesCW().size(); v++) {
            Eigen::Vector2d vertex1, vertex2, edge, edge_normal, expanded_vertex1, expanded_vertex2;
            
            if (v == obstacle.verticesCW().size() - 1) {
                vertex1 = obstacle.verticesCW()[v];
                vertex2 = obstacle.verticesCW()[0];
            } else {
                vertex1 = obstacle.verticesCW()[v];
                vertex2 = obstacle.verticesCW()[v+1];
            }

            // Find the cells that bound this current linear primitive
            std::pair<std::size_t, std::size_t> vertex1_cell = cspace.getCellFromPoint(vertex1[0], vertex1[1]);
            std::pair<std::size_t, std::size_t> vertex2_cell = cspace.getCellFromPoint(vertex2[0], vertex2[1]);

            // Bresenham's Line Algorithm that sets all cells that the line passes through to true
            int x1 = vertex1_cell.first;
            int y1 = vertex1_cell.second;
            int x2 = vertex2_cell.first;
            int y2 = vertex2_cell.second;

            int dx = abs(x2 - x1);
            int dy = abs(y2 - y1);
            int sx = (x1 < x2) ? 1 : -1;
            int sy = (y1 < y2) ? 1 : -1;
            int err = dx - dy;

            while (true) {
                // Set the current cell to true
                cspace(x1, y1) = true;

                // Expand the marked cells around the line, based on the expansion factor
                for (int i = -2; i <= 2; i++) {
                    for (int j = -2; j <= 2; j++) {
                        int new_x = x1 + i;
                        int new_y = y1 + j;
                        if (new_x >= 0 && new_x < m_cells_per_dim && new_y >= 0 && new_y < m_cells_per_dim) {
                            // Mark the expanded cells
                            cspace(new_x, new_y) = true;
                        }
                    }
                }

                if (x1 == x2 && y1 == y2) break;

                int e2 = 2 * err;
                if (e2 > -dy) {
                    err -= dy;
                    x1 += sx;
                }
                if (e2 < dx) {
                    err += dx;
                    y1 += sy;
                }
            }
        }
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}


// Wavefront Planner in Cspace
amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Implement your WaveFront algorithm here
    amp::Path2D path;

    Eigen::Vector2d q_init_new, q_goal_new;
    q_init_new = q_init;
    q_goal_new = q_goal;
    
    if(isManipulator){
        if (q_init[0] < 0){
            q_init_new[0] = 2*M_PI + q_init[0];
        }
        if (q_init[1] < 0){
            q_init_new[1] = 2*M_PI + q_init[1];
        }
        if (q_goal[0] < 0){
            q_goal_new[0] = 2*M_PI + q_goal[0];
        }
        if (q_goal[1] < 0){
            q_goal_new[1] = 2*M_PI + q_goal[1];
        }
    }

    // Push back the initial point
    path.waypoints.push_back(q_init_new);

    // Initially set every value on the grid to zero
    std::vector<std::vector<int>> distance(grid_cspace.size().first, std::vector<int>(grid_cspace.size().second, 0));
   
    // Get the cell that the goal point is in
    std::pair<std::size_t, std::size_t> q_goal_cell = grid_cspace.getCellFromPoint(q_goal_new[0], q_goal_new[1]);
    distance[q_goal_cell.first][q_goal_cell.second] = 2;

    if(grid_cspace(q_goal_cell.first, q_goal_cell.second) == 1){
        std::cout << "Goal cell is in collision" << std::endl;
       // To fix this find the other way to make this not in collision
    }

    // Get the cell that the initial point is in
    std::pair<std::size_t, std::size_t> q_init_cell = grid_cspace.getCellFromPoint(q_init_new[0], q_init_new[1]);

        if (grid_cspace(q_init_cell.first, q_init_cell.second) == 1) {
        std::cout << "The initial cell is in collision" << std::endl;
    }

    // Set every cell that is in collision with a value of 1
    for (int i = 0; i < grid_cspace.size().first; i++) {
        for (int j = 0; j < grid_cspace.size().second; j++) {
            if (grid_cspace(i, j)) { 
                distance[i][j] = 1; // set the distance to 1 if its an obstacle
            }
        }
    }
    
    // Starting from goal, set the distance of each neighboring cell to the current cell + 1
    // Any cell can have at most 4 neighbors, so check if any of those 4 are an obstacle, OW set them +=1
    // Given i,j, neighbors are (i-1, j), (i+1, j), (i, j-1), (i, j+1)
    
    // Initialize the iteration count
    int itr = 0;

    // Starting at the goal cell
    int i = q_goal_cell.first;
    int j = q_goal_cell.second;

    // Define a queue to store the cells that need to be checked
    std::queue<std::pair<std::size_t, std::size_t>> queue;
    
    // Push the goal cell to the queue
    queue.push(q_goal_cell);

    // Max iterations
    double max_itr = grid_cspace.size().first * grid_cspace.size().second;
    std::size_t x_bounds = grid_cspace.size().first;
    std::size_t y_bounds = grid_cspace.size().second;
    
    
    // Iterate until the queue is empty or the max number of iterations is reached
    while (itr < max_itr) {

        // Pop the front of the queue
        if (queue.empty()) {
            std::cout << "Queue is empty" << std::endl << std::endl;
            
            // Display the column indices with consistent formatting
            bool disp = false;
            if (disp) {
                // Generate a unique file name based on the current timestamp
                std::time_t t = std::time(nullptr);
                std::tm* now = std::localtime(&t);
                std::stringstream file_name;
                file_name << "/home/aidanbagley/AMP-Tools-public/ws/hw6/logs/grid_cspace_dump_" << (now->tm_year + 1900) << '_'
                        << (now->tm_mon + 1) << '_' << now->tm_mday << '_'
                        << now->tm_hour << now->tm_min << now->tm_sec << ".txt";

                
                // Open the file for writing
                std::cout << "Dumping grid data to " << file_name.str() << std::endl;
                std::ofstream dump_file(file_name.str());
                
                if (!dump_file.is_open()) {
                    std::cerr << "Error opening file for writing." << std::endl;
                    return path;
                }

                // Write the column indices to the file
                dump_file << "      ";  // Initial padding for row index column
                for (std::size_t j = 0; j < y_bounds; ++j) {
                    dump_file << std::setw(3) << j << " ";  // Use std::setw(3) for better spacing of two-digit indices
                }
                dump_file << std::endl;

                // Write the grid with row indices to the file
                for (std::size_t i = 0; i < x_bounds; ++i) {
                    dump_file << std::setw(2) << i << "    ";  // Row index with spacing
                    for (std::size_t j = 0; j < y_bounds; ++j) {
                        dump_file << std::setw(3) << (grid_cspace(i, j) ? "1" : "0") << " ";  // Align grid values
                    }
                    dump_file << std::endl;
                }

                dump_file << std::endl;
                dump_file << "The initial point was: " << q_goal_new.transpose() << std::endl;
                dump_file << "The goal point was: " << q_init_new.transpose() << std::endl;
                dump_file << "The initial cell was: " << q_init_cell.first << ", " << q_init_cell.second << std::endl;
                dump_file << "The goal cell was: " << q_goal_cell.first << ", " << q_goal_cell.second << std::endl <<std::endl;

                // Close the file
                dump_file.close();

                std::cout << "Grid data dumped to grid_cspace_dump.txt" << std::endl;
            }

            
            break;
        }

        
        // Get the cell indicies from the front of the queue
        std::pair<std::size_t, std::size_t> current_cell = queue.front();
        queue.pop();

        if (current_cell == q_init_cell) {
            std::cout << "Reached the initial cell" << std::endl;
            break;
        }

        std::size_t i = current_cell.first;
        std::size_t j = current_cell.second;


        // Get the current distance value for this cell
        int current_distance = distance[i][j];

        // std::cout << "Current cell: i = " << i << " j = " << j << " distance = " << current_distance << std::endl;

        // For each of the 4 neighbors (up, down, left, right) take current distance and add 1
        std::vector<std::pair<int, int>> neighbors = {{i-1, j}, {i+1, j}, {i, j-1}, {i, j+1}};
        for (const auto& [x, y] : neighbors) {
            if (x >= 0 && x < x_bounds && y >= 0 && y < y_bounds && distance[x][y] == 0) {  // Not an obstacle
                distance[x][y] = current_distance + 1;
                queue.push({x, y}); 
            }
        }
        
        // If I am a manipulator, then I may need to wrap around the grid if I havent gotten there yet
        if (isManipulator){
            if (i == x_bounds - 1 && j == y_bounds - 1 && distance[0][0] == 0) { // Top right
                distance[0][0] = current_distance + 1;
                // std::cout << "Top right" << std::endl;
                queue.push({0, 0});
            }
            if (i == 0 && j == 0 && distance[x_bounds - 1][y_bounds - 1] == 0) { // Bottom left
                distance[x_bounds - 1][y_bounds - 1] = current_distance + 1;
                // std::cout << "Bottom left i = " << i << " j = " << j << std::endl;
                queue.push({x_bounds - 1, y_bounds - 1});
            }
            if (i == x_bounds - 1 && distance[0][j] == 0) { // Top row
                distance[0][j] = current_distance + 1;
                // std::cout << "Top row" << std::endl;
                queue.push({0, j});
            }
            if (i == 0 && distance[x_bounds - 1][j] == 0) { // Bottom row
                distance[x_bounds - 1][j] = current_distance + 1;
                // std::cout << "Bottom row i = " << i << " j = " << j << std::endl;
                queue.push({x_bounds - 1, j});
            }
            if (j == 0 && distance[i][y_bounds - 1] == 0) { // Left column
                distance[i][y_bounds - 1] = current_distance + 1;
                // std::cout << "Left Column i = " << i << " j = " << j << std::endl;
                queue.push({i, y_bounds - 1});
            }
            if (j == y_bounds - 1 && distance[i][0] == 0) { // Right column
                distance[i][0] = current_distance + 1;
                // std::cout << "Right column" << std::endl;
                queue.push({i, 0});
            }
        }

        itr++;
    }

    std::cout << "Finished brushfire with " << itr << " iterations" << std::endl;
    
    // Starting at the initial cell
    std::pair<std::size_t, std::size_t> current_cell = q_init_cell;

    // Initialize the iteration count
    itr = 0;
    
    // Move towards the goal by following the gradient of decreasing distance
    while (current_cell != q_goal_cell && itr < 10000) {
        int i = current_cell.first;
        int j = current_cell.second;
        int current_distance = distance[i][j];
        
        // Find the quadrant neighbor with the smallest distance value
        std::vector<std::pair<int, int>> neighbors = {{i-1, j}, {i+1, j}, {i, j-1}, {i, j+1}};
        
        // Search for the neighbor with the smallest distance value
        std::pair<int, int> next_cell = {-1,-1};
        int min_distance = current_distance;

        for (const auto& [x, y] : neighbors) {
            if (x >= 0 && x < x_bounds && y >= 0 && y < y_bounds) {
                if (distance[x][y] > 1 && distance[x][y] < min_distance) {  // Follow decreasing distance
                    min_distance = distance[x][y];
                    next_cell = {x, y};
                }
            }
        }

        if(isManipulator){ // If I am a manipulator, then I need to wrap around the grid and brushfire finsihed so a path exists
            if (next_cell == std::pair<int, int>{-1, -1}) {
                if (i == x_bounds - 1 && j == y_bounds - 1 && distance[0][0] > 1) { // Top right
                    next_cell = {0, 0};
                }
                else if (i == 0 && j == 0 && distance[x_bounds - 1][y_bounds - 1]  > 1) { // Bottom left
                    next_cell = {x_bounds - 1, y_bounds - 1};
                }
                else if (i == x_bounds - 1 && distance[0][j]  > 1) { // Top row
                    next_cell = {0, j};
                }
                else if (i == 0 && distance[x_bounds - 1][j]  > 1) { // Bottom row
                    next_cell = {x_bounds - 1, j};
                }
                else if (j == 0 && distance[i][y_bounds - 1]  > 1) { // Left column
                    next_cell = {i, y_bounds - 1};
                }
                else if (j == y_bounds - 1 && distance[i][0]  > 1) { // Right column
                    next_cell = {i, 0};
                }
                else {
                    std::cout << "Bruh what: i =  " << i << " j = " << j << " current_distance = " << current_distance << std::endl;
                    break;
                }
            }
        }


        // Given the next cell, find the real waypoint for this cell
        current_cell = next_cell;
        i = current_cell.first;
        j = current_cell.second;

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
        itr++;
    }
    path.waypoints.push_back(q_goal_new);
    if (isManipulator) {
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
    }
    return path;
}


// Linear Primitive Collision Checker
bool MyGridCSpace2D::next_step_collision(const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step){
    
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
