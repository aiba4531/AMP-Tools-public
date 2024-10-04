#include "CSpaceSkeleton.h"

// Override this method for returning whether or not a point is in collision

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

    // std::cout << "Cell: " << cell_x << ", " << cell_y << std::endl;

    return {cell_x, cell_y};
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
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 

    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0, 2*M_PI, 0, 2*M_PI);

    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    // cspace(1, 3) = true;
    // cspace(3, 3) = true;
    // cspace(0, 1) = true;
    // cspace(1, 0) = true;
    // cspace(2, 0) = true;
    // cspace(3, 0) = true;
    // cspace(4, 1) = true;

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
