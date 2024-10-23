#include "HelpfulClass.h"

// void MyClass::hereIsAMethod() {
// }

std::vector<Eigen::Vector2d> MyClass::Minkowski(void) {
    // Define two convex polygons with CCW vertices starting at the lower left vertex
    std::vector<Eigen::Vector2d> robot = {Eigen::Vector2d(-1,-2), Eigen::Vector2d(0, 0), Eigen::Vector2d(0, -2)};
    std::vector<Eigen::Vector2d> obstacle = {Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 2), Eigen::Vector2d(1, 2)};

    std::vector<Eigen::Vector2d> cspace_vertices;
    bool is_in_cspace = false;
    // Loop through all combinations of vertices from both polygons
    for (const auto& r : robot) {
        for (const auto& o : obstacle) {
            is_in_cspace = false;
            // Minkowski sum: add corresponding vertices
            // If this vertex is not already a part of the C-space, add it
            for (const auto& vertex : cspace_vertices) {
                if (vertex == o + r) {
                    is_in_cspace = true;
                    break;
                }
            }
            if (!is_in_cspace) {
                cspace_vertices.push_back(o + r);
            }
        }
    }

    // Reorder all verticies counter clockwise starting from lower left
    std::vector <Eigen::Vector2d> cspace_vertices_ccw;

    cspace_vertices_ccw.push_back(cspace_vertices[0]);
    cspace_vertices_ccw.push_back(cspace_vertices[1]);
    cspace_vertices_ccw.push_back(cspace_vertices[3]);
    cspace_vertices_ccw.push_back(cspace_vertices[4]);
    cspace_vertices_ccw.push_back(cspace_vertices[6]);
    cspace_vertices_ccw.push_back(cspace_vertices[5]);

    for (const auto& vertex : cspace_vertices_ccw) {
        //std::cout << "C-Space Vertices CCW: " << vertex[0] << ", " << vertex[1] << std::endl;
    }

    return cspace_vertices_ccw;
}

std::vector<std::vector<Eigen::Vector2d>> MyClass::Minkowski_rotation(void) {
    // Define two convex polygons with CCW vertices starting at the lower left vertex
    std::vector<Eigen::Vector2d> robot = {Eigen::Vector2d(-1,-2), Eigen::Vector2d(0, 0), Eigen::Vector2d(0, -2)};
    std::vector<Eigen::Vector2d> obstacle = {Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 2), Eigen::Vector2d(1, 2)};

    // Define 12 rotations between zero and 2pi
    std::vector<double> rotations = {0, M_PI/6, M_PI/3, M_PI/2, 2*M_PI/3, 5*M_PI/6, M_PI, 7*M_PI/6, 4*M_PI/3, 3*M_PI/2, 5*M_PI/3, 11*M_PI/6};

    std::vector<std::vector<Eigen::Vector2d>> all_cspace_vertices;
    std::vector<Eigen::Vector2d> cspace_vertices;
    std::vector <Eigen::Vector2d> cspace_vertices_ccw;


    bool is_in_cspace = false;

    // Loop through all combinations of vertices from both polygons
    for (int i = 0; i < rotations.size(); i++) {
        std::cout << "Rotation: " << rotations[i] << std::endl;
        for (const auto& r : robot) {
            for (const auto& o : obstacle) {
                is_in_cspace = false;
                
                // Rotate the robot by rot
                Eigen::Matrix2d rot_matrix;
                rot_matrix << cos(rotations[i]), -sin(rotations[i]), sin(rotations[i]), cos(rotations[i]);
                Eigen::Vector2d r_rot = rot_matrix * r;

                
                // Minkowski sum: add corresponding vertices
                // If this vertex is not already a part of the C-space, add it

                for (const auto& vertex : cspace_vertices) {
                    if ((vertex - (o + r_rot)).norm() < 0.1) {
                        is_in_cspace = true;
                        break;
                    }
                }
                if (!is_in_cspace && (o + r_rot) != Eigen::Vector2d(0, 0)) {
                    cspace_vertices.push_back(o + r_rot);
                    //std::cout << "C-Space Vertices: " << o[0] + r_rot[0] << ", " << o[1] + r_rot[1] << std::endl;
                }
            }
        }
        cspace_vertices_ccw = sortVerticesCCW(cspace_vertices);
        // for (const auto& vertex : cspace_vertices_ccw) {
        //     std::cout << "C-Space Vertices CCW: " << vertex[0] << ", " << vertex[1] << std::endl;
        // }
        all_cspace_vertices.push_back(cspace_vertices_ccw);


        // Now we need to reorder the vertices in counter clockwise order starting from the lower left vertex but these have changed after a rotation form original




        // cspace_vertices_ccw.push_back(cspace_vertices[0]);
        // cspace_vertices_ccw.push_back(cspace_vertices[1]);
        // cspace_vertices_ccw.push_back(cspace_vertices[3]);
        // cspace_vertices_ccw.push_back(cspace_vertices[4]);
        // cspace_vertices_ccw.push_back(cspace_vertices[6]);
        // cspace_vertices_ccw.push_back(cspace_vertices[5]);
        //all_cspace_vertices.push_back(cspace_vertices_ccw);
    }


    return all_cspace_vertices;
}

// Function to find the lower-left vertex
Eigen::Vector2d  MyClass::findLowerLeft(const std::vector<Eigen::Vector2d>& vertices) {
    Eigen::Vector2d lowerLeft = vertices[0];
    for (const auto& v : vertices) {
        if (v[1] < lowerLeft[1] || (v[1] == lowerLeft[1] && v[0] < lowerLeft[0])) {
            lowerLeft = v;
        }
    }
    return lowerLeft;
}

// Function to compute the polar angle between two points
double  MyClass::polarAngle(const Eigen::Vector2d& origin, const Eigen::Vector2d& point) {
    return atan2(point[1] - origin[1], point[0] - origin[0]);
}

// Comparator to sort vertices CCW around the lower-left vertex
bool  MyClass::comparePolar(const Eigen::Vector2d& origin, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return polarAngle(origin, a) < polarAngle(origin, b);
}

// Function to sort vertices in CCW order starting from the lower-left vertex
std::vector<Eigen::Vector2d>  MyClass::sortVerticesCCW(std::vector<Eigen::Vector2d> vertices) {
    // Step 1: Find the lower-left vertex
    Eigen::Vector2d lowerLeft = findLowerLeft(vertices);

    // Step 2: Sort vertices by polar angle with respect to the lower-left vertex
    std::sort(vertices.begin(), vertices.end(), 
              [this, &lowerLeft](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
                  return comparePolar(lowerLeft, a, b);
              });

    return vertices;
}











// 2D distance from a point to a line segment
double distance_to_segment(const Eigen::Vector2d& circle_center, const Eigen::Vector2d& edge_start, const Eigen::Vector2d& edge_end) {
    // Calculate the length of the edge
    Eigen::Vector2d edge = edge_end - edge_start;
    double edge_length_squared = edge.squaredNorm();

    // Calculate the projection of the circle center onto the edge
    double t = ((circle_center - edge_start).dot(edge)) / edge_length_squared;

    // Clamp t to the range [0, 1] to find the closest point on the segment
    t = std::max(0.0, std::min(1.0, t));

    // Calculate the closest point on the segment
    Eigen::Vector2d closest_point = edge_start + t * edge;

    // Return the distance from the circle center to the closest point
    return (circle_center - closest_point).norm();
}

// Checking if a point is inside a polygon
bool point_in_polygons(Eigen::Vector2d point, const amp::MultiAgentProblem2D& problem) {   

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

// Robot Inside 2D Polygon
bool robot_in_polygons(const Eigen::Vector2d& robot_center, double radius, const amp::MultiAgentProblem2D& problem){
    // Check if the robot's center is inside the polygon
    if (point_in_polygons(robot_center, problem)) {
        return true;
    }

    // Check if the robot's radius intersects with the polygon edges
    for (const auto& obstacle : problem.obstacles) {
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCW();
        for (size_t i = 0; i < vertices.size(); ++i) {
            Eigen::Vector2d vertex1 = vertices[i];
            Eigen::Vector2d vertex2 = vertices[(i + 1) % vertices.size()];

            // Check if the distance from the robot center to the edge is less than the radius
            if (distance_to_segment(robot_center, vertex1, vertex2) < radius) {
                return true;
            }
        }
    }

    return false;
}

// 2D Robot Collision Check
bool robot_in_robot( const Eigen::Vector2d& robot1_center, double robot1_radius, const Eigen::Vector2d& robot2_center, double robot2_radius) { 
    double distance = (robot1_center - robot2_center).norm();
    return distance < (robot1_radius + robot2_radius);
}