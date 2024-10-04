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