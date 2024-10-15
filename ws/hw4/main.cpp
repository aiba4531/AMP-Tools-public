// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;

int main(int argc, char** argv) {

    MyClass my_class;
    // std::vector<Eigen::Vector2d> vertices = my_class.Minkowski();

    // // Create a polygon with thesee verticies
    // Polygon polygon(vertices);
    // std::vector<Polygon> polygons = {polygon};
    // Visualizer::makeFigure(polygons);

    std::vector<std::vector<Eigen::Vector2d>> all_new_vertices;
    all_new_vertices = my_class.Minkowski_rotation();
    std::vector<Polygon> polygons;

    // Create a 12 polygons with thesee verticies
    for (int i = 0; i < all_new_vertices.size(); i++) {
        Polygon polygon(all_new_vertices[i]);
        polygons.push_back(polygon);
    }

    std::vector<double> rotations = {0, M_PI/6, M_PI/3, M_PI/2, 2*M_PI/3, 5*M_PI/6, M_PI, 7*M_PI/6, 4*M_PI/3, 3*M_PI/2, 5*M_PI/3, 11*M_PI/6};

    Visualizer::makeFigure(polygons, rotations);
    

    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    // amp::RNG::seed(amp::RNG::randiUnbounded());

    // std::vector<double> link_lengths = {0.5, 1.0, 0.5};
    // MyManipulator2D manipulator(link_lengths);

    // // You can visualize your manipulator given an angle state like so:
    // amp::ManipulatorState test_state = Eigen::VectorXd::Zero(manipulator.nLinks());
    // test_state << M_PI/6, M_PI/3, 7*M_PI/4;

    // // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    // Visualizer::makeFigure(manipulator, test_state); 

    // // You can also visualize the inverse kinematics solution for a given end effector location
    // std::vector<double> link_lengths2 = {1.0, 0.5, 1.0};
    // MyManipulator2D manipulator2(link_lengths2);  
    // amp::ManipulatorState IK_State = manipulator2.getConfigurationFromIK(Eigen::Vector2d(2.0,  0.0));
        
    // Visualizer::makeFigure(manipulator2, IK_State);


    //Create the collision space constructor
    MyManipulator2D manipulator3(std::vector<double>{1.0, 1.0});
    std::size_t n_cells = 500;

    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator3, HW4::getEx3Workspace3());

    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace);

    // Visualizer::makeFigure(HW4::getEx3Workspace1());

    //Visualizer::showFigures();

    // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "aidan.bagley@colorado.edu", argc, argv);
    return 0;
}