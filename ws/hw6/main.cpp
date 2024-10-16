#include "AMPCore.h"

#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW6.h"

#include "MyAStar.h"
#include "MyCSConstructors.h"
#include "ManipulatorSkeleton.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // You will need your 2-link manipulator from HW4
    MyManipulator2D manipulator;
    
    // Define a point robot problem
    //Problem2D point_problem = HW2::getWorkspace1();
    Problem2D point_problem = HW2::getWorkspace2();
    //Problem2D point_problem = HW5::getWorkspace1();
    
    // Define a manipulator problem
    Problem2D manip_problem = HW6::getHW4Problem3();

    // Define a wavefront algorithm
    std::shared_ptr<WaveFrontAlgorithm> wf_algo = std::make_shared<MyWaveFrontAlgorithm>();

    // Construct point-agent and manipulator cspace instances
    std::size_t point_n_cells = 240;
    std::size_t mainip_n_cells = 36; 

    // Construct the cspace for the point-agent
    std::shared_ptr<MyPointAgentCSConstructor> point_agent_ctor = std::make_shared<MyPointAgentCSConstructor>(point_n_cells);
    std::unique_ptr<amp::GridCSpace2D> point_cspace = point_agent_ctor->construct(point_problem);
    
    // Combine your wavefront planner with a cspace object (you do not need to modify these classes).
    PointWaveFrontAlgorithm point_algo(wf_algo, point_agent_ctor);
    

    // Construct the cspace for the manipulator
    std::shared_ptr<MyManipulatorCSConstructor> manipulator_ctor = std::make_shared<MyManipulatorCSConstructor>(mainip_n_cells);
    std::unique_ptr<amp::GridCSpace2D> manipulator_cspace = manipulator_ctor->construct(manipulator, manip_problem);
   
    // Populate the cspace cells with collision values for visulaization.
    ManipulatorWaveFrontAlgorithm manip_algo(wf_algo, manipulator_ctor);


    // Solve the problems using the wavefront algorithm


    // // Return a path for the point-agent using c-space planning.
    // Path2D path = point_algo.plan(point_problem);
    // Visualizer::makeFigure(point_problem, path); // Visualize path in workspace
    // Visualizer::makeFigure(*point_cspace, path); // Visualize path in cspace
    // std::cout << "Path length for point robot: " << path.length() << std::endl;

    // Return a path for the manipulator using c-space planning.
    // ManipulatorTrajectory2Link trajectory = manip_algo.plan(manipulator, manip_problem);
    // Visualizer::makeFigure(manip_problem, manipulator, trajectory);
    // Visualizer::makeFigure(*manipulator_cspace, trajectory);
    // std::cout << "Path length for manipulator: " << trajectory.length() << std::endl;

    // For Exercise 3, you will need to implement the A* algorithm.
    ShortestPathProblem problem = HW6::getEx3SPP();
    LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();
    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult result = algo.search(problem, heuristic);

    //Visualizer::showFigures();

    //
    amp::HW6::grade<PointWaveFrontAlgorithm, ManipulatorWaveFrontAlgorithm, MyAStarAlgo>("aidan.bagley@colorado.edu", argc, argv, std::make_tuple(wf_algo, point_agent_ctor), std::make_tuple(wf_algo, manipulator_ctor), std::make_tuple());
    return 0;
}