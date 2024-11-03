// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"

using namespace amp;

// int main(int argc, char** argv) {

//     // Initialize a random seed
//     amp::RNG::seed(amp::RNG::randiUnbounded());

//     // Number of 2D agents
//     int m = 6;

//     // Get 2D workspace with m agents
//     MultiAgentProblem2D problem = HW8::getWorkspace1(m);

//     // Vector 2D to store collision states
//     std::vector<std::vector<Eigen::Vector2d>> collision_states = {{}};

//     // Construct central planner
//     double r = 0.5; // radius for the RRT
//     double goal_bias = 0.05; // goal bias for the RRT
//     int max_itr = 7500; // maximum iterations for the RRT
//     double epsilon = 0.25; // epsilon for the RRT
//     MyCentralPlanner central_planner(r, goal_bias, max_itr, epsilon);
//     MyDecentralPlanner decentral_planner(r, goal_bias, max_itr, epsilon);

//     // Start a timer
//     bool once = true, success = false; // Flags for visualization
//     double start_time, elapsed_time;
//     Timer timer("timer"); // Create a timer

//     std::vector<double> elapsed_times; // Store elapsed times for each iteration
//     std::vector<double> num_nodes; // Store number of nodes for each iteration

//     for (int i = 0; i < 100; i++) {

//         collision_states.clear(); // Clear collision states for each iteration

//         // Start the timer
//         start_time = timer.now(TimeUnit::ms); 

//         // Solve using a centralized approach
//         MultiAgentPath2D path = decentral_planner.plan(problem);

//         // Calculate elapsed time
//         elapsed_time = timer.now(TimeUnit::ms) - start_time;

//         // Check if the path is valid
//         success = HW8::check(path, problem, collision_states);

//         if (success) {
//             elapsed_times.push_back(elapsed_time);
//             //num_nodes.push_back(central_planner.getNodes().size());

//             if (once) {
//                 Visualizer::makeFigure(problem, path, collision_states);
//                 once = false;
//             }
//         }
//     }

//     // Compute average
//     double avg_time = std::accumulate(elapsed_times.begin(), elapsed_times.end(), 0.0) / elapsed_times.size();
//     //double avg_nodes = std::accumulate(num_nodes.begin(), num_nodes.end(), 0.0) / num_nodes.size();

//     std::cout << "For m = " << m << std::endl;
//     std::cout << "Average computation time: " << avg_time << " ms" << std::endl;
//     //std::cout << "Average number of nodes: " << avg_nodes << std::endl;
//     std::cout << "Total number of successful runs: " << elapsed_times.size() << std::endl;

//     // Prepare data sets and labels for the box plot
//     std::list<std::vector<double>> computation_times_data_sets = {elapsed_times};
//     //std::list<std::vector<double>> size_of_tree_data_sets = {num_nodes};

//     // Prepare labels for the box plot
//     std::vector<std::string> labels = { "n=7500, r=0.5, goal_bias=0.05, epsilon=0.25"};

//     // Call makeBoxPlot with the prepared data sets and labels
//     // Visualizer::makeBoxPlot(computation_times_data_sets, labels, "Run Times for Centralized RRT", "Parameters", "Computation Time [ms]");
//     // Visualizer::makeBoxPlot(size_of_tree_data_sets, labels, "Size of Tree for Centralized RRT", "Parameter Combinations", "Number of Nodes");
  
//     Visualizer::makeBoxPlot(computation_times_data_sets, labels, "Run Times for Decentralized RRT", "Parameters", "Computation Time [ms]");
//     //Visualizer::makeBoxPlot(size_of_tree_data_sets, labels, "Size of Tree for Decentralized RRT", "Parameter Combinations", "Number of Nodes");
  
//     //Visualize and grade methods
//     Visualizer::showFigures();
//     return 0;
// }


int main(int argc, char** argv) {
    // Initialize a random seed
    amp::RNG::seed(amp::RNG::randiUnbounded());

    MyCentralPlanner central_planner;
    MyDecentralPlanner decentral_planner;

    MultiAgentProblem2D problem = HW8::getWorkspace1(4); // Get a workspace with 3 agents
    MultiAgentPath2D path;// = central_planner.plan(problem);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;
    bool success = false;
    // int itr = 0;
    // while( !success && itr < 1) {
    //     collision_states.clear();
    //     path = central_planner.plan(problem);
    //     success = HW8::check(path, problem, collision_states);
    //     Visualizer::makeFigure(problem, path, collision_states);
    //     itr++;
    // }
    //Visualizer::makeFigure(problem, path, collision_states);
    HW8::grade<MyCentralPlanner, MyDecentralPlanner>("aidan.bagley@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    //Visualizer::showFigures();
    return 0;
}