// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <list>
#include "MySamplingBasedPlanners.h"

using namespace amp;

// int main(int argc, char** argv) {

//     // Test PRM on Workspace1 of HW2
//     //Problem2D problem = HW5::getWorkspace1();
//     Problem2D problem = HW2::getWorkspace2();

//     // For 100 runs
//     const int num_runs = 100;

//     // Variables to store the number of valid paths, run times, and path lengths
//     // std::vector<double> valid_paths(8);
//     // std::vector<std::vector<double>> run_times(8);
//     // std::vector<std::vector<double>> path_lengths(8);
//     std::vector<double> valid_paths(6);
//     std::vector<std::vector<double>> run_times(6);
//     std::vector<std::vector<double>> path_lengths(6);


//     // Define the parameter combinations to test
//     // const std::vector<double> n_values = {200, 200, 200, 200, 500, 500, 500, 500};
//     // const std::vector<double> r_values = {0.5, 1, 1.5, 2, 0.5, 1, 1.5, 2};
//     const std::vector<double> n_values = {200, 200, 500, 500, 1000, 1000};
//     const std::vector<double> r_values = {1, 2, 1, 2, 1 ,2};

//     for (int p = 0; p < n_values.size() ; p++){ // for each parameter combination

//         for (int i = 0; i < num_runs; i++) {
//             // Define the PRM object

//             MyPRM prm(n_values[p], r_values[p]);
    

//             // Time the planning process
//             auto start = std::chrono::high_resolution_clock::now();
//             Path2D path = prm.plan(problem);
//             auto end = std::chrono::high_resolution_clock::now();


//             std::chrono::duration<double> elapsed = end - start;
//             run_times[p].push_back(elapsed.count()); 

//             // Valid path
//             if(path.waypoints.size() > 0) {
//                 valid_paths[p]++;
//                 path_lengths[p].push_back(path.length());
//             }

//             std::cout << "Finished run " << i << " of " << num_runs << " for n=" << n_values[p] << " and r=" << r_values[p] << std::endl;
//         }
//     }
    
//     // Prepare data sets and labels for the box plot
//     // std::list<std::vector<double>> run_times_data_sets = {run_times[0], run_times[1], run_times[2], run_times[3], run_times[4], run_times[5], run_times[6], run_times[7]};
//     // std::list<std::vector<double>> path_lengths_data_sets = {path_lengths[0], path_lengths[1], path_lengths[2], path_lengths[3], path_lengths[4], path_lengths[5], path_lengths[6], path_lengths[7]};
//     std::list<std::vector<double>> run_times_data_sets = {run_times[0], run_times[1], run_times[2], run_times[3], run_times[4], run_times[5]};
//     std::list<std::vector<double>> path_lengths_data_sets = {path_lengths[0], path_lengths[1], path_lengths[2], path_lengths[3], path_lengths[4], path_lengths[5]};



//     // Prepare labels for the box plot
//     //std::vector<std::string> labels = {"n=200, r=0.5", "n=200, r=1", "n=200, r=1.5", "n=200, r=2", "n=500, r=0.5", "n=500, r=1", "n=500, r=1.5", "n=500, r=2"};
//     std::vector<std::string> labels = { "n=200, r=1", "n=200, r=2", "n=500, r=1", "n=500, r=2", "n=1000, r=1", "n=1000, r=2"};


//     // Call makeBoxPlot with the prepared data sets and labels
//     Visualizer::makeBoxPlot(run_times_data_sets, labels, "PRM Run Times", "Parameter Combinations", "Run Times");
//     Visualizer::makeBoxPlot(path_lengths_data_sets, labels, "PRM Path Lengths", "Parameter Combinations", "Path Lengths");
//     Visualizer::makeBarGraph(valid_paths, labels, "PRM Valid Paths", "Parameter Combinations", "Valid Paths");
//     Visualizer::showFigures();
    
//     //HW7::grade<MyPRM, MyRRT>("aidan.bagley@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());

//     return 0;
    
// }


int main( int argc, char** argv ) {
    // Test RRT on Workspace1 of HW2
    Problem2D problem = HW2::getWorkspace2();

    // For 100 runs
    const int num_runs = 100;

    // Variables to store the number of valid paths, run times, and path lengths
    std::vector<double> valid_paths(1);
    std::vector<std::vector<double>> run_times(1);
    std::vector<std::vector<double>> path_lengths(1);

    // Define the parameter combinations to test
    const std::vector<double> n_values = {5000};
    const std::vector<double> r_values = {0.5};
    const std::vector<double> goal_bias_values = {0.05};
    const std::vector<double> epsilon_values = {0.25};

    for (int p = 0; p < n_values.size() ; p++){ // for each parameter combination

        for (int i = 0; i < num_runs; i++) {
            // Define the RRT object
            MyRRT rrt(n_values[p], r_values[p], goal_bias_values[p], epsilon_values[p]);

            // Time the planning process
            auto start = std::chrono::high_resolution_clock::now();
            Path2D path = rrt.plan(problem);
            auto end = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double> elapsed = end - start;
            run_times[p].push_back(elapsed.count()); 

            // Valid path
            if(path.waypoints.size() > 0) {
                valid_paths[p]++;
                path_lengths[p].push_back(path.length());
            }

            std::cout << "Finished run " << i << " of " << num_runs << " for n=" << n_values[p] << std::endl;
        }
    }

    // Prepare data sets and labels for the box plot
    std::list<std::vector<double>> run_times_data_sets = {run_times[0]};
    std::list<std::vector<double>> path_lengths_data_sets = {path_lengths[0]};

    // Prepare labels for the box plot
    std::vector<std::string> labels = { "n=5000, r=0.1, goal_bias=0.05, epsilon=0.25"};

    // Call makeBoxPlot with the prepared data sets and labels
    Visualizer::makeBoxPlot(run_times_data_sets, labels, "RRT Run Times", "Parameter Combinations", "Run Times");
    Visualizer::makeBoxPlot(path_lengths_data_sets, labels, "RRT Path Lengths", "Parameter Combinations", "Path Lengths");
    Visualizer::makeBarGraph(valid_paths, labels, "RRT Valid Paths", "Parameter Combinations", "Valid Paths");

    Visualizer::showFigures();

    return 0;
}  // end of main

// int main(int argc, char** argv) {

//     // Test PRM on Workspace1 of HW2
//     Problem2D problem = HW2::getWorkspace2();
//     // for (int i = 0; i < 100; i++){
//     //     MyPRM prm(500, 2.0);
//     //     Path2D path = prm.plan(problem);
//     //     std::shared_ptr<Graph<double>> graphPtr = prm.get_graphPtr();
//     //     std::map<Node, Eigen::Vector2d> nodes = prm.get_nodes();
        
//     //     if (path.waypoints.size() > 0){
//     //         std::cout << "Path length:" << path.length()  << std::endl;

//     //         std::shared_ptr<Graph<double>> graphPtr = prm.get_graphPtr();
//     //         std::map<Node, Eigen::Vector2d> nodes = prm.get_nodes();
//     //         Visualizer::makeFigure(problem, path, *graphPtr, nodes);
//     //         break;

//     //     }
//     // }

//     for (int i = 0; i < 100; i++){
//         MyRRT rrt(5000, 0.5, 0.05, 0.25);
//         Path2D path = rrt.plan(problem);
//         std::shared_ptr<Graph<double>> graphPtr = rrt.get_graphPtr();
//         std::map<Node, Eigen::Vector2d> nodes = rrt.get_nodes();
        
//         if (path.waypoints.size() > 0){
//             std::cout << "Path length:" << path.length()  << std::endl;

//             std::shared_ptr<Graph<double>> graphPtr = rrt.get_graphPtr();
//             std::map<Node, Eigen::Vector2d> nodes = rrt.get_nodes();
//             Visualizer::makeFigure(problem, path, *graphPtr, nodes);
//             break;

//         }
//     }

//     Visualizer::showFigures();

//     // Grade method
//     // HW7::grade<MyPRM, MyRRT>("aidan.bagley@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
//     return 0;
// }