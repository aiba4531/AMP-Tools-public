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

int main(int argc, char** argv) {

    // Test PRM on Workspace1 of HW2
    Problem2D problem = HW5::getWorkspace1();

    // Define the PRM object
    MyPRM prm;

    // For 100 runs
    const int num_runs = 100;

    // Variables to store the number of valid paths, run times, and path lengths
    std::vector<std::vector<double>> valid_paths(8);
    std::vector<std::vector<double>> run_times(8);
    std::vector<std::vector<double>> path_lengths(8);

    // Define the parameter combinations to test
    const std::vector<double> n_values = {200, 200, 200, 200, 500, 500, 500, 500};
    const std::vector<double> r_values = {0.5, 1, 1.5, 2, 0.5, 1, 1.5, 2};

    for (int p = 0; p < 8 ; p++){ // for each parameter combination
        prm.set_n(n_values[p]);
        prm.set_r(r_values[p]);

        for (int i = 0; i < num_runs; i++) {
            
            // Time the planning process
            auto start = std::chrono::high_resolution_clock::now();
            Path2D path = prm.plan(problem);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;

            run_times[p].push_back(elapsed.count());            
            // Valid path
            if(path.waypoints.size() > 0){
                valid_paths[p].push_back(1);
                path_lengths[p].push_back(path.length());
            }
            else {
                valid_paths[p].push_back(0);
                path_lengths[p].push_back(0);
            }
        }
    }
    
    // Prepare data sets and labels for the box plot
    std::list<std::vector<double>> valid_paths_data_sets = {valid_paths[0], valid_paths[1], valid_paths[2], valid_paths[3], valid_paths[4], valid_paths[5], valid_paths[6], valid_paths[7]};
    std::list<std::vector<double>> run_times_data_sets = {run_times[0], run_times[1], run_times[2], run_times[3], run_times[4], run_times[5], run_times[6], run_times[7]};
    std::list<std::vector<double>> path_lengths_data_sets = {path_lengths[0], path_lengths[1], path_lengths[2], path_lengths[3], path_lengths[4], path_lengths[5], path_lengths[6], path_lengths[7]};

    // Prepare labels for the box plot
    std::vector<std::string> labels = {"n=200, r=1.5", "n=200, r=1", "n=200, r=1.5", "n=200, r=2", "n=500, r=0.5", "n=500, r=1", "n=500, r=1.5", "n=500, r=2"};

    // Call makeBoxPlot with the prepared data sets and labels
    Visualizer::makeBoxPlot(valid_paths_data_sets, labels, "PRM Valid Paths", "Parameter Combinations", "Valid Paths");
    Visualizer::makeBoxPlot(run_times_data_sets, labels, "PRM Run Times", "Parameter Combinations", "Run Times");
    Visualizer::makeBoxPlot(path_lengths_data_sets, labels, "PRM Path Lengths", "Parameter Combinations", "Path Lengths");
    Visualizer::showFigures();

    //makeBoxPlot(const std::list<std::vector<double>>& data_sets, const std::vector<std::string>& labels, const std::string& title = std::string(), const std::string& xlabel = std::string(), const std::string& ylabel = std::string());

    return 0;
}
// int main(int argc, char** argv) {
//     //HW7::hint(); // Consider implementing an N-dimensional planner 

//     // // Example of creating a graph and adding nodes for visualization
//     // std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
//     // std::map<amp::Node, Eigen::Vector2d> nodes;
    
//     // std::vector<Eigen::Vector2d> points = {{3, 3}, {4, 5}, {5, 3}, {6, 5}, {5, 7}, {7, 3}}; // Points to add to the graph
//     // for (amp::Node i = 0; i < points.size(); ++i) nodes[i] = points[i]; // Add point-index pair to the map
//     // std::vector<std::tuple<amp::Node, amp::Node, double>> edges = {{0, 4, 1}, {0, 5, 1}, {4, 5, 1}, {1, 2, 1}, {1, 3, 1}, {2, 3, 1}}; // Edges to connect
//     // for (const auto& [from, to, weight] : edges) graphPtr->connect(from, to, weight); // Connect the edges in the graph
//     //graphPtr->print();

//     // Test PRM on Workspace1 of HW2
//     Problem2D problem = HW5::getWorkspace1();
//     //Problem2D problem = HW2::getWorkspace1();
//     //Problem2D problem = HW2::getWorkspace2();
//     MyPRM prm;
//     Path2D path = prm.plan(problem);
//     std::shared_ptr<Graph<double>> graphPtr = prm.get_graphPtr();
//     std::map<Node, Eigen::Vector2d> nodes = prm.get_nodes();
//     Visualizer::makeFigure(problem, path, *graphPtr, nodes);
//     //Visualizer::makeFigure(problem, path, *graphPtr, nodes);

//     // Generate a random problem and test RRT
//     // MyRRT rrt;
//     // Path2D path;
//     // HW7::generateAndCheck(rrt, path, problem);
//     // Visualizer::makeFigure(problem, path, *graphPtr, nodes);
//     Visualizer::showFigures();

//     // Grade method
//     //HW7::grade<MyPRM, MyRRT>("firstName.lastName@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
//     return 0;
// }