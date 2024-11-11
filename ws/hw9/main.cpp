// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"

using namespace amp;

// Load problems and map agent for quick testing
std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
    {AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
    {AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
};

// int main(int argc, char** argv) {
//     // Select problem, plan, check, and visualize
//     int select = 7;
//     KinodynamicProblem2D prob = problems[select];
//     amp::RNG::seed(amp::RNG::randiUnbounded());

//     // Create a planner and plan
//     double goal_bias = 0.05;
//     double max_itr = 1000000;
//     double dt = 0.3;
//     double max_sampled_controls = 10.0;
    
//     MyKinoRRT kino_planner(goal_bias, max_itr, dt, max_sampled_controls);

//     // // Plan the path
//     // KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
//     // HW9::check(path, prob); 

//     // if (path.valid)
//     //    Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation
//     // Visualizer::showFigures();
//     HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("aidan.bagley@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
//     return 0;
// }

int main(int argc, char** argv) {

    // Benchmark valid solutions, path lengths, and computation times for 50 runs and plot in box plots
    std::vector<double> valid_solutions(4,0);
    std::vector<std::vector<double>> path_lengths(4);
    std::vector<std::vector<double>> computation_times(4);

    std::vector<double> num_samples = {1.0, 5.0, 10.0, 15.0};
    Timer timer("timer"); // Create a timer
    double start_time, elapsed_time;    


    for (int p = 0 ; p < 4; p++){
            
        // Run the benchmark for 50 runs
        for (int i = 0; i < 50; i++) {
            // Select problem, plan, check, and visualize
            int select = 4;
            KinodynamicProblem2D prob = problems[select];
            amp::RNG::seed(amp::RNG::randiUnbounded());

            // Create a planner and plan
            double goal_bias = 0.05;
            double max_itr = 50000;
            double dt = 0.20;
            double max_sampled_controls = num_samples[p];

            // Create a planner and plan
            MyKinoRRT kino_planner(goal_bias, max_itr, dt, max_sampled_controls);
            
            // Start the timer
            start_time = timer.now(TimeUnit::ms); 
            // Plan the path
            KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
            
            // Stop the timer
            elapsed_time = timer.now(TimeUnit::ms) - start_time;
            
            // Check if the path is valid
            if (path.waypoints.size() > 0) {
                // Store the valid solutions, path lengths, and computation times
                valid_solutions[p] += 1;
                path_lengths[p].push_back(kino_planner.get_path_length());
                computation_times[p].push_back(elapsed_time);
            }
            std::cout << "Finished run " << i << " of 50 for n=" << num_samples[p] << std::endl;
        }
    }

    // Plot the box plots
    std::list<std::vector<double>> run_times_data_sets = {computation_times[0], computation_times[1], computation_times[2], computation_times[3]};
    std::list<std::vector<double>> path_lengths_data_sets = {path_lengths[0], path_lengths[1], path_lengths[2], path_lengths[3]};

    // Prepare labels for the box plot
    std::vector<std::string> labels = { "n=50000, u_samples = 1", "n=50000, u_samples = 5", "n=50000, u_samples = 10", "n=50000, u_samples = 15"};

    // Call makeBoxPlot with the prepared data sets and labels
    Visualizer::makeBoxPlot(run_times_data_sets, labels, "Kino RRT Compuational Times for 50 runs", "Parameter Combinations", "Computation Time [ms]");
    Visualizer::makeBoxPlot(path_lengths_data_sets, labels, "Kino RRT Path Lengths for 50 runs", "Parameter Combinations", "Path Length");
    Visualizer::makeBarGraph(valid_solutions, labels, "Kino RRT Valid Paths for 50 runs", "Parameter Combinations", "Valid Paths");

    Visualizer::showFigures();


    return 0;
}