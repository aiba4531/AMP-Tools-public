// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // THIS WORKS FOR HW2 WS1!!!
    // // double d_star = 2;
    // // double zetta = 2.5;
    // // double Q_star = 0.25;
    // // double eta = 0.5;

    double d_star = 25;
    double zetta = 5;
    double Q_star = 0.25;
    double eta = 0.5;
    MyGDAlgorithm algo(d_star, zetta, Q_star, eta);

    // // Problem 5.1
    // amp::Problem2D prob = HW5::getWorkspace1();
    // amp::Path2D path = algo.plan(prob);

    // std::cout << "HW 5 Path length: " << path.length() << std::endl;

    // amp::Visualizer::makeFigure(prob, path);
    // amp::Visualizer::makeFigure(MyPotentialFunction{prob, d_star, zetta, Q_star, eta}, prob, 50);

    // // Problem 2.1
    // amp::Problem2D prob2 = HW2::getWorkspace1();
    // amp::Path2D path2 = algo.plan(prob2);

    // std::cout << "HW2.1 Path length: " << path2.length() << std::endl;

    // amp::Visualizer::makeFigure(prob2, path2);
    // amp::Visualizer::makeFigure(MyPotentialFunction{prob2, d_star, zetta, Q_star, eta}, prob2.x_min, prob2.x_max, prob2.y_min, prob2.y_max, 500);


    // Problem 2.2
    // amp::Problem2D prob3 = HW2::getWorkspace2();
    // amp::Path2D path3 = algo.plan(prob3);

    // std::cout << "HW2.2 Path length: " << path3.length() << std::endl;

    // amp::Visualizer::makeFigure(prob3, path3);
    // amp::Visualizer::makeFigure(MyPotentialFunction{prob3, d_star, zetta, Q_star, eta}, prob3, 50);


    
    // // Test your gradient descent algorithm on a random problem.
    // MyGDAlgorithm algo_rand(d_star, zetta, Q_star, eta);
    amp::Problem2D prob_rand;
    amp::Path2D path_rand;
    bool success = HW5::generateAndCheck(algo, path_rand, prob_rand);

    Visualizer::makeFigure(prob_rand, path_rand);
    amp::Visualizer::makeFigure(MyPotentialFunction{prob_rand, d_star, zetta, Q_star, eta}, prob_rand, 50);

    // // Visualize your potential function
    Visualizer::showFigures();
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    //HW5::grade<MyGDAlgorithm>("aidan.bagley@colorado.edu", argc, argv, d_star, zetta, Q_star, eta);
    return 0;
}