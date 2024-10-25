#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"

// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "MySamplingBasedPlanners.h" 


class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
    public:

        // Default constructor
        MyCentralPlanner() : r(0.5), goal_bias(0.05), max_itr(100000), epsilon(0.25) {}

        // Constructor with parameters
        MyCentralPlanner(double r, double goal_bias, double max_itr, double epsilon) : r(r), goal_bias(goal_bias), max_itr(max_itr), epsilon(epsilon) {}

        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

    private:
        double r, goal_bias, max_itr, epsilon;
       
        

};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
        
        // Default constructor
        MyDecentralPlanner() : r(0.5), goal_bias(0.05), max_itr(100000), epsilon(0.25) {}

        // Constructor with parameters
        MyDecentralPlanner(double r, double goal_bias, double max_itr, double epsilon) : r(r), goal_bias(goal_bias), max_itr(max_itr), epsilon(epsilon) {}

        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

    private:
        double r, goal_bias, max_itr, epsilon;
};