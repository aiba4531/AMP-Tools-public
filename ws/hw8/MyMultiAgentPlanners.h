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
        MyCentralPlanner() : r(1.0), goal_bias(0.05), max_itr(1000), epsilon(1.0) {}

        // Constructor with parameters
        MyCentralPlanner(double r, double goal_bias, double max_itr, double epsilon) : r(r), goal_bias(goal_bias), max_itr(max_itr), epsilon(epsilon) {}

        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        // Geter methods
        // std::shared_ptr<amp::Graph<double>> get_graphPtr()  { return graphPtr;}
        // std::map<amp::Node, Eigen::Vector2d> get_nodes()  { return nodes;}

    private:
        double r, goal_bias, max_itr, epsilon;
        //std::shared_ptr<amp::Graph<double>> graphPtr;
        //std::map<amp::Node, std::vector<Eigen::Vector2d>> nodes;
        //std::vector<std::map<amp::Node, amp::Node>> parent_map;

        

};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
};