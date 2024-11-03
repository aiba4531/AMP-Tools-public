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
        MyCentralPlanner() : r(0.5), goal_bias(0.05), max_itr(30000), epsilon(0.25) {}

        // Constructor with parameters
        MyCentralPlanner(double r, double goal_bias, double max_itr, double epsilon) : r(r), goal_bias(goal_bias), max_itr(max_itr), epsilon(epsilon) {}

        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        // Getter for nodes, parents, and succeess
        const std::map<amp::Node, Eigen::VectorXd>& getNodes() const { return nodes; }
        const std::map<amp::Node, amp::Node>& getParentMap() const { return parent_map; }
        bool isSuccess() const { return success; }

    private:
        double r, goal_bias, max_itr, epsilon;
        std::map<amp::Node, Eigen::VectorXd> nodes; 
        std::map<amp::Node, amp::Node> parent_map;
        bool success = false;
       
        

};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
    public:
        
        // Default constructor
        MyDecentralPlanner() : r(0.50), goal_bias(0.05), max_itr(12500), epsilon(0.25) {}

        // Constructor with parameters
        MyDecentralPlanner(double r, double goal_bias, double max_itr, double epsilon) : r(r), goal_bias(goal_bias), max_itr(max_itr), epsilon(epsilon) {}

        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        // Getter for nodes, parents, and time
        const std::vector<std::map<amp::Node, Eigen::Vector2d>>& getNodes() const { return nodes; }
        const std::vector<std::map<amp::Node, amp::Node>>& getParentMap() const { return parent_map; }
        const std::vector<std::map<amp::Node, int>>& getTimeMap() const { return time_map; } // Getter for time map

    private:
        double r, goal_bias, max_itr, epsilon;
        std::vector<std::map<amp::Node, Eigen::Vector2d>> nodes;
        std::vector<std::map<amp::Node, amp::Node>> parent_map;
        std::vector<std::map<amp::Node, int>> time_map;  // Store the time for each node
};