#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; // don't touch this line

        // Add any other methods here...
        std::tuple<Eigen::Vector2d, Eigen::Vector2d> getPrimitive(Eigen::Vector2d vertex1, Eigen::Vector2d vertex2);
        std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> get_all_primitives(const amp::Problem2D& problem);
        std::vector<std::tuple<bool, std::tuple<int, int>>> next_step_collision(const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step);
        bool jump_to_goal(const Eigen::Vector2d& current_point, const Eigen::Vector2d& goal, const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>>& all_primitives, amp::Path2D& path);
        bool found_mLine(std::tuple<Eigen::Vector2d, Eigen::Vector2d> mLine, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step, double prev_distance_goal, double& distance_goal);

    private:
        // Add any member variables here...
};