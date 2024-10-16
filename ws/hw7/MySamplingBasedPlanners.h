#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"
#include "hw/HW6.h"


#include "MyAStar.h"

class MyPRM : public amp::PRM2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 

        std::shared_ptr<amp::Graph<double>> get_graphPtr() const {
            return graphPtr;
        }
        const std::map<amp::Node, Eigen::Vector2d> get_nodes() const {
            return nodes;
        }

    private:
        int n = 1500;
        int r = 1.5; // Number of samples and radius of the circle

        std::shared_ptr<amp::Graph<double>> graphPtr;
        std::map<amp::Node, Eigen::Vector2d> nodes;
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
};

bool point_in_polygons(Eigen::Vector2d point, const amp::Problem2D& problem); 
bool line_segment_in_polygon(const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step);
std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> get_all_primitives(const amp::Environment2D& env);

