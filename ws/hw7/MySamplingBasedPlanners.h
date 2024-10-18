#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"
//#include "hw/HW6.h"

#include "MyAStar.h"

class MyPRM : public amp::PRM2D {
    public:
        // Add default constructor
        MyPRM() : n(1500), r(1.5) {}

        // Constructor to set values of n and r
        MyPRM(int n, double r) : n(n), r(r) {}

        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 

        std::shared_ptr<amp::Graph<double>> get_graphPtr()  { return graphPtr;}
        std::map<amp::Node, Eigen::Vector2d> get_nodes()  { return nodes;}
        // const std::shared_ptr<std::map<amp::Node, Eigen::Vector2d>> get_nodes_ptr() const { return nodes_ptr;}


    private:
        int n;
        double r;
        std::shared_ptr<amp::Graph<double>> graphPtr;
        // std::shared_ptr<std::map<amp::Node, Eigen::Vector2d>> nodes_ptr;
        std::map<amp::Node, Eigen::Vector2d> nodes;
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 

        std::shared_ptr<amp::Graph<double>> get_graphPtr() const {
            return graphPtr;
        }

        const std::map<amp::Node, Eigen::Vector2d> get_nodes() const {
            return nodes;
        }

        void set_n(double n) {
            this->n = n;
        }

        void set_r(double r) {
            this->r = r;
        }

        void set_goal_bias(double goal_bias) {
            this->goal_bias = goal_bias;
        }

        void set_epsilon(double epsilon) {
            this->epsilon = epsilon;
        }

    private:
        double n = 5000, r = 0.1, goal_bias = 0.05, epsilon = 0.25;
        std::shared_ptr<amp::Graph<double>> graphPtr;
        std::map<amp::Node, Eigen::Vector2d> nodes;
};

bool point_in_polygons(Eigen::Vector2d point, const amp::Problem2D& problem); 
bool line_segment_in_polygon(const std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step);
std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> get_all_primitives(const amp::Environment2D& env);



struct LookupSearchHeuristic : public amp::SearchHeuristic {
	/// @brief Get the heuristic value stored in `heuristic_values`. 
	/// @param node Node to get the heuristic value h(node) for. 
	/// @return Heuristic value
	virtual double operator()(amp::Node node) const override {
        if (heuristic_values.find(node) == heuristic_values.end()) {
            return 0; // Return infinity if the node is not found
        } else {
            return heuristic_values.at(node);
        }
    }

    /// @brief Store the heursitic values for each node in a map
    std::map<amp::Node, double> heuristic_values; 
};