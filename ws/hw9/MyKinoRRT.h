#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        // Default Constructor
        MyKinoRRT() : goal_bias(0.05), max_itr(10000), dt(0.5), max_sampled_controls(5.0) {}

        // Constructor with parameters
        MyKinoRRT(double goal_bias, double max_itr, double dt, double max_sampled_controls) :  goal_bias(goal_bias), max_itr(max_itr), dt(dt), max_sampled_controls(max_sampled_controls){}

        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;

        // Get the path length
        double get_path_length() { return path_length; }


    private:
        // double r, goal_bias, max_itr, epsilon;
        // max_itr = 1000;
        double goal_bias = 0.05;
        double max_itr = 10000;
        double dt = 0.5;
        double max_sampled_controls = 5;
        double path_length = 0.0;
        std::map<amp::Node, Eigen::VectorXd> potential_states;
        std::map<amp::Node, Eigen::VectorXd> potential_controls;
        std::map<amp::Node, double> potential_duration;
        std::map<amp::Node, amp::Node> parent_map;
};  

class MySingleIntegrator : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MyFirstOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
        Eigen::VectorXd state_derivative(const Eigen::VectorXd& state, const Eigen::VectorXd& control, double r);

};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
        void propagate_car(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt, const amp::KinodynamicProblem2D& problem);
        double get_length(const amp::KinodynamicProblem2D& problem);
};