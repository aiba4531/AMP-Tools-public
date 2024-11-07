#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        // Default Constructor
        MyKinoRRT() : goal_bias(0.05), max_itr(10000), dt(0.5) {}

        // Constructor with parameters
        MyKinoRRT(double goal_bias, double max_itr, double dt) :  goal_bias(goal_bias), max_itr(max_itr), dt(dt) {}

        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;


    private:
        // double r, goal_bias, max_itr, epsilon;
        // max_itr = 1000;
        double goal_bias = 0.05;
        double max_itr = 10000;
        double dt = 0.5;
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
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};