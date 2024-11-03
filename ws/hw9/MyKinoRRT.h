#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;


    private:
        // double r, goal_bias, max_itr, epsilon;
        // max_itr = 1000;
        double epsilon = 0.5;
        double goal_bias = 0.1;
        double max_itr = 50;
        double r = 0.5;
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
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
};