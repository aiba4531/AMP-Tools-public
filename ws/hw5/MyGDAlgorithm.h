#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

class MyGDAlgorithm : public amp::GDAlgorithm {
	public:
		// Consider defining a class constructor to easily tune parameters, for example: 
		MyGDAlgorithm(double d_star, double zetta, double Q_star, double eta) :
			d_star(d_star), // threshold distance from goal to not overstep
			zetta(zetta), // attraction strength
			Q_star(Q_star), // threshold obstacle value to ignore far away obstacles
			eta(eta) {} // repulsion strength

		// Override this method to solve a given problem.
		virtual amp::Path2D plan(const amp::Problem2D& problem) override;
		std::tuple<double, Eigen::Vector2d> get_closest_distance_to_obstacle(const auto& obs, const Eigen::Vector2d& q);
		bool next_step_collision(const amp::Problem2D& env, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step);
		Eigen::Vector2d take_random_step(const amp::Problem2D& env, const Eigen::Vector2d& new_point);


	private:
		double d_star, zetta, Q_star, eta;
		// Add additional member variables here...
};

class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
		// Make a constructor that inputs the required parameters
        MyPotentialFunction(const amp::Problem2D& prob, double d_star, double zetta, double Q_star, double eta) : 
			problem(prob),
			d_star(d_star),
			zetta(zetta),
			Q_star(Q_star),
			eta(eta) {}

		// Get the potential function value at a given point
		virtual double operator()(const Eigen::Vector2d& q) const override {
			
			// Define the attraction potential
			double att_potential = 0;

			// Current distance from goal
			double distance = (q - problem.q_goal).norm();

			// If I am close to goal, this will be quadratic attractive force
			if (distance <= d_star) { 
				att_potential = 0.5*zetta*distance*distance;
			}
			else { //Otherwise, I am far from goal, I want a smaller attractive force
				att_potential = d_star*zetta*distance - 0.5*zetta*d_star*d_star;
			}

			// Repulsive Force
			double rep_potential = 0;

			// Create an instance of MyGDAlgorithm
			MyGDAlgorithm algo(d_star, zetta, Q_star, eta);

			// Get the closest distance to every obstacle
			std::tuple<double, Eigen::Vector2d> obs_closest_point_tuple;

			for (const auto& obs : problem.obstacles) {
				obs_closest_point_tuple = algo.get_closest_distance_to_obstacle(obs, q);
				double obs_distance = std::get<0>(obs_closest_point_tuple);
				Eigen::Vector2d closest_point = std::get<1>(obs_closest_point_tuple);

				// if current point is close to obstacle 
				if (obs_distance <= Q_star) { // large repulsion
					rep_potential += 0.5*eta* (1/obs_distance - 1/Q_star)*(1/obs_distance - 1/Q_star);
				}
				else { // if current point is far from obstacle
					rep_potential += 0;
				}
			}
			// The total potential is the attraction potential plus the repulsion potential
			double potential = att_potential + rep_potential;

			return potential;
		}

		virtual Eigen::Vector2d gradient(const Eigen::Vector2d& q)  {

			// Initialize attractive gradient
			Eigen::Vector2d att_grad;

			// Initialize gradient to zero
			att_grad.setZero();

			// Distance from goal
			double distance = (q - problem.q_goal).norm();
			
			// If I am close to goal, linear gradient
			if (distance <= d_star) {
				att_grad = zetta*(q - problem.q_goal);
			}
			else { // If I am from goal, I want a smaller attractive gradient
				att_grad = d_star*zetta*(q - problem.q_goal)/distance;
			}

			// Repulsive Gradient
			Eigen::Vector2d rep_grad;
			rep_grad.setZero();

			// Create an instance of MyGDAlgorithm
			MyGDAlgorithm algo(d_star, zetta, Q_star, eta);

			// Get the closest distance to every obstacle
			std::tuple<double, Eigen::Vector2d> obs_closest_point_tuple;

			for (const auto& obs : problem.obstacles) {
				obs_closest_point_tuple = algo.get_closest_distance_to_obstacle(obs, q);

				double obs_distance = std::get<0>(obs_closest_point_tuple);
				Eigen::Vector2d closest_point = std::get<1>(obs_closest_point_tuple);

				if (obs_distance <= Q_star) { // if I am close to obstacle
					Eigen::Vector2d gradient = (q - closest_point)/obs_distance;
					rep_grad += eta*(1/Q_star - 1/obs_distance)* gradient/(obs_distance*obs_distance);
				}
			}

			return att_grad + rep_grad;
		}
		
	private:
		const amp::Problem2D& problem;
		double d_star, zetta, Q_star, eta;


};