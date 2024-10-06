#include "MyGDAlgorithm.h"

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    // Create an instance of MyPotentialFunction
    MyPotentialFunction potentialFunc(problem, d_star, zetta, Q_star, eta);

    // Compute potential at the initial point
    double potential = potentialFunc(path.waypoints[0]);
    
    // Compute graidient and add some randomness to first step
    Eigen::Vector2d grad = potentialFunc.getGradient(path.waypoints[0]);

    // Run Gradient Descent    
    double itr = 0;
    double alpha = 0.005;
    double local_min_counter = 0;
    Eigen::Vector2d new_point;
    Eigen::Vector2d old_point;
    
    while ((path.waypoints.back() - problem.q_goal).norm() > 0.25 && itr < 5000) {
        // Move in the direction of the gradient
        old_point = path.waypoints.back();
        new_point = path.waypoints.back() - grad * alpha;
        path.waypoints.push_back(new_point);
        grad = potentialFunc.getGradient(path.waypoints.back());

    
        // If the potential has incrased, then we have reached a local minimum
        if (grad.norm() < 0.001) {
            // std::cout << "Potential New Point: " << potentialFunc(new_point) << std::endl;
            // std::cout << "Potential Old Point: " << potentialFunc(old_point) << std::endl;
            // std::cout << "Local Minimum Counter: " << local_min_counter << std::endl;
            local_min_counter++;
        }

        if (local_min_counter > 5) {
            new_point = take_random_step(problem, new_point);
            path.waypoints.pop_back();
            path.waypoints.push_back(new_point);
            grad = potentialFunc.getGradient(path.waypoints.back());
            local_min_counter = 0;
            //std::cout << "Random Step" << new_point.transpose() << std::endl;
        }   

        itr++;
    }
    path.waypoints.push_back(problem.q_goal);
    return path;
}

Eigen::Vector2d MyGDAlgorithm::take_random_step(const amp::Problem2D& env, const Eigen::Vector2d& new_point) {
    double random1, random2, random3;
    Eigen::Vector2d random_step;
    std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step;
    bool collision = true;

    while (collision) {
        // random1 = static_cast<double>(1.5*rand())/RAND_MAX;
        // random2 = static_cast<double>(1.5*rand())/RAND_MAX;
        random1 = static_cast<double>(1.5*rand())/RAND_MAX;
        random2 = static_cast<double>(1.5*rand())/RAND_MAX;
        random3 = static_cast<double>(rand())/RAND_MAX - 0.5;

        if (random3 <= 0.0) {
            random1 = -random1;
            random2 = 0.0;
        }
        else {
            random2 = -random2;
            random1 = 0.0;
        }

        random_step = Eigen::Vector2d(random1, random2);
        next_step = std::make_tuple(new_point, random_step);
        collision = next_step_collision(env, next_step);
    }

    return random_step;

    
}



bool MyGDAlgorithm::next_step_collision(const amp::Problem2D& env, const std::tuple<Eigen::Vector2d, Eigen::Vector2d> next_step){
     // Check for collisions with all obstacle primitives
    std::vector<std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>> all_primitives;
    for (int i = 0; i < env.obstacles.size(); i++) {
        std::vector<Eigen::Vector2d> verticies = env.obstacles[i].verticesCW();
        std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d>> primitives;
        

        for (int j = 0; j < verticies.size(); j++) {
            if (j == verticies.size() - 1) {
                primitives.push_back(std::make_tuple(verticies[j], verticies[0]-verticies[j]));
                continue;
            }
            primitives.push_back(std::make_tuple(verticies[j], verticies[j+1]-verticies[j]));
        }
        all_primitives.push_back(primitives);
    }

    std::vector<std::tuple<bool, std::tuple<int, int>>> all_intersecting_primitives;

    // Assume there is no collision
    all_intersecting_primitives.push_back(std::make_tuple(false, std::make_tuple(-1, -1)));

    std::tuple<int, int> intersecting_primitive;
    for (int i = 0; i < all_primitives.size(); i++) {
        for (int j = 0; j < all_primitives[i].size(); j++) {

            std::tuple<Eigen::Vector2d, Eigen::Vector2d> primitive = all_primitives[i][j];
            // Get intersection of two line segments
            // Get the two vectors
            Eigen::Vector2d vector1 = std::get<1>(primitive);
            Eigen::Vector2d vector2 = std::get<1>(next_step);

            // Get the two points
            Eigen::Vector2d point1 = std::get<0>(primitive);
            Eigen::Vector2d point2 = std::get<0>(next_step);

            // Get the intersection point
            Eigen::Vector2d intersection = point1 - point2;

            // Get the determinant
            double det = vector1[0]*vector2[1] - vector1[1]*vector2[0];

            // Get the intersection point
            double t = (vector2[0]*intersection[1] - vector2[1]*intersection[0]) / det;
            double u = (vector1[0]*intersection[1] - vector1[1]*intersection[0]) / det;


            // If the intersection point is between 0 and 1, then the vectors intersect
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
                return true;

            }
        }
    }
    return false;
}

std::tuple<double, Eigen::Vector2d> MyGDAlgorithm::get_closest_distance_to_obstacle(const auto& obs, const Eigen::Vector2d& q) {
    double min_distance = std::numeric_limits<double>::max();
    Eigen::Vector2d closest_point;
    int num_vertices = obs.verticesCW().size();
        for (int i = 0; i < num_vertices; i++) {
            // Current vertex
            Eigen::Vector2d v1 = obs.verticesCW()[i];
            // Next vertex (wrap around to the first vertex)
            Eigen::Vector2d v2 = obs.verticesCW()[(i + 1) % num_vertices];

            // Make a line from current vertex to next vertex
            Eigen::Vector2d obstacle_edge = v2 - v1;

            // Find the projection of the point on the obstacle_edge
            Eigen::Vector2d projection = v1 + (q - v1).dot(obstacle_edge) / obstacle_edge.squaredNorm() * obstacle_edge;

            // Clamp the projection to the obstacle_edge segment
            projection = v1 + std::max(0.0, std::min(1.0, (projection - v1).dot(obstacle_edge) / obstacle_edge.squaredNorm())) * obstacle_edge;

            // Calculate the distance from q to the projection point
            double distance = (q - projection).norm();

            // Update the minimum distance
            if (distance < min_distance) {
                closest_point = projection;
                min_distance = distance;
            }
        }
    return std::make_tuple(min_distance, closest_point);
}
