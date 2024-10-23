#include "AMPCore.h"

class MyClass {
    public:
        void hereIsAMethod();
        std::vector<Eigen::Vector2d> Minkowski(void);
        std::vector<std::vector<Eigen::Vector2d>> Minkowski_rotation(void);
        Eigen::Vector2d findLowerLeft(const std::vector<Eigen::Vector2d>& vertices);
        double polarAngle(const Eigen::Vector2d& origin, const Eigen::Vector2d& point);
        bool comparePolar(const Eigen::Vector2d& origin, const Eigen::Vector2d& a, const Eigen::Vector2d& b);
        std::vector<Eigen::Vector2d> sortVerticesCCW(std::vector<Eigen::Vector2d> vertices);
        bool robot_in_robot( const Eigen::Vector2d& robot1_center, double robot1_radius, const Eigen::Vector2d& robot2_center, double robot2_radius);
        bool robot_in_polygons(const Eigen::Vector2d& robot_center, double radius, const amp::MultiAgentProblem2D& problem);
        bool point_in_polygons(Eigen::Vector2d point, const amp::MultiAgentProblem2D& problem);
        double distance_to_segment(const Eigen::Vector2d& circle_center, const Eigen::Vector2d& edge_start, const Eigen::Vector2d& edge_end);

};