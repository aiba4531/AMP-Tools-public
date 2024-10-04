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

};