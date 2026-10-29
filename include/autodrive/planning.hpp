#pragma once
#include "autodrive/types.hpp"
#include <string>
#include <vector>
namespace autodrive {
class AStar {
public:
    static std::vector<Point> solve(const std::vector<std::string>& grid, Point start, Point goal);
};
class Route {
public:
    explicit Route(std::vector<Point> points);
    const std::vector<Point>& points() const { return points_; }
    double length() const { return length_; }
    double progress(Point location) const;
    Point lookahead(double progress_m, double lookahead_m) const;
private:
    std::vector<Point> points_;
    std::vector<double> distances_;
    double length_ = 0.0;
};
Route default_route(double cell_size_m);
}
