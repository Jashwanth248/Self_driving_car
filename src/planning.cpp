#include "autodrive/planning.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
namespace autodrive {
namespace {
double distance(Point a, Point b) { return std::hypot(a.x - b.x, a.y - b.y); }
}
std::vector<Point> AStar::solve(const std::vector<std::string>& grid, Point start, Point goal) {
    if (grid.empty() || grid.front().empty()) throw std::invalid_argument("empty grid");
    const int height = static_cast<int>(grid.size());
    const int width = static_cast<int>(grid.front().size());
    for (const auto& row : grid) if (static_cast<int>(row.size()) != width)
        throw std::invalid_argument("grid must be rectangular");
    auto valid = [&](int x, int y) { return x >= 0 && y >= 0 && x < width && y < height && grid[y][x] != '#'; };
    const int sx = static_cast<int>(start.x), sy = static_cast<int>(start.y);
    const int gx = static_cast<int>(goal.x), gy = static_cast<int>(goal.y);
    if (!valid(sx,sy) || !valid(gx,gy)) throw std::invalid_argument("invalid start or goal");
    auto id = [&](int x,int y) { return y * width + x; };
    const int s = id(sx,sy), g = id(gx,gy);
    std::vector<int> parent(width * height,-1), cost(width * height,std::numeric_limits<int>::max());
    using Item = std::pair<int,int>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    cost[s] = 0; queue.push({std::abs(sx-gx)+std::abs(sy-gy),s});
    while (!queue.empty()) {
        const auto [score,u] = queue.top(); queue.pop();
        const int x = u%width, y = u/width;
        if (score != cost[u]+std::abs(x-gx)+std::abs(y-gy)) continue;
        if (u == g) break;
        for (const auto& [dx,dy] : {std::pair<int,int>{1,0},{-1,0},{0,1},{0,-1}}) {
            const int nx=x+dx, ny=y+dy;
            if (!valid(nx,ny)) continue;
            const int v=id(nx,ny);
            if (cost[u]+1 < cost[v]) {
                parent[v]=u;cost[v]=cost[u]+1;
                queue.push({cost[v]+std::abs(nx-gx)+std::abs(ny-gy),v});
            }
        }
    }
    if (cost[g]==std::numeric_limits<int>::max()) return {};
    std::vector<Point> reversed;
    for (int at=g;at!=-1;at=parent[at]) reversed.push_back({double(at%width),double(at/width)});
    std::reverse(reversed.begin(),reversed.end()); return reversed;
}
Route::Route(std::vector<Point> points):points_(std::move(points)) {
    if (points_.size()<2) throw std::invalid_argument("route needs at least two waypoints");
    distances_.push_back(0);
    for (size_t i=1;i<points_.size();++i) {
        const double d=distance(points_[i],points_[i-1]);
        if (d<=0) throw std::invalid_argument("duplicate route waypoints");
        length_+=d; distances_.push_back(length_);
    }
}
double Route::progress(Point p) const {
    double best_dist2=std::numeric_limits<double>::max(), best_progress=0;
    for (size_t i=1;i<points_.size();++i) {
        const Point a=points_[i-1], b=points_[i];
        const double dx=b.x-a.x, dy=b.y-a.y;
        const double t=std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy),0.0,1.0);
        const double px=a.x+t*dx,py=a.y+t*dy;
        const double d2=(p.x-px)*(p.x-px)+(p.y-py)*(p.y-py);
        if (d2<best_dist2) {best_dist2=d2;best_progress=distances_[i-1]+t*(distances_[i]-distances_[i-1]);}
    }
    return best_progress;
}
Point Route::lookahead(double progress_m,double ahead) const {
    const double target=std::clamp(progress_m+ahead,0.0,length_);
    for (size_t i=1;i<points_.size();++i) {
        if (target<=distances_[i]) {
            const double t=(target-distances_[i-1])/(distances_[i]-distances_[i-1]);
            return {points_[i-1].x+t*(points_[i].x-points_[i-1].x),points_[i-1].y+t*(points_[i].y-points_[i-1].y)};
        }
    }
    return points_.back();
}
Route default_route(double scale) {
    auto grid_path=AStar::solve({"...........","..###......","...........","......###..","..........."},{0,0},{10,4});
    // Gentle corner rounding. Keep endpoints fixed, and iteratively smooth the sharp right angle.
    std::vector<Point> pts;
    for (auto p:grid_path)pts.push_back({p.x*scale,p.y*scale});
    for (int pass=0;pass<30;++pass) {
        auto old=pts;
        for(size_t i=1;i+1<pts.size();++i) {
            pts[i].x=0.25*old[i-1].x+0.5*old[i].x+0.25*old[i+1].x;
            pts[i].y=0.25*old[i-1].y+0.5*old[i].y+0.25*old[i+1].y;
        }
    }
    return Route(std::move(pts));
}
}
