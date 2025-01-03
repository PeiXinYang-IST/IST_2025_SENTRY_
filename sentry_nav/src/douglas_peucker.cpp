//道格拉斯-普克算法对全局路径进行降采样，获取目标点按顺序作为fast-planner目标点
#include <iostream>
#include <vector>
#include <cmath>

struct Point {
    double x, y;
    Point(double x, double y) : x(x), y(y) {}
};

double perpendicularDistance(Point point, Point start, Point end) {
    if (start.x == end.x && start.y == end.y) {
        return sqrt(pow(point.x - start.x, 2) + pow(point.y - start.y, 2));
    }
 
    double num = fabs((end.y - start.y) * point.x - (end.x - start.x) * point.y + end.x * start.y - end.y * start.x);
    double denom = sqrt(pow(end.y - start.y, 2) + pow(end.x - start.x, 2));
    return num / denom;
}
 
std::vector<Point> douglasPeucker(std::vector<Point> points, double epsilon) {
    if (points.size() < 2) return points;
 
    Point start = points.front();
    Point end = points.back();
 
    double maxDistance = 0.0;
    int index = 0;
 
    for (int i = 1; i < points.size() - 1; i++) {
        double distance = perpendicularDistance(points[i], start, end);
        if (distance > maxDistance) {
            index = i;
            maxDistance = distance;
        }
    }
 
    std::vector<Point> result;
    if (maxDistance > epsilon) {
        std::vector<Point> left = douglasPeucker(std::vector<Point>(points.begin(), points.begin() + index + 1), epsilon);
        std::vector<Point> right = douglasPeucker(std::vector<Point>(points.begin() + index, points.end()), epsilon);
        result.insert(result.end(), left.begin(), left.end() - 1); // Remove last point of left
        result.insert(result.end(), right.begin(), right.end());
    } else {
        result.push_back(start);
        result.push_back(end);
    }
 
    return result;
}
 
int main() {
    std::vector<Point> points = {Point(0, 0), Point(1, 1), Point(2, 0), Point(3, 1), Point(4, 0)};
    double epsilon = 0.5;
    std::vector<Point> simplifiedPoints = douglasPeucker(points, epsilon);
    return 0;
}