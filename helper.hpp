#ifndef HELPER_H
#define HELPER_H

#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <iostream>
#include <queue>
#include <utility>

#include "eyebot++.h"
#include "image.hpp"

constexpr int WORLD_SIZE = 4000;
constexpr int EPSILON = 2;
constexpr int DRIVING_SPEED = 300;
constexpr int ADJUSTING_SPEED = 100;
constexpr int ADJUSTING_ANGLE = 10;
constexpr int INITIAL_ANGLE = 0;
constexpr int SAFE_DISTANCE = 300;
constexpr int ABSOLUTE_SAFE_DISTANCE = 150;
constexpr int SENSOR_INF_DIST = 9999;
constexpr int STEP = 1000;
constexpr int COMPLETE_ANGLE = 360;
constexpr int STRAIGHT_ANGLE = COMPLETE_ANGLE / 2;
constexpr int RIGHT_ANGLE = STRAIGHT_ANGLE / 2;
constexpr std::size_t LIDAR_FRONT = STRAIGHT_ANGLE;
constexpr std::size_t LIDAR_FRONT_RIGHT = 210;
constexpr std::size_t LIDAR_FRONT_LEFT = 150;
constexpr std::size_t LIDAR_RIGHT = 270;
constexpr std::size_t LIDAR_LESS_RIGHT = 265;
constexpr std::size_t LIDAR_MORE_RIGHT = 275;

/**
 * @brief Position in the real map
 */
struct Position {
	int x;
	int y;
	friend auto operator==(Position const& lhs, Position const& rhs) -> bool {
		constexpr int DEVIATION = 100;
		return std::abs(lhs.x - rhs.x) < DEVIATION
		       and std::abs(lhs.y - rhs.y) < DEVIATION;
	}
	friend auto operator!=(Position const& lhs, Position const& rhs) -> bool {
		return not(lhs == rhs);
	}
	friend auto operator<<(std::ostream& os, Position const& e) -> std::ostream& {
		return os << '(' << e.x << ", " << e.y << ")\n";
	}
};

/**
 * @brief Point in a graph representation of the real environment
 * or in an image for visualisation purposes
 */
struct Point {
	int x;
	int y;
	Point(): x{0}, y{0} {}
	Point(int x, int y)
	: x{x}
	, y{y} {}
	friend auto operator<(Point const& lhs, Point const& rhs) -> bool {
		return lhs.x == rhs.x ? lhs.y < rhs.y : lhs.x < rhs.x;
	}
	friend auto operator==(Point const& lhs, Point const& rhs) -> bool = default;
	friend auto operator<<(std::ostream& os, Point const& point) -> std::ostream& {
		return os << '(' << point.x << ", " << point.y << ")\n";
	}
};

struct HashPoint {
	size_t operator()(Point const& p) const {
		auto hash1 = std::hash<int>{}(p.x);
		auto hash2 = std::hash<int>{}(p.y);
		return hash1 != hash2 ? hash1 ^ hash2 : hash1;
	}
};

using Graph =
   std::unordered_map<Point,
                      std::pair<std::unordered_map<Point, double, HashPoint>, double>,
                      HashPoint>;

using Path = std::pair<int, std::vector<Point>>;

struct Compare {
	using is_transparent = void;
	auto operator()(Path const& a, Path const& b) const -> bool {
		return a.first > b.first;
	}
};

double euclidean_distance(int dx, int dy) {
	return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}

Path a_star(Graph& graph, Point const& src, Point const& dest) {
	std::priority_queue<Path, std::vector<Path>, Compare> next;
	std::unordered_set<Point, HashPoint> visited{src};
	next.emplace(Path{graph.at(src).second, {src}});
	while (not next.empty()) {
		auto const [dist_so_far, path] = next.top();
		auto const& curr = path.back();
		visited.insert(curr);
		next.pop();
		for (auto&& [neighbour, relative_dist] : graph.at(curr).first) {
			if (not visited.contains(neighbour)) {
				auto new_path = path;
				new_path.push_back(neighbour);
				next.emplace(Path{dist_so_far - graph.at(curr).second
				                     + relative_dist + graph.at(neighbour).second,
				                  new_path});
			}
		}
		if (next.top().second.back() == dest) return next.top();
	}
	// Goal not reachable
	return {};
}

void drive_waypoint(float wx, float wy) {
	int x, y, phi;
	VWGetPosition(&x, &y, &phi);
	int dx = wx - x;
	int dy = wy - y;
	int theta = std::atan2(dy, dx) * STRAIGHT_ANGLE / M_PI;
	VWTurn(theta - phi, ADJUSTING_SPEED);
	VWWait();
	VWStraight(euclidean_distance(dx, dy), DRIVING_SPEED);
	VWWait();
}

#define STOP VWSetSpeed(0, 0)
#define OS_WAIT OSWait(100)

#endif // HELPER_H
