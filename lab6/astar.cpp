#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <iomanip>
#include <ios>
#include <sstream>
#include <string>

#include "../helper.hpp"
#include "eyebot++.h"

int main(int argc, char const* argv[]) {
	if (argc < 2) {
		std::cerr << "astar.x nodes.txt\n";
		return EXIT_FAILURE;
	}
	std::vector<std::pair<Point, std::unordered_set<std::size_t>>> points;
	std::ifstream nodes(argv[1]);
	std::string line;
	while (std::getline(nodes, line)) {
		std::stringstream ss{line};
		Point point;
		ss >> point.x >> point.y;
		LCDPixel(point.x * 128 / 2000, (2000 - point.y) * 128 / 2000, GREEN);
		points.push_back({point, {}});
		std::size_t neighbour;
		while (ss >> neighbour) {
			points.back().second.insert(neighbour);
		}
	}
	Point const src = points.front().first;
	Point const dest = points.back().first;

	LCDMenu("SET", "START", "DRIVE", "END");
	KEYWait(KEY1);
	SIMSetRobot(0, src.x, src.y, 0, INITIAL_ANGLE);
	VWSetPosition(src.x, src.y, INITIAL_ANGLE);

	Graph graph;
	for (auto&& [point, indices] : points) {
		std::unordered_map<Point, double, HashPoint> neighbours;
		for (auto&& i : indices) {
			Point const& neighbour = points[i - 1].first;
			neighbours.emplace(
			   neighbour,
			   euclidean_distance(neighbour.x - point.x, neighbour.y - point.y));
		}
		graph[point] = {neighbours,
		                euclidean_distance(dest.x - point.x, dest.y - point.y)};
	}
	for (auto&& [point1, _] : points) {
		for (auto&& [point2, _] : points) {
			if (point1 == point2) {
				std::cout << std::fixed << std::setprecision(1) << 0.0 << '\t';
			} else if (graph[point1].first.contains(point2)) {
				std::cout << std::fixed << std::setprecision(1)
				          << graph[point1].first[point2] << '\t';
			} else {
				std::cout << std::fixed << std::setprecision(1) << -1.0 << '\t';
			}
			std::cout << ' ';
		}
		std::cout << std::endl;
	}

	KEYWait(KEY2);
	auto const& [distance, src_to_dest] = a_star(graph, src, dest);
	if (src_to_dest.empty()) {
		std::cerr << "No path found to goal\n";
		return EXIT_FAILURE;
	}
	std::cout << "Shortest distance: " << distance << "\n";

	KEYWait(KEY3);
	for (std::size_t i = 1; i < src_to_dest.size(); ++i) {
		LCDLine(src_to_dest[i - 1].x * 128 / 2000,
		        (2000 - src_to_dest[i - 1].y) * 128 / 2000,
		        src_to_dest[i].x * 128 / 2000,
		        (2000 - src_to_dest[i].y) * 128 / 2000,
		        BLUE);
		drive_waypoint(src_to_dest[i].x, src_to_dest[i].y);
	}

	KEYWait(KEY4);
	return EXIT_SUCCESS;
}
