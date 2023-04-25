#include <cmath>
#include <utility>
#include <vector>

#include "../helper.hpp"
#include "../image.hpp"
#include "eyebot++.h"

constexpr int INITIAL_X = 500;
constexpr int INITIAL_Y = 3500;
constexpr int SRC = 16;
constexpr int DEST = IMAGE_SIZE - SRC;
constexpr int MAXIMUM_RESOLUTION_LEVEL = 4;

struct Square {
	int x;
	int y;
	int size;
	Square(int x, int y, int size)
	: x{x}
	, y{y}
	, size{size} {}
};

void quad_tree(Square area,
               BYTE* image,
               std::vector<Square>& free_squares,
               std::vector<Square>& occupied_squares) {
	bool all_free = true;
	bool all_occupied = true;
	for (int i = area.x; i < area.x + area.size; ++i) {
		for (int j = area.y; j < area.y + area.size; ++j) {
			if (image[i * IMAGE_SIZE + j])
				all_free = false;
			else
				all_occupied = false;
		}
	}
	if (all_free) {
		printf("free %d %d (%d)\n", area.x, area.y, area.size);
		free_squares.emplace_back(area.x, area.y, area.size);
		LCDArea(area.y + 1,
		        area.x + 1,
		        area.y + area.size - 1,
		        area.x + area.size - 1,
		        GREEN,
		        0);
	} else if (all_occupied or area.size <= MAXIMUM_RESOLUTION_LEVEL) {
		occupied_squares.emplace_back(area.x, area.y, area.size);
		LCDArea(area.y + 1,
		        area.x + 1,
		        area.y + area.size - 1,
		        area.x + area.size - 1,
		        RED,
		        0);
	} else if (area.size > 1) {
		int half = area.size / 2;
		quad_tree({area.x, area.y, half}, image, free_squares, occupied_squares);
		quad_tree({area.x + half, area.y, half},
		          image,
		          free_squares,
		          occupied_squares);
		quad_tree({area.x, area.y + half, half},
		          image,
		          free_squares,
		          occupied_squares);
		quad_tree({area.x + half, area.y + half, half},
		          image,
		          free_squares,
		          occupied_squares);
	}
}

void collision_free_paths(std::vector<Square>& free_squares,
                          std::vector<Square>& occupied_squares,
                          Graph& graph) {
	for (int i = 0; i < free_squares.size(); ++i) {
		for (int j = i + 1; j < free_squares.size(); ++j) {
			bool over_occupied_squares = false;
			int Ax = free_squares[i].x + free_squares[i].size / 2;
			int Ay = free_squares[i].y + free_squares[i].size / 2;
			int Bx = free_squares[j].x + free_squares[j].size / 2;
			int By = free_squares[j].y + free_squares[j].size / 2;
			for (int k = 0; k < occupied_squares.size(); ++k) {
				auto const line_equation = [Ax, Ay, Bx, By](int Px, int Py) {
					return (By - Ay) * Px + (Ax - Bx) * Py + (Bx * Ay - Ax * By);
				};

				int Rx = occupied_squares[k].x;
				int Ry = occupied_squares[k].y;
				int Sx = occupied_squares[k].x + occupied_squares[k].size;
				int Sy = occupied_squares[k].y;
				int Tx = occupied_squares[k].x;
				int Ty = occupied_squares[k].y + occupied_squares[k].size;
				int Ux = occupied_squares[k].x + occupied_squares[k].size;
				int Uy = occupied_squares[k].y + occupied_squares[k].size;

				int top_left = line_equation(Rx, Ry);
				int top_right = line_equation(Sx, Sy);
				int bottom_left = line_equation(Tx, Ty);
				int bottom_right = line_equation(Ux, Uy);

				int negatives = 0;
				int positives = 0;
				if (top_left >= MAXIMUM_RESOLUTION_LEVEL)
					++positives;
				else if (top_left < -MAXIMUM_RESOLUTION_LEVEL)
					++negatives;
				if (top_right >= MAXIMUM_RESOLUTION_LEVEL)
					++positives;
				else if (top_right < -MAXIMUM_RESOLUTION_LEVEL)
					++negatives;
				if (bottom_left >= MAXIMUM_RESOLUTION_LEVEL)
					++positives;
				else if (bottom_left < -MAXIMUM_RESOLUTION_LEVEL)
					++negatives;
				if (bottom_right >= MAXIMUM_RESOLUTION_LEVEL)
					++positives;
				else if (bottom_right < -MAXIMUM_RESOLUTION_LEVEL)
					++negatives;
				if (negatives == 4 or positives == 4) continue;

				if (not((Ax - Ux >= MAXIMUM_RESOLUTION_LEVEL
				         and Bx - Ux >= MAXIMUM_RESOLUTION_LEVEL)
				        or (Rx - Ax >= MAXIMUM_RESOLUTION_LEVEL
				            and Rx - Bx >= MAXIMUM_RESOLUTION_LEVEL)
				        or (Ay - Uy >= MAXIMUM_RESOLUTION_LEVEL
				            and By - Uy >= MAXIMUM_RESOLUTION_LEVEL)
				        or (Ry - Ay >= MAXIMUM_RESOLUTION_LEVEL
				            and Ry - By >= MAXIMUM_RESOLUTION_LEVEL))) {
					over_occupied_squares = true;
					break;
				}
			}
			if (not over_occupied_squares) {
				LCDLine(Ay, Ax, By, Bx, BLUE);
				int distance = euclidean_distance(Ax - Bx, Ay - By);
				std::cout << "Distance from (" << Ax << ", " << Ay << ") -> (" << Bx
				          << ", " << By << "): " << distance << std::endl;
				graph[{Ax, Ay}].first.emplace(
				   std::pair<Point, int>({Bx, By}, distance));
				graph[{Bx, By}].first.emplace(
				   std::pair<Point, int>({Ax, Ay}, distance));
			}
		}
	}
}

int main(int argc, char const* argv[]) {
	if (argc < 2) {
		std::cerr << "quadtree.x map.pbm\n";
		return EXIT_FAILURE;
	}
	BYTE* image;
	read_pbm(argv[1], &image);
	LCDImageStart(0, 0, IMAGE_SIZE, IMAGE_SIZE);
	LCDImageBinary(image);

	std::vector<Square> free_squares;
	std::vector<Square> occupied_squares;
	Graph graph;

	SIMSetRobot(0, INITIAL_X, INITIAL_Y, 0, INITIAL_ANGLE);
	LCDMenu("QUADTREE", "PATHS", "DRIVE", "END");
	while (true) {
		switch (KEYRead()) {
		case KEY1:
			quad_tree({0, 0, IMAGE_SIZE}, image, free_squares, occupied_squares);
			break;
		case KEY2:
			collision_free_paths(free_squares, occupied_squares, graph);
			break;
		case KEY3: {
			for (auto&& [point, neighbours] : graph) {
				neighbours.second =
				   euclidean_distance(DEST - point.x, DEST - point.y);
			}
			std::vector<Point> src_to_dest =
			   a_star(graph, {SRC, SRC}, {DEST, DEST}).second;
			for (std::size_t i = 1; i < src_to_dest.size(); ++i) {
				auto const& curr_point = src_to_dest[i - 1];
				auto const& next_point = src_to_dest[i];
				LCDLine(curr_point.y, curr_point.x, next_point.y, next_point.x, RED);
			}

			std::vector<Point> path;
			std::transform (
			   src_to_dest.cbegin(),
			   src_to_dest.cend(),
			   std::back_inserter(path),
			   [](Point const& point) {
				   return Point{point.y * WORLD_SIZE / IMAGE_SIZE,
				                (IMAGE_SIZE - point.x) * WORLD_SIZE / IMAGE_SIZE};
			   });

			VWSetPosition(path.front().x, path.front().y, 0);
			for (std::size_t i = 1; i < path.size(); ++i) {
				printf("(%d, %d)\n", path[i].x, path[i].y);
				drive_waypoint(path[i].x, path[i].y);
			}
			break;
		}
		case KEY4: return EXIT_SUCCESS;
		default: break;
		}
	}
}
