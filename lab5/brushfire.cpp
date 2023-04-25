#include <unordered_map>
#include <unordered_set>
#include <array>
#include <iostream>
#include <queue>
#include <set>
#include <vector>

#include "../helper.hpp"
#include "../image.hpp"
#include "eyebot++.h"

constexpr int INITIAL_X = 500;
constexpr int INITIAL_Y = 3500;
constexpr int SRC = 2;
constexpr int DEST = IMAGE_SIZE - SRC;
constexpr int colours[] =
   {RED, GREEN, BLUE, CYAN, TEAL, MAGENTA, PURPLE, MAROON, YELLOW, OLIVE, ORANGE};
constexpr std::array<std::pair<int, int>, 8> neighbouring_indices = {{
   {0, -1},
   {0, 1},
   {-1, 0},
   {1, 0},
   {-1, -1},
   {-1, 1},
   {1, -1},
   {1, 1},
}};

auto get_neighbouring_ids(std::vector<std::vector<std::size_t>> const& ids,
                          int i,
                          int j) -> std::unordered_set<std::size_t> {
	// Including a 0 so no need to check if any of the neighbouring pixels have
	// an id just insert them
	std::unordered_set<std::size_t> neighbouring_ids{0};
	std::transform (neighbouring_indices.cbegin(),
	                neighbouring_indices.cend(),
	                std::inserter(neighbouring_ids, {}),
	                [&ids, i, j](std::pair<int, int> const& pair) {
		                return ids[i + pair.first][j + pair.second];
	                });
	return neighbouring_ids;
}

void plot_coloured_obstacles(std::vector<std::vector<std::size_t>> const& ids) {
	// Display all obstacles and 4 walls and give them unique colours
	BYTE coloured_obstacles[IMAGE_SIZE * IMAGE_SIZE * 3] = {};
	for (int i = 0; i < IMAGE_SIZE; ++i) {
		for (int j = 0; j < IMAGE_SIZE; ++j) {
			if (ids[i][j]) {
				std::size_t index = (i * IMAGE_SIZE + j) * 3;
				// 0 index can be used for colours since id starts from 1
				int colour = colours[ids[i][j] - 1];
				coloured_obstacles[index] = colour >> 16;
				coloured_obstacles[index + 1] = colour >> 8;
				coloured_obstacles[index + 2] = colour;
			}
		}
	}
	LCDImageStart(IMAGE_SIZE, IMAGE_SIZE, IMAGE_SIZE, IMAGE_SIZE);
	LCDImage(coloured_obstacles);
}

int main(int argc, char const* argv[]) {
	if (argc < 2) {
		std::cerr << "bushfire.x map.pbm\n";
		return EXIT_FAILURE;
	}

	BYTE* image;
	read_pbm(argv[1], &image);

	// Manually enter 4 walls
	for (std::size_t i = 0; i < IMAGE_SIZE; ++i) { // left
		image[i * IMAGE_SIZE] = 1;
	}
	for (std::size_t i = 0; i < IMAGE_SIZE; ++i) { // top
		image[i] = 1;
	}
	for (std::size_t i = 0; i < IMAGE_SIZE; ++i) { // right
		image[i * IMAGE_SIZE + IMAGE_SIZE - 1] = 1;
	}
	for (std::size_t i = IMAGE_SIZE * IMAGE_SIZE - 1;
	     i >= IMAGE_SIZE * IMAGE_SIZE - IMAGE_SIZE;
	     --i) { // bottom
		image[i] = 1;
	}

	// Start id from 1, 0 means no id
	std::unordered_map<std::size_t, std::unordered_set<Point, HashPoint>> obstacles{
	   {1, {}}, // left
	   {2, {}}, // right
	   {3, {}}, // top
	   {4, {}}, // bottom
	};
	for (std::size_t i = 0; i < IMAGE_SIZE; ++i) {
		obstacles[1].emplace(i, 0);
		obstacles[2].emplace(i, IMAGE_SIZE - 1);
		obstacles[3].emplace(0, i);
		obstacles[4].emplace(IMAGE_SIZE - 1, i);
	}

	std::size_t id = 5;
	std::vector<std::vector<std::size_t>> ids(
	   IMAGE_SIZE,
	   std::vector<std::size_t>(IMAGE_SIZE, 0));
	std::unordered_set<Point, HashPoint> voronoi_points;

	LCDImageStart(0, 0, IMAGE_SIZE, IMAGE_SIZE);
	LCDImageBinary(image);
	SIMSetRobot(0, INITIAL_X, INITIAL_Y, 0, INITIAL_ANGLE);
	LCDMenu("Obstacle", "Voronoi", "Drive", "Exit");
	while (true) {
		switch (KEYRead()) {
		case KEY1: {
			for (int i = 1; i < IMAGE_SIZE - 1; ++i) {
				for (int j = 1; j < IMAGE_SIZE - 1; ++j) {
					if (image[i * IMAGE_SIZE + j]) { // This pixel is occupied
						std::queue<std::size_t> ids;
						for (auto&& obstacle : obstacles) {
							// Check if its 8-nearest neighbours have a label
							if (std::any_of(
							       neighbouring_indices.cbegin(),
							       neighbouring_indices.cend(),
							       [&obstacle, i, j](std::pair<int, int> const& pair) {
								       return obstacle.second.contains(
								          {i + pair.first, j + pair.second});
							       }))
								ids.push(obstacle.first);
						}
						if (ids.empty()) { // Create a new label for this pixel
							obstacles[++id] = {{i, j}};
						} else { // Merge all labels into the first one
							std::size_t first = ids.front();
							ids.pop();
							obstacles[first].emplace(i, j);
							while (not ids.empty()) {
								std::size_t id = ids.front();
								ids.pop();
								obstacles[first].insert(obstacles[id].cbegin(),
								                        obstacles[id].cend());
								obstacles.erase(id);
							}
						}
					}
				}
			}

			// Set the ids for each pixel
			for (auto&& [id, pixels] : obstacles) {
				for (auto&& [i, j] : pixels) {
					ids[i][j] = id;
				}
			}
			plot_coloured_obstacles(ids);
			break;
		}
		case KEY2: {
			bool changes = true;
			while (changes) {
				// Make a copy of the ids for the current iteration
				std::vector<std::vector<std::size_t>> curr_ids = ids;

				changes = false;
				for (int i = 1; i < IMAGE_SIZE - 1; ++i) {
					for (int j = 1; j < IMAGE_SIZE - 1; ++j) {
						std::size_t id = ids[i][j];
						if (id == 0 and not voronoi_points.contains({i, j})) {
							std::unordered_set<std::size_t> neighbouring_ids =
							   get_neighbouring_ids(ids, i, j);
							if (neighbouring_ids.size() > 2) {
								// Neighbouring pixels have at least 2 different ids
								changes = true;
								voronoi_points.emplace(i, j);
								LCDPixel(j, i, BLACK);
							} else if (neighbouring_ids.size() > 1) {
								// Neighbouring pixels contain an id
								curr_ids[i][j] =
								   *std::find_if(neighbouring_ids.cbegin(),
								                 neighbouring_ids.cend(),
								                 [](int id) { return id != 0; });
								LCDPixel(j, i, LIGHTGRAY);
								if (get_neighbouring_ids(curr_ids, i, j).size() > 2) {
									// Neighbouring pixels have been overwritten with at
									// least 2 different ids in this iteration
									voronoi_points.emplace(i, j);
									LCDPixel(j, i, BLACK);
								}
							}
						}
					}
				}
				// Save the copy back to ids so the next iteration will have the
				// ids inserted in this iteration
				ids = curr_ids;
			}
			plot_coloured_obstacles(ids);
			std::cout << "Number of Voronoi Points: " << voronoi_points.size()
			          << std::endl;
			break;
		}
		case KEY3: {
			Graph graph;
			for (auto&& point : voronoi_points) {
				std::unordered_map<Point, double, HashPoint> neighbours;
				for (auto&& [i, j] : neighbouring_indices) {
					Point neighbour = {point.x + i, point.y + j};
					if (voronoi_points.contains(neighbour)) {
						neighbours.emplace(neighbour,
						                   euclidean_distance(neighbour.x - point.x,
						                                      neighbour.y - point.y));
					}
				}
				graph[point] = {neighbours,
				                euclidean_distance(DEST - point.x, DEST - point.y)};
			}

			std::vector<Point> src_to_dest =
			   a_star(graph, {SRC, SRC}, {DEST, DEST}).second;
			// Reduce the number of points in the path generated by A*
			std::vector<Point> reduced_path;
			std::size_t i = 0;
			std::copy_if(src_to_dest.cbegin(),
			             src_to_dest.cend(),
			             std::back_inserter(reduced_path),
			             [&i](Point const&) { return ++i % 20 == 0; });

			// Plot the reduced shortest path
			for (std::size_t i = 1; i < reduced_path.size(); ++i) {
				auto const& curr_point = reduced_path[i - 1];
				auto const& next_point = reduced_path[i];
				LCDLine(curr_point.y, curr_point.x, next_point.y, next_point.x, RED);
			}

			// Convert path from image coordinates to map coordinates
			std::vector<Position> path;
			std::transform (reduced_path.cbegin(),
			                reduced_path.cend(),
			                std::back_inserter(path),
			                [](Point const& point) {
				                return Position{point.y * WORLD_SIZE / IMAGE_SIZE,
				                                (IMAGE_SIZE - point.x) * WORLD_SIZE
				                                   / IMAGE_SIZE};
			                });

			// Drive to these waypoints
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
