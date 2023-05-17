#include "eyebot++.h"
#include <cmath>
#include <fstream>
#include <vector>
#include <iostream>

constexpr float INTERVAL = 0.02;
constexpr int INITIAL_ANGLE = 0;
constexpr int SPEED = 50;
constexpr float K = 2;

void SplineDrive(int x, int y, int alpha) {
	int ax, ay, rphi;
	VWGetPosition(&ax, &ay, &rphi);

	int bx = x;
	int by = y;

	int lastx = ax;
	int lasty = ay;

	float len = K * std::sqrt(std::pow(x, 2) + std::pow(y, 2));
	float Dax = len;
	float Day = 0;
	float Dbx = len * std::cos(alpha * M_PI / 180);
	float Dby = len * std::sin(alpha * M_PI / 180);
	for (float u = 0; u < 1 + INTERVAL; u += INTERVAL) {
		int rx, ry;
		VWGetPosition(&rx, &ry, &rphi);

		float u2 = std::pow(u, 2);
		float u3 = std::pow(u, 3);

		float h0 = 2 * u3 - 3 * u2 + 1;
		float h1 = -2 * u3 + 3 * u2;
		float h2 = u3 - 2 * u2 + u;
		float h3 = u3 - u2;

		float sx = ax * h0 + bx * h1 + Dax * h2 + Dbx * h3;
		float sy = ay * h0 + by * h1 + Day * h2 + Dby * h3;

		int sphi = std::round(std::atan2(sy - lasty, sx - lastx) * 180 / M_PI);
		float distance = len / (K * std::floor(1 / INTERVAL));

		VWCurve(distance, sphi - rphi, SPEED);
		VWWait();

		lastx = sx;
		lasty = sy;
	}
	VWGetPosition(&ax, &ay, &rphi);
	LCDPrintf("(%d, %d) %d\n", ax, ay, rphi);
}

struct Pose {
	int x;
	int y;
	int phi;
};

int main(int argc, char const* argv[]) {
	if (argc < 2) {
		std::cerr << "spline_drive.x way.txt\n";
		return EXIT_FAILURE;
	}
	LCDMenu("Set", "Task1", "Task2", "END");
	while (true) {
		auto key = KEYRead();
		if (key == KEY1) {
			SIMSetRobot(0, 1500, 1500, 0, INITIAL_ANGLE);
			VWSetPosition(0, 0, 0);
		} else if (key == KEY2) {
			SplineDrive(-800, 500, -90);
		} else if (key == KEY3) {
			std::ifstream way(argv[1]);
			if (way.is_open()) {
				std::vector<Pose> waypoints{{0, 0, 0}};
				std::size_t i = 0;
				while (way >> waypoints[i].x >> waypoints[i].y) {
					if (i == 20) break;
					waypoints.push_back({0, 0, 0});
					++i;
				}
				waypoints.erase(--waypoints.cend());
				i = 0;
				int prev_x = 0;
				int prev_y = 0;
				while (true) {
					if (i == waypoints.size()) i = 0;
					LCDPrintf("%d %d\n", waypoints[i].x - prev_x, waypoints[i].y - prev_y);
					SplineDrive(waypoints[i].x - prev_x, waypoints[i].y - prev_y, INITIAL_ANGLE);
					prev_x = waypoints[i].x;
					prev_y = waypoints[i].y;
					++i;
				}
			}
		} else if (key == KEY4) {
			break;
		} else {
			OSWait(100);
		}
	}
	return 0;
}
