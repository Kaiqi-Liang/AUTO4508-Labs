#include "../helper.hpp"
#include "eyebot++.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

constexpr int INITIAL_X = 500;
constexpr int INITIAL_Y = 500;

enum State {
	DRIVING,
	ROTATING,
	FOLLOWING,
};

int main(int argc, char const* argv[]) {
	if (argc < 3) {
		std::cerr << "distbug.x x y\n";
		return EXIT_FAILURE;
	}
	Position GOAL = {atoi(argv[1]), atoi(argv[2])};

	LCDMenu("Set", "Start", "", "END");
	KEYWait(KEY1);
	SIMSetRobot(0, INITIAL_X, INITIAL_Y, 0, INITIAL_ANGLE);
	VWSetPosition(INITIAL_X, INITIAL_Y, INITIAL_ANGLE);

	KEYWait(KEY2);
	int dists[COMPLETE_ANGLE];
	int x, y, phi;
	State state = DRIVING;
	VWSetSpeed(DRIVING_SPEED, 0);
	Position hit;
	bool left_hit_point = false;
	int d_min = SENSOR_INF_DIST;
	int prev;
	while (true) {
		VWGetPosition(&x, &y, &phi);
		Position curr = {x, y};
		int dx = GOAL.x - curr.x;
		int dy = GOAL.y - curr.y;
		float theta = std::atan2(dy, dx) * STRAIGHT_ANGLE / M_PI;
		if (theta > STRAIGHT_ANGLE) theta -= COMPLETE_ANGLE;
		float diff = std::round(theta - phi);

		if (curr == GOAL) {
			LCDPrintf("Goal found\n");
			STOP;
			KEYWait(KEY4);
			break;
		}

		// Calculate the difference between the goal angle and the robot’s
		// heading angle to find the goal heading from the robot’s current pose
		LIDARGet(dists);
		switch (state) {
		case DRIVING: { // Drive towards the goal
			if (dists[LIDAR_FRONT] < SAFE_DISTANCE
			    or dists[LIDAR_FRONT_LEFT] < ABSOLUTE_SAFE_DISTANCE
			    or dists[LIDAR_FRONT_RIGHT] < ABSOLUTE_SAFE_DISTANCE) {
				LCDPrintf("Start rotating\n");
				STOP;
				state = ROTATING;
				hit = curr;
			} else if (abs(diff) > 1.0) {
				VWSetSpeed(DRIVING_SPEED, diff);
			} else {
				VWSetSpeed(DRIVING_SPEED, 0);
			}
			break;
		}
		case ROTATING: { // Turn left until parallel to obstacle boundary
			VWTurn(ADJUSTING_ANGLE, ADJUSTING_SPEED);
			VWWait();
			if (dists[LIDAR_RIGHT] < SAFE_DISTANCE
			    and dists[LIDAR_MORE_RIGHT] >= dists[LIDAR_RIGHT]
			    and dists[LIDAR_LESS_RIGHT] >= dists[LIDAR_RIGHT]) {
				LCDPrintf("Start wall following\n");
				VWSetSpeed(ADJUSTING_SPEED, 0);
				state = FOLLOWING;
			}
			break;
		}
		case FOLLOWING: { // Follow along obstacle boundary
			if (not left_hit_point and hit != curr) {
				left_hit_point = true;
			} else if (left_hit_point and hit == curr) {
				LCDPrintf("Goal unreachable\n");
				STOP;
				KEYWait(KEY4);
				break;
			}
			prev = dists[LIDAR_RIGHT];
			OS_WAIT;
			LIDARGet(dists);
			if (dists[LIDAR_RIGHT] > 350) {
				std::cout << "Too far from the wall\n";
				VWTurn(-20, ADJUSTING_SPEED);
				VWWait();
				VWStraight(120, ADJUSTING_SPEED);
				VWWait();
			} else if (dists[LIDAR_FRONT] < SAFE_DISTANCE) {
				std::cout << "Heading straight to the wall\n";
				VWTurn(RIGHT_ANGLE, ADJUSTING_SPEED);
				VWWait();
				VWStraight(25, ADJUSTING_SPEED);
				VWWait();
			} else if (dists[LIDAR_RIGHT] < 50
			           or dists[LIDAR_FRONT_RIGHT] < ABSOLUTE_SAFE_DISTANCE) {
				std::cout << "Too close to the wall\n";
				VWTurn(20, ADJUSTING_SPEED);
				VWWait();
				VWStraight(120, ADJUSTING_SPEED);
				VWWait();
			} else if (dists[LIDAR_RIGHT] - prev > EPSILON
			           or dists[LIDAR_RIGHT] > SAFE_DISTANCE
			           or dists[LIDAR_MORE_RIGHT] <= dists[LIDAR_RIGHT]) {
				VWSetSpeed(ADJUSTING_SPEED, -ADJUSTING_ANGLE);
			} else if (prev - dists[LIDAR_RIGHT] > EPSILON
			           or dists[LIDAR_RIGHT] < ABSOLUTE_SAFE_DISTANCE
			           or dists[LIDAR_LESS_RIGHT] <= dists[LIDAR_RIGHT]) {
				VWSetSpeed(ADJUSTING_SPEED, ADJUSTING_ANGLE);
			}

			int d = euclidean_distance(dx, dy);
			d_min = std::min(d, d_min);

			int angle = STRAIGHT_ANGLE - (theta - phi);
			if (angle < 0) angle += COMPLETE_ANGLE;
			int f = dists[angle];

			if (d - f <= d_min - STEP) {
				VWStraight(250, DRIVING_SPEED);
				VWWait();
				VWTurn(diff, ADJUSTING_SPEED);
				VWWait();
				LCDPrintf("Start driving\n");
				state = DRIVING;
				left_hit_point = false;
				d_min = SENSOR_INF_DIST;
			}
		}
		}
	}
	std::cout << "end" << '\n';
	return EXIT_SUCCESS;
}
