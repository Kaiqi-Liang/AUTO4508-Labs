#include <algorithm>
#include <array>
#include <cmath>

#include "eyebot++.h"
#include "../helper.hpp"

void align() {
	int prev, curr;
	prev = PSDGet(PSD_RIGHT);
	VWStraight(25, 100);
	VWWait();
	curr = PSDGet(PSD_RIGHT);
	if (curr - prev > EPSILON) {
		VWTurn(-ADJUSTING_ANGLE, ADJUSTING_SPEED);
		VWWait();
	} else if (prev - curr > EPSILON) {
		VWTurn(ADJUSTING_ANGLE, ADJUSTING_SPEED);
		VWWait();
	}
	VWStraight(25, 100);
	VWWait();
}

void turn() {
	LCDPrintf("Turning to be parallel to the wall\n");
	double l = PSDGet(PSD_LEFT);
	double r = PSDGet(PSD_RIGHT);
	double f = PSDGet(PSD_FRONT);
	double alpha;
	if (l > r) {
		alpha = std::atan2(r, f) * 180 / M_PI;
		VWTurn(-(180 - alpha), ADJUSTING_SPEED);
	} else {
		alpha = std::atan2(l, f) * 180 / M_PI;
		VWTurn(-alpha, ADJUSTING_SPEED);
	}
	VWWait();
}

int main() {
	LCDMenu("START", "", "", "END");
	KEYWait(KEY1);

	LCDPrintf("Drive straight to a wall\n");
	VWSetSpeed(300, 0);
	int prev, curr;
	bool parallel = false;
	while (PSDGet(PSD_FRONT) > SAFE_DISTANCE) {
		prev = PSDGet(PSD_RIGHT);
		OS_WAIT;
		curr = PSDGet(PSD_RIGHT);
		if (curr - prev < EPSILON) parallel = true;
	}
	STOP;

	LCDPrintf("Turn right to face a corner\n");
	if (parallel) {
		VWTurn(-RIGHT_ANGLE, ADJUSTING_SPEED);
		VWWait();
	} else {
		turn();
	}

	std::size_t count = 1;
	while (true) {
		LCDPrintf("Drive straight to a wall\n");
		while (PSDGet(PSD_FRONT) > SAFE_DISTANCE) align();

		LCDPrintf("Take a U-turn\n");
		// even count negative angle (clockwise)
		int direction = (count % 2 ? -1 : 1);
		STOP;
		VWTurn(RIGHT_ANGLE * direction, ADJUSTING_SPEED);
		VWWait();

		if (PSDGet(PSD_FRONT) < SAFE_DISTANCE) {
			LCDPrintf("Stop\n");
    		KEYWait(KEY4);
            return 0;
		}

		// drive forward 200 (4 * 50) every align drives forward 50
		for (std::size_t i = 0; i < 4; ++i) align();

		VWTurn(RIGHT_ANGLE * direction, ADJUSTING_SPEED);
		VWWait();
		++count;
	}
	return 0;
}
