#include "eyebot++.h"
// Thruster IDs.
constexpr int LEFT = 1;
constexpr int RIGHT = 3;

constexpr int EPSILON = 100;
constexpr int SPEED = 70;

void forward(float secs, int speed = SPEED) {
	MOTORDrive(LEFT, speed);
	MOTORDrive(RIGHT, speed);
	sleep(secs);
	MOTORDrive(LEFT, 0);
	MOTORDrive(RIGHT, 0);
}

void turn_left(float secs, int speed = SPEED) {
	MOTORDrive(LEFT, -speed);
	MOTORDrive(RIGHT, speed);
	sleep(secs);
	MOTORDrive(LEFT, 0);
	MOTORDrive(RIGHT, 0);
}

void turn_right(float secs, int speed = SPEED) {
	MOTORDrive(LEFT, speed);
	MOTORDrive(RIGHT, -speed);
	sleep(secs);
	MOTORDrive(LEFT, 0);
	MOTORDrive(RIGHT, 0);
}

void turn_right_align() {
	LCDPrintf("Turn right to face a corner\n");
	turn_right(1);
	int distF;
	do {
		distF = PSDGet(PSD_FRONT);
		turn_right(0.1);
	} while (abs(distF - PSDGet(PSD_FRONT)) > 2);
}

int main() {
	LCDMenu("START", "", "", "END");
	KEYWait(KEY1);
	LCDPrintf("Drive straight to a wall\n");
	int prev, curr;
	while (PSDGet(PSD_FRONT) > 1500) forward(1);
	turn_right_align();

	std::size_t count = 0;
	while (true) {
		prev = PSDGet(PSD_LEFT);
		forward(1);
		curr = PSDGet(PSD_LEFT);
		if (PSDGet(PSD_FRONT) < 1500) {
			++count;
			turn_right(3);
			if (count > 4) {
			    LCDPrintf("Stop\n");
				KEYWait(KEY4);
                return 0;
			}
		} else if (curr - prev > EPSILON or curr > 2000) {
			turn_left(1, 40);
			forward(1);
			turn_right(1, 25);
		} else if (prev - curr > EPSILON or curr < 1000) {
			turn_right(1, 40);
			forward(1);
			turn_left(1, 25);
		}
	}
	return 0;
}