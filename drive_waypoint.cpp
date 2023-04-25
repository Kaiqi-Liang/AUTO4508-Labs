#include "eyebot++.h"
#include "helper.hpp"

int main() {
	LCDMenu("SET", "DRIVE", "", "END");
	KEYWait(KEY1);
	SIMSetRobot(0, 1500, 1500, 0, INITIAL_ANGLE);
	VWSetPosition(1500, 1500, INITIAL_ANGLE);

	KEYWait(KEY2);
	drive_waypoint(1000, 500);

	KEYWait(KEY4);
	return 0;
}
