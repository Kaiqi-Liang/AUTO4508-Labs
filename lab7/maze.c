/* ************************************ */
/* Recursive Maze exploration program   */
/* in "MicroMouse" style                */
/*                                      */
/* Thomas Braunl, UWA, 1998, 2000, 2004 */
/* ************************************ */
#include "eyebot.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define DIST 360
#define SPEED 180
#define ASPEED 45
#define THRES 175
#define MAZESIZE 16

/* pos.0,0 is bottom left, dir. 0 is facing up (north) */
int mark[MAZESIZE][MAZESIZE]; /* 1 if visited */
int wall[MAZESIZE + 1][MAZESIZE + 1][2]; /* 1 if wall, 0 if free, -1 if unknown
                                          */
/*  BOTTOM: wall[x][y][0]  LEFT: wall[x][y][1] */
int map[MAZESIZE][MAZESIZE]; /* distance to goal */
int nmap[MAZESIZE][MAZESIZE]; /* copy */
int path[MAZESIZE * MAZESIZE]; /* shortest path */

int GRAPH = 1, DEBUG = 0, DEBUG2 = 0;

/** print mark full to X window (sim only).
    print positions that robot has already visited.  */
void print_mark_W() {
	int i, j;
	/* print to window (sim. only) */
	fprintf(stderr, "MARK\n");
	for (i = MAZESIZE - 1; i >= 0; i--) {
		for (j = 0; j < MAZESIZE; j++)
			if (mark[i][j])
				fprintf(stderr, "x ");
			else
				fprintf(stderr, ". ");
		fprintf(stderr, "\n");
	}
	fprintf(stderr, "\n");
}

/** print mark on LCD (6 lines).
    print positions that robot has already visited.  */
void print_mark() {
	int i, j;
	LCDSetPos(1, 0);
	for (i = 5; i >= 0; i--) {
		for (j = 0; j < 14; j++)
			if (mark[i][j])
				LCDPrintf("x");
			else
				LCDPrintf(".");
		if (i > 0) LCDPrintf("\n");
	}
}

/** print ful maze in X window (sim only).
    print maze as explored by robot.  */
void print_maze_W() {
	int i, j;
	fprintf(stderr, "MAZE\n");
	for (i = MAZESIZE; i >= 0; i--) {
		for (j = 0; j <= MAZESIZE; j++) {
			if (wall[i][j][1] == 1)
				fprintf(stderr, "|"); /* left   */
			else if (wall[i][j][1] == 0)
				fprintf(stderr, " ");
			else
				fprintf(stderr, ".");
			if (wall[i][j][0] == 1)
				fprintf(stderr, "_"); /* bottom */
			else if (wall[i][j][0] == 0)
				fprintf(stderr, " ");
			else
				fprintf(stderr, ".");
		}
		fprintf(stderr, "\n");
	}
	fprintf(stderr, "\n");
}

/** print ful maze on LCD (6 lines).
    print maze as explored by robot.  */
void print_maze() {
	int i, j;
	LCDSetPos(1, 0);
	for (i = 5; i >= 0; i--) {
		for (j = 0; j <= 6; j++) {
			if (wall[i][j][1] == 1)
				LCDPrintf("|"); /* left   */
			else if (wall[i][j][1] == 0)
				LCDPrintf(" ");
			else
				LCDPrintf(".");
			if (wall[i][j][0] == 1)
				LCDPrintf("_"); /* bottom */
			else if (wall[i][j][0] == 0)
				LCDPrintf(" ");
			else
				LCDPrintf(".");
		}
		if (i > 0) LCDPrintf("\n");
	}
}

void wall_set(int* w, int v) {
	if (*w == -1)
		*w = v; /* not seen before, set value */
	else if (*w != v) /* seen before and CONTRADITION */
	{
		*w = 1; /* assume wall to be safe */
		LCDSetPrintf(0, 0, "CONTRADICTION\n"); // AUBeep();
	}
}

/** maze_entry.
    enter recognized walls or doors.  */
void maze_entry(int x, int y, int dir, int open) {
	switch (dir) /* record bottom or left wall per square */
	{
	case 0:
		wall_set(&wall[y + 1][x][0], !open);
		break; /* top = bottom of next */
	case 2: wall_set(&wall[y][x][0], !open); break;
	case 1: wall_set(&wall[y][x][1], !open); break;
	case 3:
		wall_set(&wall[y][x + 1][1], !open);
		break; /* right = left of next */
	}
}

/** robo position and orientation.
   dir = 0, 1, 2, 3 equals: north, west, south, east.  */
int rob_x, rob_y, rob_dir;

/** init_maze.
    inits internal map of maze.
    set marks to 0 and walls to -1 (unknown).  */
void init_maze() {
	int i, j;
	for (i = 0; i < MAZESIZE; i++)
		for (j = 0; j < MAZESIZE; j++)
			mark[i][j] = 0;
	for (i = 0; i < MAZESIZE + 1; i++)
		for (j = 0; j < MAZESIZE + 1; j++) {
			wall[i][j][0] = -1;
			wall[i][j][1] = -1;
		}
}

int xneighbor(int x, int dir) {
	switch (dir) {
	case 0: return x; /* north */
	case 1: return x - 1; /* west  */
	case 2: return x; /* south */
	case 3:
	default: return x + 1; /* east  */
	}
}

int yneighbor(int y, int dir) {
	switch (dir) {
	case 0: return y + 1; /* north */
	case 1: return y; /* west  */
	case 2: return y - 1; /* south */
	case 3:
	default: return y; /* east  */
	}
}

int unmarked(int y, int x, int dir) {
	dir = (dir + 4) % 4;
	return !mark[yneighbor(y, dir)][xneighbor(x, dir)];
}

/** go_to.
  walk one square in current direction */
void go_to(int dir) {
	int turn;
	static int cur_x, cur_y, cur_p;

	dir = (dir + 4) % 4; /* keep goal dir in 0..3 */
	turn = dir - rob_dir;
	if (turn == 3)
		turn = -1; /* turn shorter angle */
	else if (turn == -3)
		turn = 1;

	if (turn) {
		if (DEBUG) LCDSetPrintf(13, 0, "Turn %d %d   ", turn * 90, ASPEED);
		VWTurn(turn * 90, ASPEED); /* turn */
		VWWait();
	}

	if (DEBUG) LCDSetPrintf(13, 0, "Straight %d %d   ", DIST, SPEED);

	int step = 20;
	for (size_t i = 0; i < DIST / step; ++i) {
		VWStraight(step, SPEED);
		VWWait();
		if (PSDGet(PSD_LEFT) < 100
		    || (PSDGet(PSD_RIGHT) < 200 && PSDGet(PSD_RIGHT) > 150)) {
			VWTurn(-25, 50);
			VWWait();
			VWStraight(step, SPEED);
			VWWait();
			VWTurn(10, 50);
			VWWait();
			++i;
		} else if (PSDGet(PSD_RIGHT) < 100
		           || (PSDGet(PSD_LEFT) < 200 && PSDGet(PSD_LEFT) > 150)) {
			VWTurn(25, 50);
			VWWait();
			VWStraight(step, SPEED);
			VWWait();
			VWTurn(-10, 50);
			VWWait();
			++i;
		}
	}

	VWGetPosition(&cur_x, &cur_y, &cur_p);
	if (DEBUG) LCDSetPrintf(14, 0, "X %d; Y %d; Phi %d   ", cur_x, cur_y, cur_p);

	rob_dir = dir;
	rob_x = xneighbor(rob_x, rob_dir);
	rob_y = yneighbor(rob_y, rob_dir);
}

/** check_mark.
    if ALL walls of a square are known, mark square as visited.
    this avoids unnecessary exploration.  */
void check_mark() {
	int i, j;
	for (i = 1; i < MAZESIZE; i++)
		for (j = 0; j < MAZESIZE - 1; j++)
		/* careful: watch boundaries!! i from 1 / j until size-1 */
		{
			if (wall[i][j][0] != -1 && wall[i][j][1] != -1 && /* bottom / left  */
			    wall[i + 1][j][0] != -1 && wall[i][j + 1][1] != -1) /* top    /
			                                                           right */
				mark[i][j] = 1;
		}
}

/**  explore.
    search maze goal from given start position and orientation.
    if more than one possible way: search all recursively.
    mark all visited maze squares.  */
void explore() {
	int front_open, left_open, right_open, old_dir;

	mark[rob_y][rob_x] = 1; /* mark current square */
	left_open = PSDGet(PSD_LEFT) > THRES;
	front_open = PSDGet(PSD_FRONT) > THRES;
	right_open = PSDGet(PSD_RIGHT) > THRES;
	maze_entry(rob_x, rob_y, rob_dir, front_open);
	maze_entry(rob_x, rob_y, (rob_dir + 1) % 4, left_open);
	maze_entry(rob_x, rob_y, (rob_dir + 3) % 4, right_open);
	check_mark();
	old_dir = rob_dir;

	if (GRAPH) {
		LCDSetPos(0, 0);
		LCDPrintf("Pos[%2d,%2d,%1d]", rob_x, rob_y, rob_dir);
		if (left_open)
			LCDSetPrintf(0, 13, "<");
		else
			LCDSetPrintf(0, 13, "|");
		if (front_open)
			LCDSetPrintf(0, 14, "^");
		else
			LCDSetPrintf(0, 14, "-");
		if (right_open)
			LCDSetPrintf(0, 15, ">");
		else
			LCDSetPrintf(0, 15, "|");
		print_maze();
	}

	if (DEBUG) {
		print_mark_W();
		print_maze_W();
		LCDMenu("Next", " ", " ", " ");
		KEYWait(KEY1);
		LCDMenu(" ", " ", " ", " ");
	}

	if (front_open && unmarked(rob_y, rob_x, old_dir)) /* then go straight */
	{
		go_to(old_dir); /* go 1 forward, 0 if first choice */
		explore(); /* recursive call */
		go_to(old_dir + 2); /* go 1 back */
	}

	if (left_open && unmarked(rob_y, rob_x, old_dir + 1)) /* then turn left */
	{
		go_to(old_dir + 1); /* go 1 left */
		explore(); /* recursive call */
		go_to(old_dir - 1); /* go 1 right, -1 = +3 */
	}

	if (right_open && unmarked(rob_y, rob_x, old_dir - 1)) /* then turn right */
	{
		go_to(old_dir - 1); /* go 1 right, -1 = +3 */
		explore(); /* recursive call */
		go_to(old_dir + 1); /* go 1 left */
	}
}

/** print shortest distances from start in X window (sim only).  */
void print_map_W() {
	int i, j;
	fprintf(stderr, "MAP\n");
	for (i = MAZESIZE - 1; i >= 0; i--) {
		for (j = 0; j < MAZESIZE; j++)
			fprintf(stderr, "%3d", map[i][j]);
		fprintf(stderr, "\n");
	}
	fprintf(stderr, "\n");
}

/** print shortest distances from start on LCD (6 lines).  */
void print_map() {
	int i, j;
	LCDClear();
	LCDPrintf("Map distances\n");
	for (i = 5; i >= 0; i--) {
		for (j = 0; j < 4; j++)
			LCDPrintf("%3d", map[i][j]);
		if (i > 0) LCDPrintf("\n");
	}
}

/** shortest_path.
    analyze shortest path after maze has been searched.
    returns path length to goal.
    or -1 if no path could be found.  */
int shortest_path(int goal_y, int goal_x) {
	int i, j, iter;

	LCDSetPrintf(0, 0, "                "); /* clear top line */
	LCDSetPrintf(0, 0, "Map..");
	for (i = 0; i < MAZESIZE; i++)
		for (j = 0; j < MAZESIZE; j++) {
			map[i][j] = -1; /* init */
			nmap[i][j] = -1;
		}
	map[0][0] = 0;
	nmap[0][0] = 0;
	iter = 0;

	do {
		iter++;
		for (i = 0; i < MAZESIZE; i++)
			for (j = 0; j < MAZESIZE; j++) {
				if (map[i][j] == -1) {
					if (i > 0)
						if (!wall[i][j][0] && map[i - 1][j] != -1)
							nmap[i][j] = map[i - 1][j] + 1;
					if (i < MAZESIZE - 1)
						if (!wall[i + 1][j][0] && map[i + 1][j] != -1)
							nmap[i][j] = map[i + 1][j] + 1;
					if (j > 0)
						if (!wall[i][j][1] && map[i][j - 1] != -1)
							nmap[i][j] = map[i][j - 1] + 1;
					if (j < MAZESIZE - 1)
						if (!wall[i][j + 1][1] && map[i][j + 1] != -1)
							nmap[i][j] = map[i][j + 1] + 1;
				}
			}

		for (i = 0; i < MAZESIZE; i++)
			for (j = 0; j < MAZESIZE; j++)
				map[i][j] = nmap[i][j]; /* copy back */

		if (DEBUG2) {
			print_map();
			print_map_W();
			LCDMenu("Next", " ", " ", " ");
			KEYWait(KEY1);
			LCDMenu(" ", " ", " ", " ");
		}
	} while (map[goal_y][goal_x] == -1 && iter < (MAZESIZE * MAZESIZE));
	LCDPrintf("done\n");
	return map[goal_y][goal_x];
}

/** build path.
  build shortest path after finding it.
  uses map and wall.
  sets path.  */
void build_path(int i, int j, int len) {
	int k;

	LCDSetPrintf(0, 0, "                "); /* clear top line */
	LCDSetPrintf(0, 0, "Path..");
	if (i <= 5 && j <= 6) LCDSetPrintf(6 - i, 2 * j + 1, "G"); /* mark goal */
	for (k = len - 1; k >= 0; k--) {
		if (i > 0 && !wall[i][j][0] && map[i - 1][j] == k) {
			i--;
			path[k] = 0; /* north */
		} else if (i < MAZESIZE - 1 && !wall[i + 1][j][0] && map[i + 1][j] == k) {
			i++;
			path[k] = 2; /* south */
		} else if (j > 0 && !wall[i][j][1] && map[i][j - 1] == k) {
			j--;
			path[k] = 3; /* east */
		} else if (j < MAZESIZE - 1 && !wall[i][j + 1][1] && map[i][j + 1] == k) {
			j++;
			path[k] = 1; /* west */
		} else {
			LCDPrintf("ERROR"); // AUBeep();
			KEYWait(ANYKEY);
		}

		/* mark path in maze on LCD */
		if (i <= 5 && j <= 6) {
			if (k > 0)
				LCDSetPrintf(6 - i, 2 * j + 1, "*"); /* path */
			else
				LCDSetPrintf(6 - i, 2 * j + 1, "S"); /* start */
		}
		if (DEBUG2) fprintf(stderr, "path %3d:%1d\n", k, path[k]);
	}
	LCDSetPrintf(0, 6, "done");
}

/** drive path.
  drive path after building it.
  parametere specifies start to finish (0).
  or finish to start (1).  */
void drive_path(int len, int reverse) {
	int i;
	if (reverse) {
		for (i = len - 1; i >= 0; i--)
			go_to(path[i] + 2);

		if (rob_dir != 0) /* back in start field */
		{
			VWTurn(-rob_dir * 90, ASPEED); /* turn */
			VWWait();
			rob_dir = 0;
		}
	}

	else
		for (i = 0; i < len; i++)
			go_to(path[i]);
}

/** main program.
    search maze from start in (0,0).
    search whole maze and generate map.
    @AUTHOR    Thomas Braunl, UWA, 1998.-- Updated 2017  */
int main() {
	int key, path_len;
	int goalY, goalX, incr;

	LCDPrintf("MAZE\nfull search\n\n");
	init_maze();

	LCDMenu("GO", "ROT", "DEBUG", "END");
	key = KEYGet();
	DEBUG = key == KEY3;
	if (key == KEY2) {
		VWTurn(90, 90);
		VWWait();
	}
	if (key == KEY4) return 0;

	rob_x = 0;
	rob_y = 0;
	rob_dir = 0; /* start in pos. 0,0, facing north */
	explore();

	/* back in [0,0] turn robot to original direction */
	if (rob_dir != 0) {
		VWTurn(-rob_dir * 90, ASPEED); /* turn */
		VWWait();
		rob_dir = 0;
	}

	do {
		print_maze();
		LCDMenu("Y+-", "X+-", "+/-", "GO");
		incr = 1;
		goalX = 0;
		goalY = 0;
		do {
			LCDSetPos(0, 0);
			LCDPrintf("GOAL y,x: %2d %2d", goalY, goalX); // AUBeep();
			if (goalY <= 5 && goalX <= 6) LCDSetPos(6 - goalY, 2 * goalX + 1);
			switch (key = KEYGet()) {
			case KEY1: goalY = (goalY + incr + MAZESIZE) % MAZESIZE; break;
			case KEY2: goalX = (goalX + incr + MAZESIZE) % MAZESIZE; break;
			case KEY3: incr = -incr; break;
			}
		} while (key != KEY4);
		LCDMenu(" ", " ", " ", " ");

		path_len = shortest_path(goalY, goalX); /* from start 0,0 to goal */
		if (path_len != -1) /* if path exists */
		{
			print_map_W();
			LCDSetPos(0, 0);
			LCDPrintf("Path length: %3d", path_len);
			build_path(goalY, goalX, path_len);

			do /* optional data display */
			{
				LCDMenu("Map", "Mrk", "Maz", "DRV");
				switch (key = KEYGet()) {
				case KEY1: print_map(); break;
				case KEY2: print_mark(); break;
				case KEY3: print_maze(); break;
				}
			} while (key != KEY4);

			drive_path(path_len, 0); /* drive start to finish */
			drive_path(path_len, 1); /* drive finish to start */
		} else {
			LCDSetPos(0, 0);
			LCDPrintf("No path exists!"); /*AUBeep();*/
		}

		LCDMenu("REP", " ", " ", "END");
	} while (KEYGet() != KEY4);
	return 0;
}
