#ifndef _FLOODFILL_H
#define _FLOODFILL_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define STOP 0
#define STRAIGHT 1
#define TURN_RIGHT 2
#define TURN_LEFT 8
#define BACK 4

#define UP_DIR 1
#define RIGHT_DIR 2
#define DOWN_DIR 4
#define LEFT_DIR 8

#define turnRightDir(d) ({ __typeof__ (d) _d = (d); _d >= LEFT_DIR ? UP_DIR : (_d<<1); })
#define turnLeftDir(d) ({ __typeof__ (d) _d = (d); _d <= UP_DIR ? LEFT_DIR : (_d>>1); })
#define turn180(d) ({ __typeof__ (d) _d = (d); (_d == UP_DIR || _d==DOWN_DIR) ? (5 - _d) : (10 - _d); })

#define hasTopWall(c) (((c) & 0x01))
#define hasLeftWall(c) (((c) & 0x08) )
#define hasRightWall(c) (((c) & 0x02))
#define hasBottomWall(c) (((c) & 0x04))

#define hasWallInDir(walls, dir) (((walls) & (dir)))


#define insertTopWall(c) ((c) | 0x01)
#define insertLeftWall(c) ((c) | 0x08)
#define insertRightWall(c) ((c) | 0x02)
#define insertBottomWall(c) ((c) | 0x04)

#define CellToStack(x,y) (((x) & 0x0F)<<4 | ((y) & 0x0F))
#define StackToCellX(q) (((q)>>4 & 0x0F))
#define StackToCellY(q) (((q)& 0x0F))

void testStack();
void FFtestLoop();

extern short distances[16][16];
extern byte walls[16][16];
extern byte visited[16][16];

int EEPROM_save_maze();
int EEPROM_load_maze();

short getNextMovement(short x,short y,short actDir);
short updateDir(short actDir, short mov);
short yIncrementByDir(short dir);
short xIncrementByDir(short dir);
short  FFUpdateDistances(short xRobot, short yRobot);
void initializeDistancesCenterGoal();
void initializeDistancesIniGoal();

void updateCurrentWalls(short xRobot, short yRobot, short wallsRobot);

void printMaze(short xRobot, short yRobot, short dirRobot);

void initializeMinDistances();

#endif
