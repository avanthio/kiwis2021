#pragma once
#include "main.h"

struct Position{
  double x;
  double y;
  double angle;
  bool reversed = false;
};

extern struct Position getCurrXangY();
extern double limitAngle(double);
extern struct Position getCurrXandY(double);
extern double calcHeadingToGoalPos(struct Position curr, struct Position goal);
extern void setUpPIDs();
extern void moveForward(double targetDistance, int velo, bool hookBool = false);
extern void moveForwardCoast(double targetDistance, int velo);
extern void turnForDegrees(double turnAngle);
extern void turnToFacePosition(struct Position goal);
extern void moveToPosition(struct Position goal);
extern void moveForwardTest(double targetDistance, int veloc, bool hookBool, double distanceForArmMotion);