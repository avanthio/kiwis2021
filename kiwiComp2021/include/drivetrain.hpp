#pragma once
#include "main.h"

struct Position{
  double x;
  double y;
  double angle;
  bool reversed = false;
};

extern bool gpsErrOut;

extern struct Position getCurrXangY();
extern double limitAngle(double);
extern struct Position getCurrXandY(double);
extern double calcHeadingToGoalPos(struct Position curr, struct Position goal);
extern void setUpPIDs();
extern void moveForward(double targetDistance, bool hookBool = false);
extern void moveForwardCoast(double targetDistance, int velo);
extern void turnForDegrees(double turnAngle);
extern void turnForDegrees2(double turnAngle);
extern void turnToFacePosition(struct Position goal);
extern void moveToPosition(struct Position goal, bool liftArm = false, double distanceToStartLift = 0);
extern void moveForwardTest(double targetDistance, bool hookBool, double distanceForArmMotion);