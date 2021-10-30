#pragma once
#include "main.h"

class KiwiPID{
public:
  KiwiPID(double, double, double);
  void reset();
  void init();

  void setSetpoint(double);
  void setMaxErrForI(double);
  void setIMax(double);
  void setIMin(double);
  void setDeadzone(double);
  void setMaxOutput(double);
  void setMinOutput(double);
  double getOutput(double);
  double getOutput(double setpt, double actual);
  double getRawOutput();
  double getiOutput();
  double getpOutput();
  double getdOutput();


private:
  double p;
  double i;
  double d;
  double iMax;
  double iMin;
  double iOutp;
  double dOutp;
  double pOutp;
  double maxOutput;
  double minOutput;
  double setpoint;
  double deadzone;
  double errorSum;
  double lastError;
  double lastActual;
  double rawOutput;
  bool firstRun;
  double maxErrForI;
  void initialize();

};
