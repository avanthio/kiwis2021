#include "pid.hpp"

KiwiPID::KiwiPID(double kp,double ki, double kd){
  init();
  p = kp;
  i = ki;
  d = kd;
}

void KiwiPID::init(){
  p=0;
  i=0;
  d=0;

  iMax = 0;
  iMin = 0;
  deadzone = 0;
  errorSum = 0;
  maxErrForI = 0;
  lastError = 0;
  maxOutput = 0;
  minOutput = 0;
  rawOutput = 0;
  setpoint = 0;
  lastActual = 0;
  firstRun = true;
}

//resets some stored values for the PID
void KiwiPID::reset(){
  firstRun = true;
  errorSum = 0;
  lastError = 0;
  rawOutput = 0;
  iOutp = 0;
}

//sets how small the error must before the PID starts using integral
void KiwiPID::setMaxErrForI(double maxErrFrI){
  maxErrForI = maxErrFrI;
}

//sets the goal for the PID
void KiwiPID::setSetpoint(double setpnt){
  setpoint = setpnt;
}

void KiwiPID::setDeadzone(double deadZne){
  deadzone = deadZne;
}

//set the maximum output of the integral portion of the PID
void KiwiPID::setIMax(double IMaximum){
  iMax = IMaximum;
}

void KiwiPID::setIMin(double IMinimum){
  iMin = IMinimum;
}

void KiwiPID::setMaxOutput(double maxOut){
  maxOutput = maxOut;
}

void KiwiPID::setMinOutput(double minOut){
  minOutput = minOut;
}

double KiwiPID::getRawOutput(){
  return rawOutput;
}

double KiwiPID::getiOutput(){
  return iOutp;
}

double KiwiPID::getpOutput(){
  return pOutp;
}

double KiwiPID::getdOutput(){
  return dOutp;
}

double KiwiPID::getSetpoint(){
  return setpoint;
}

double KiwiPID::getOutput(double actual){
  double output;
  double pOutput;
  double iOutput;
  double dOutput;

  double error = setpoint - actual;

  pOutput = error*p;

  if(abs(error)<maxErrForI){
    errorSum += error;
  }
  else{
    errorSum = 0;
  }

  if(error*lastError<0){
    errorSum = 0;
  }

  iOutput = errorSum*i;

  //limit integral output to certain range
  if(iOutput>iMax){
    errorSum = iMax/i;
    iOutput = iMax;
  }

  if(iOutput<-iMax){
    errorSum = -iMax/i;
    iOutput = -iMax;
  }

  //make it so that if the integral output is very small and the error is in a certain range
  //the integral jumps to a minimum value
  if(abs(iOutput)<iMin&&error<maxErrForI&&error>0){
    iOutput = iMin+iOutput;
    errorSum = iMin/i;
  }
  else if(abs(iOutput)<iMin&&error>-maxErrForI&&error<0){
    iOutput = -iMin+iOutput;
    errorSum = -iMin/i;
  }


  dOutput = (error-lastError)*d;

  output = pOutput+iOutput+dOutput;
  rawOutput = output;

  if(output>maxOutput){
    output = maxOutput;
  }

  if(output<minOutput){
    output = minOutput;
  }

  //this seems to be working correctly (i think?)
  if(abs(error)<deadzone){
    output = 0;
    errorSum = 0;
  }

  lastError = error;
  iOutp = iOutput;
  pOutp = pOutput;
  dOutp = dOutput;
  return output;
}
