#include "device_management.hpp"
#include "device_setup.hpp"
#include "pros/llemu.hpp"

void setBrakeTypes(){
 leftBackMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
 leftMiddleMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
 rightMiddleMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
 rightBackMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
 rightFrontMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
 leftFrontMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
 intakeMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
 //hookMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
 liftMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
 //stickMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
}

void resetDevices(){
  pros::lcd::initialize();
  pros::lcd::set_text(1,"initializing...");
  inertialSens.reset();
  intakeMotor.tarePosition();
  leftFrontMotor.tarePosition();
  rightBackMotor.tarePosition();
  leftBackMotor.tarePosition();
  rightFrontMotor.tarePosition();
  //hookMotor.tarePosition();
  liftMotor.tarePosition();
  //stickMotor.tarePosition();
  goalLiftPneum.set_value(false);
  hookPneum.set_value(false);
  goalHookPneum.set_value(false);
  fourBar.reset();
  while(inertialSens.is_calibrating()){
    pros::delay(20);
  }
  pros::delay(20);
}

//convert an angle measure in degrees to radians
double degreesToRadians(double inputInDegrees){
  return inputInDegrees*M_PI/180;
}

//convert an angle measuer in radians to degrees
double radiansToDegrees(double inputInRadians){
  return inputInRadians*180/M_PI;
}