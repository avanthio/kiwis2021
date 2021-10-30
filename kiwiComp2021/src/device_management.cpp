#include "device_management.hpp"
#include "device_setup.hpp"

void setBrakeTypes(){
 leftBackMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
 rightBackMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
 rightFrontMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
 leftFrontMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
 intakeMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
 hookMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
 liftMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
 stickMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
}

void resetDevices(){
  intakeMotor.tarePosition();
  leftFrontMotor.tarePosition();
  rightBackMotor.tarePosition();
  leftBackMotor.tarePosition();
  rightFrontMotor.tarePosition();
  hookMotor.tarePosition();
  liftMotor.tarePosition();
  stickMotor.tarePosition();
  //goalLiftPneum.set_value(false);
  stickPneum.set_value(true);
}