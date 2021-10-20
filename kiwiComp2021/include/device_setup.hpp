#pragma once

#include "main.h"

//declare the buttons and controller object
extern okapi::Controller master;
extern okapi::ControllerButton liftUpBtn;
extern okapi::ControllerButton liftDownBtn;
extern okapi::ControllerButton intakeInBtn;
extern okapi::ControllerButton hookOnBtn;
extern okapi::ControllerButton hookOffBtn;
extern okapi::ControllerButton goalLiftPneumBtn;


//declare all the motors
extern okapi::Motor  leftFrontMotor;
extern okapi::Motor  leftBackMotor;
extern okapi::Motor  rightFrontMotor;
extern okapi::Motor  rightBackMotor;
extern okapi::Motor  intakeMotor;
extern okapi::Motor  hookMotor;
extern okapi::Motor  liftMotor;


//declare the pneumatics
extern pros::ADIDigitalOut goalLiftLeftPneum;
extern pros::ADIDigitalOut goalLiftRightPneum;

//declare the limit switch
extern pros::ADIDigitalIn limitSwitch;