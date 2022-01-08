#pragma once

#include "main.h"
#include "okapi/impl/device/button/controllerButton.hpp"
#include "pros/adi.hpp"

//declare the buttons and controller object
extern okapi::Controller master;
extern okapi::ControllerButton liftUpBtn;
extern okapi::ControllerButton liftDownBtn;
extern okapi::ControllerButton intakeInBtn;
extern okapi::ControllerButton hookPneumBtn;
extern okapi::ControllerButton goalLiftPneumBtn;
//extern okapi::ControllerButton stickUpBtn;
//extern okapi::ControllerButton stickDownBtn;
//extern okapi::ControllerButton stickPneumBtn;
extern okapi::ControllerButton driveReverseBtn;

//declare all the motors
extern okapi::Motor  leftFrontMotor;
extern okapi::Motor  leftBackMotor;
extern okapi::Motor  leftMiddleMotor;
extern okapi::Motor  rightMiddleMotor;
extern okapi::Motor  rightFrontMotor;
extern okapi::Motor  rightBackMotor;
extern okapi::Motor  intakeMotor;
//extern okapi::Motor  hookMotor;
extern okapi::Motor  liftLeftMotor;
//extern okapi::Motor  stickMotor;
extern okapi::Motor  liftRightMotor;
extern okapi::MotorGroup liftMotor;

extern pros::Imu inertialSens;
//declare the pneumatics
extern pros::ADIDigitalOut goalLiftPneum;
extern pros::ADIDigitalOut hookPneum;
//declare the limit switch
extern pros::ADIDigitalIn limitSwitch;

extern pros::Gps gpsSens;
