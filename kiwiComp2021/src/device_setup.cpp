#include "device_setup.hpp"
#include "okapi/impl/device/button/controllerButton.hpp"


okapi::Controller master(okapi::ControllerId::master);
okapi::ControllerButton liftUpBtn(okapi::ControllerDigital::R1);
okapi::ControllerButton liftDownBtn(okapi::ControllerDigital::R2);
okapi::ControllerButton intakeInBtn(okapi::ControllerDigital::L1);
okapi::ControllerButton hookOnBtn(okapi::ControllerDigital::Y);
okapi::ControllerButton hookOffBtn(okapi::ControllerDigital::A);
okapi::ControllerButton goalLiftPneumBtn(okapi::ControllerDigital::up);


okapi::Motor  leftFrontMotor(1,false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  leftBackMotor(11,false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  rightFrontMotor(2,true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  rightBackMotor(12,true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  intakeMotor(1, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  hookMotor(17, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  liftMotor(20,false,okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);


pros::ADIDigitalOut goalLiftLeftPneum('C');
pros::ADIDigitalOut goalLiftRightPneum('B');
pros::ADIDigitalIn limitSwitch('A');