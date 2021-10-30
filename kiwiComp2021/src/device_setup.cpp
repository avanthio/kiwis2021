#include "device_setup.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/impl/device/button/controllerButton.hpp"
#include "pros/adi.hpp"


okapi::Controller master(okapi::ControllerId::master);
okapi::ControllerButton liftUpBtn(okapi::ControllerDigital::R1);
okapi::ControllerButton liftDownBtn(okapi::ControllerDigital::R2);
okapi::ControllerButton intakeInBtn(okapi::ControllerDigital::L1);
okapi::ControllerButton hookOnBtn(okapi::ControllerDigital::Y);
okapi::ControllerButton hookOffBtn(okapi::ControllerDigital::A);
//okapi::ControllerButton goalLiftPneumBtn(okapi::ControllerDigital::up);
okapi::ControllerButton stickUpBtn(okapi::ControllerDigital::up);
okapi::ControllerButton stickDownBtn(okapi::ControllerDigital::down);
okapi::ControllerButton stickPneumBtn(okapi::ControllerDigital::left);
okapi::ControllerButton driveReverseBtn(okapi::ControllerDigital::L2);


okapi::Motor  leftFrontMotor(1,false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  leftBackMotor(11,false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  rightFrontMotor(10,true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  rightBackMotor(12,true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  intakeMotor(8, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  hookMotor(17, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  liftMotor(19,false,okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  stickMotor(20, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);

 
//pros::ADIDigitalOut goalLiftPneum('B');
pros::ADIDigitalOut stickPneum('C');
pros::ADIDigitalIn limitSwitch('A');