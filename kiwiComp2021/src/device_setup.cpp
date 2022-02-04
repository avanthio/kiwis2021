#include "device_setup.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/impl/device/button/controllerButton.hpp"
#include "okapi/impl/device/rotarysensor/IMU.hpp"
#include "pros/adi.hpp"


okapi::Controller master(okapi::ControllerId::master);
okapi::ControllerButton liftUpBtn(okapi::ControllerDigital::R1);
okapi::ControllerButton liftDownBtn(okapi::ControllerDigital::R2);
okapi::ControllerButton intakeInBtn(okapi::ControllerDigital::Y); //was L1
okapi::ControllerButton hookPneumBtn(okapi::ControllerDigital::L1); // was Y
okapi::ControllerButton goalLiftPneumBtn(okapi::ControllerDigital::L2);
//okapi::ControllerButton stickUpBtn(okapi::ControllerDigital::up);
//okapi::ControllerButton stickDownBtn(okapi::ControllerDigital::down);
//okapi::ControllerButton stickPneumBtn(okapi::ControllerDigital::left);
okapi::ControllerButton driveReverseBtn(okapi::ControllerDigital::A); //was L2


okapi::Motor  rightMiddleMotor(10, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  leftMiddleMotor(8,true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  leftFrontMotor(5,false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  leftBackMotor(16,false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  rightFrontMotor(6,true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  rightBackMotor(18,true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  intakeMotor(2, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
//okapi::Motor  hookMotor(2, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  liftMotor(11,true,okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
//okapi::Motor  stickMotor(20, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
//okapi::Motor liftLeftMotor(20, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);

//okapi::MotorGroup liftMotor({ liftRightMotor,liftLeftMotor});
pros::Imu inertialSens(14);
pros::ADIDigitalOut goalLiftPneum('A');
pros::ADIDigitalOut hookPneum('C');  
pros::ADIDigitalIn limitSwitch('B');

pros::Gps gpsSens(21);
//port, x pos, y pos, heading, x relative to center of rotation, y relative to center of rotation (all in meters, degrees)
