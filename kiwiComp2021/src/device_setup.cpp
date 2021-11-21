#include "device_setup.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/impl/device/button/controllerButton.hpp"
#include "okapi/impl/device/rotarysensor/IMU.hpp"
#include "pros/adi.hpp"


okapi::Controller master(okapi::ControllerId::master);
okapi::ControllerButton liftUpBtn(okapi::ControllerDigital::R1);
okapi::ControllerButton liftDownBtn(okapi::ControllerDigital::R2);
okapi::ControllerButton intakeInBtn(okapi::ControllerDigital::Y); //was L1
okapi::ControllerButton hookOnBtn(okapi::ControllerDigital::L1); // was Y
okapi::ControllerButton hookOffBtn(okapi::ControllerDigital::L2); // was A
okapi::ControllerButton goalLiftPneumBtn(okapi::ControllerDigital::up);
//okapi::ControllerButton stickUpBtn(okapi::ControllerDigital::up);
//okapi::ControllerButton stickDownBtn(okapi::ControllerDigital::down);
//okapi::ControllerButton stickPneumBtn(okapi::ControllerDigital::left);
okapi::ControllerButton driveReverseBtn(okapi::ControllerDigital::A); //was L2


okapi::Motor  leftFrontMotor(11,false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  leftBackMotor(18,false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  rightFrontMotor(14,true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  rightBackMotor(16,true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  intakeMotor(8, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  hookMotor(2, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor  liftMotor(10,false,okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);
//okapi::Motor  stickMotor(20, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::degrees);

pros::Imu inertialSens(20);
pros::ADIDigitalOut goalLiftPneum('A');
//pros::ADIDigitalOut stickPneum('C');  
pros::ADIDigitalIn limitSwitch('B');