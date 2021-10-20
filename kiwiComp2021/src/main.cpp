#include "main.h"
#include "device_management.hpp"
#include "device_setup.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	//set the brake type of all the motos
	setBrakeTypes();
	pros::delay(20);
	//reset all the motors and pneumatics
	//set encoders to zero and pneumatics to off/false
	resetDevices();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	//used to store values of controller joysticks
	int axis2 = 0;
	int axis3 = 0;
	//used to store speed of each motor
	int leftFrontSpeed;
	int leftBackSpeed;
	int rightFrontSpeed;
	int rightBackSpeed;
	bool goalLiftBool = 0;
	bool stickPneumBool = 0;


	while (true) {
		axis3 = master.getAnalog(okapi::ControllerAnalog::leftY)*200;
		axis2 = master.getAnalog(okapi::ControllerAnalog::rightY)*200;

		leftFrontSpeed = axis3;
		leftBackSpeed = axis3;
		rightFrontSpeed = axis2;
		rightBackSpeed = axis2;


		leftFrontMotor.moveVelocity(leftFrontSpeed);
		leftBackMotor.moveVelocity(leftBackSpeed);
		rightFrontMotor.moveVelocity(rightFrontSpeed);
		rightBackMotor.moveVelocity(rightBackSpeed);

		if(intakeInBtn.isPressed()){
			intakeMotor.moveVelocity(200);
		}
		else{
			intakeMotor.moveVelocity(0); 
		}


		if(hookOnBtn.changedToPressed()){
			hookMotor.moveRelative(90,200);
		}
		else if(hookOffBtn.changedToPressed()){
			hookMotor.moveRelative(-90,200);
		}


		if(liftUpBtn.isPressed()){
			liftMotor.moveVelocity(100);
		}
		else if(liftDownBtn.isPressed()){
			if(limitSwitch.get_value()==false){
				liftMotor.moveVelocity(-100);
			}
			else{
				liftMotor.moveVelocity(0);
				master.rumble("-."); //fun fact: this is "n" in morse code! (meaning the lift can't move any more)
			}
		}
		else{
			liftMotor.moveVelocity(0);
		}

		


		if(goalLiftPneumBtn.changedToPressed()){
			goalLiftBool = !goalLiftBool; 
		}
		
		goalLiftLeftPneum.set_value(goalLiftBool);
		goalLiftRightPneum.set_value(goalLiftBool);

		pros::delay(20);
	}

}
