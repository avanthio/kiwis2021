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
	//set the brake type of all the motors
	setBrakeTypes();
	pros::delay(20);

	//reset all the motors and pneumatics (set encoders to zero and pneumatics to correct start setting)
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
void autonomous() {
	goalGrabAuton();
}

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
	//bool goalLiftBool = 0;
	bool stickBool = 1;
	bool driveDirectBool = 0;


	while (true) {

		if(driveReverseBtn.changedToPressed()){
			driveDirectBool = !driveDirectBool;
		}

		if(driveDirectBool == 1){
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
		}
		else{
			axis3 = master.getAnalog(okapi::ControllerAnalog::leftY)*200;
			axis2 = master.getAnalog(okapi::ControllerAnalog::rightY)*200;

			leftFrontSpeed = axis2;
			leftBackSpeed = axis2;
			rightFrontSpeed = axis3;
			rightBackSpeed = axis3;


			leftFrontMotor.moveVelocity(-leftFrontSpeed);
			leftBackMotor.moveVelocity(-leftBackSpeed);
			rightFrontMotor.moveVelocity(-rightFrontSpeed);
			rightBackMotor.moveVelocity(-rightBackSpeed);
		}

		if(intakeInBtn.isPressed()){
			intakeMotor.moveVelocity(200);
		}
		else{
			intakeMotor.moveVelocity(0); 
		}


		if(hookOnBtn.isPressed()){
			hookMotor.moveVelocity(100);
		}
		else if(hookOffBtn.isPressed()){
			hookMotor.moveVelocity(-50);
		}
		else{
			hookMotor.moveVelocity(0);
		}


		if(liftUpBtn.isPressed()){
			liftMotor.moveVelocity(80);
		}
		else if(liftDownBtn.isPressed()){
			if(limitSwitch.get_value()==false){
				liftMotor.moveVelocity(-80);
			}
			else{
				liftMotor.moveVelocity(0);
				master.rumble("-."); //fun fact: this is "n" in morse code! (meaning the lift can't move any more)
			}
		}
		else{
			liftMotor.moveVelocity(0);
		}

		
		if(stickUpBtn.isPressed()){
			stickMotor.moveVelocity(25);
		}
		else if (stickDownBtn.isPressed()) {
			stickMotor.moveVelocity(-25);
		}
		else{
			stickMotor.moveVelocity(0);
		}

		if(stickPneumBtn.changedToPressed()){
			stickBool = !stickBool;
		}

		stickPneum.set_value(stickBool);
	

		pros::delay(20);
	}

}
