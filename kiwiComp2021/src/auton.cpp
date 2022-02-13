#include "auton.hpp"
#include "device_setup.hpp"
#include "drivetrain.hpp"


//this is the risky auton (~8/10 success rate):
//it grabs a goal from the neutral zone and also
//scores a ring in our alliance goal and moves it away
//from the bonus line

void wpAndGoalRight(){
  fourBar.setGoalAngleAndVolt(0, 8000);
  moveForwardTest(39, true, 5);
  moveForward(-25);
  fourBar.setGoalAngleAndVolt(20,8000);
  fourBar.moveToAngle();
  turnForDegrees(-90);
  moveForwardCoast(-11,8000);
  goalLiftPneum.set_value(true);
  pros::delay(2000);
  moveForward(15);
  intakeMotor.moveVelocity(190);
  pros::delay(3000);
  goalLiftPneum.set_value(false);
  intakeMotor.moveVelocity(0);
  
}

//this is the other risky auton (~8/10 success rate):
//it grabs a goal from the neutral zone and also
//scores a ring in our alliance goal on the "left" side of the field
void wpAndGoalLeft(){
  fourBar.setGoalAngleAndVolt(0, 8000);
  moveForwardTest(39, true, 5);
  moveForward(-33);
  fourBar.setGoalAngleAndVolt(20,8000);
  fourBar.moveToAngle();
  turnForDegrees(-55);
  moveForwardCoast(-5.5,8000);
  intakeMotor.moveVelocity(190);
  pros::delay(3000);
  intakeMotor.moveVelocity(0);
}

void fullWP(){
  hookPneum.set_value(true);
  pros::delay(1000);
  hookPneum.set_value(false);
  moveForwardCoast(-6,8000);
  turnForDegrees(-90);
  moveForward(24);
  turnForDegrees(88);
  moveForward(80);
  turnForDegrees(180);
  moveForwardCoast(-10,8000);
  goalLiftPneum.set_value(true);
  pros::delay(1000);
  intakeMotor.moveVelocity(190);
  moveForwardCoast(15,8000);
  goalLiftPneum.set_value(false);
}

void autonSkills(){

  moveForwardCoast(-5,8000);
  goalLiftPneum.set_value(true);
  moveForwardCoast(10,8000);
  fourBar.setGoalAngleAndVolt(0, 12000);
  fourBar.moveToAngle();
  turnForDegrees(90);
  moveForwardCoast(15,12000);
  struct Position goalPos = {-0.2,1,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  hookPneum.set_value(true);
  pros::delay(500);
  fourBar.setGoalAngleAndVolt(10,12000);
  fourBar.moveToAngle();
  goalPos = {0.75,0.38};
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(90,12000);
  fourBar.moveToAngle();
  goalPos = {1.19,0.2,0};
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(50,12000);
  fourBar.moveToAngle();
  hookPneum.set_value(false);
  fourBar.setGoalAngleAndVolt(55,12000);
  fourBar.moveToAngle();
  moveForwardCoast(-13,12000);
  fourBar.setGoalAngleAndVolt(0,12000);
  fourBar.moveToAngle();
  goalLiftPneum.set_value(false);
  pros::delay(500);
  moveForwardCoast(5,12000);
  turnForDegrees(175);
  moveForwardCoast(7,12000);
  hookPneum.set_value(true);
  fourBar.setGoalAngleAndVolt(90,12000);
  fourBar.moveToAngle();
  turnForDegrees(140);
  goalPos = {1.14,0.42,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  goalPos = {2,0.42,0};
  turnToFacePosition(goalPos);
  moveForwardCoast(2,12000);
  fourBar.setGoalAngleAndVolt(45,12000);
  fourBar.moveToAngle();
  hookPneum.set_value(false);
  fourBar.setGoalAngleAndVolt(50,12000);
  fourBar.moveToAngle();
  moveForwardCoast(-10,8000);
  goalPos = {1.17,1.37,0};
  fourBar.setGoalAngleAndVolt(0,12000);
  fourBar.moveToAngle();
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  moveForwardCoast(2,75);
  hookPneum.set_value(true);  
  goalPos = {-0.8,0.24,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(90,12000);
  fourBar.moveToAngle();
  goalPos = {-1.14,0.07,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(50,12000);
  fourBar.moveToAngle();
  hookPneum.set_value(false);
  goalPos = {-0.85,-0.87,0,true};
  turnToFacePosition(goalPos);
  fourBar.setGoalAngleAndVolt(0,12000);
  fourBar.moveToAngle();
  moveToPosition(goalPos);
  goalPos = {-0.17,-0.9,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  hookPneum.set_value(true);
  fourBar.setGoalAngleAndVolt(5,12000);
  fourBar.moveToAngle();
  goalPos = {-0.84,-0.24,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(90,12000);
  fourBar.moveToAngle();
  goalPos = {-0.96,0,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  turnForDegrees(-60);
  moveForward(4,12000);
  fourBar.setGoalAngleAndVolt(50,12000);
  fourBar.moveToAngle();
  hookPneum.set_value(false);
  moveForwardCoast(-10,12000);

}