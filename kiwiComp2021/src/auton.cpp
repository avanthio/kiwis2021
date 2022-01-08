#include "auton.hpp"
#include "device_setup.hpp"
#include "drivetrain.hpp"


//this is the risky auton (~8/10 success rate):
//it grabs a goal from the neutral zone and also
//scores a ring in our alliance goal and moves it away
//from the bonus line
void wpAndGoalRight(){
  fourBar.setGoalAngleAndVolt(0, 8000);
  fourBar.moveToAngle();
  pros::delay(100);
  moveForward(44.5,200,true);
  hookPneum.set_value(false);
  fourBar.setGoalAngleAndVolt(5, 8000);
  fourBar.moveToAngle();
  moveForward(-28.5,200);
  turnForDegrees(-90);
  moveForwardCoast(-10,-150);
  pros::delay(1000);
  goalLiftPneum.set_value(true);
  moveForward(25,100);
  goalLiftPneum.set_value(false);
}

//this is the other risky auton (~8/10 success rate):
//it grabs a goal from the neutral zone and also
//scores a ring in our alliance goal on the "left" side of the field
void wpAndGoalLeft(){
  fourBar.setGoalAngleAndVolt(0, 8000);
  fourBar.moveToAngle();
  pros::delay(100);
  moveForward(52,200,true);
  hookPneum.set_value(false);
  fourBar.setGoalAngleAndVolt(5, 8000);
  fourBar.moveToAngle();
  moveForward(-46,200);
  turnForDegrees(-80);
  moveForwardCoast(-10,-100);
  goalLiftPneum.set_value(true);
  pros::delay(500);
  goalLiftPneum.set_value(false);

}


void autonSkills(){

  moveForward(-5,-50);
  goalLiftPneum.set_value(true);
  moveForward(10,50);
  turnForDegrees(90);
  moveForward(15,100);
  struct Position goalPos = {-0.2,1,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(0, 3000);
  fourBar.moveToAngle();
  moveToPosition(goalPos);
  hookPneum.set_value(false);
  pros::delay(500);
  fourBar.setGoalAngleAndVolt(10,3000);
  fourBar.moveToAngle();
  goalPos = {0.75,0.38};
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(90,6000);
  fourBar.moveToAngle();
  goalPos = {1.19,0.15,0};
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(45,7000);
  fourBar.moveToAngle();
  hookPneum.set_value(true);
  fourBar.setGoalAngleAndVolt(50,7000);
  fourBar.moveToAngle();
  moveForward(-13,-75);
  fourBar.setGoalAngleAndVolt(0,8000);
  fourBar.moveToAngle();
  goalLiftPneum.set_value(false);
  pros::delay(500);
  moveForward(3,100);
  turnForDegrees(175);
  moveForward(5,100);
  hookPneum.set_value(false);
  fourBar.setGoalAngleAndVolt(90,6000);
  fourBar.moveToAngle();
  turnForDegrees(140);
  goalPos = {1.14,0.42,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  goalPos = {2,0.42,0};
  turnToFacePosition(goalPos);
  moveForward(2,100);
  fourBar.setGoalAngleAndVolt(45,6000);
  fourBar.moveToAngle();
  hookPneum.set_value(true);
  fourBar.setGoalAngleAndVolt(50,7000);
  fourBar.moveToAngle();
  moveForward(-10,-75);
  goalPos = {1.1,1.37,0};
  fourBar.setGoalAngleAndVolt(0,8000);
  fourBar.moveToAngle();
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  moveForward(2,75);
  hookPneum.set_value(false);  
  goalPos = {-0.8,0.24,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(90,6000);
  fourBar.moveToAngle();
  goalPos = {-1.14,0.07,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(45,6000);
  fourBar.moveToAngle();
  hookPneum.set_value(true);
  fourBar.setGoalAngleAndVolt(50,7000);
  fourBar.moveToAngle();
}