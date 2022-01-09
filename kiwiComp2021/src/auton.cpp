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
  hookPneum.set_value(true);
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
  hookPneum.set_value(true);
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

  fourBar.setGoalAngleAndVolt(0, 10000);
  fourBar.moveToAngle();
  moveForwardCoast(-5,-50);
  goalLiftPneum.set_value(true);
  moveForwardCoast(10,50);
  turnForDegrees(90);
  moveForwardCoast(15,100);
  struct Position goalPos = {-0.2,1,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  hookPneum.set_value(true);
  pros::delay(500);
  fourBar.setGoalAngleAndVolt(10,10000);
  fourBar.moveToAngle();
  goalPos = {0.75,0.38};
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(90,10000);
  fourBar.moveToAngle();
  goalPos = {1.19,0.15,0};
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(50,10000);
  fourBar.moveToAngle();
  hookPneum.set_value(false);
  fourBar.setGoalAngleAndVolt(55,10000);
  fourBar.moveToAngle();
  moveForwardCoast(-13,-75);
  fourBar.setGoalAngleAndVolt(0,10000);
  fourBar.moveToAngle();
  goalLiftPneum.set_value(false);
  pros::delay(500);
  moveForwardCoast(5,100);
  turnForDegrees(175);
  moveForwardCoast(7,100);
  hookPneum.set_value(true);
  fourBar.setGoalAngleAndVolt(90,10000);
  fourBar.moveToAngle();
  turnForDegrees(140);
  goalPos = {1.14,0.42,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  goalPos = {2,0.42,0};
  turnToFacePosition(goalPos);
  moveForwardCoast(2,100);
  fourBar.setGoalAngleAndVolt(45,8000);
  fourBar.moveToAngle();
  hookPneum.set_value(false);
  fourBar.setGoalAngleAndVolt(50,10000);
  fourBar.moveToAngle();
  moveForwardCoast(-10,-75);
  goalPos = {1.17,1.37,0};
  fourBar.setGoalAngleAndVolt(0,10000);
  fourBar.moveToAngle();
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  moveForwardCoast(2,75);
  hookPneum.set_value(true);  
  goalPos = {-0.8,0.24,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(90,10000);
  fourBar.moveToAngle();
  goalPos = {-1.14,0.07,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(50,10000);
  fourBar.moveToAngle();
  hookPneum.set_value(false);
  goalPos = {-0.85,-0.87,0,true};
  turnToFacePosition(goalPos);
  fourBar.setGoalAngleAndVolt(0,10000);
  fourBar.moveToAngle();
  moveToPosition(goalPos);
  goalPos = {-0.17,-0.9,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  hookPneum.set_value(true);
  fourBar.setGoalAngleAndVolt(5,10000);
  fourBar.moveToAngle();
  goalPos = {-0.84,-0.24,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(90,10000);
  fourBar.moveToAngle();
  goalPos = {-0.96,0,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  turnForDegrees(-60);
  moveForward(4,100);
  fourBar.setGoalAngleAndVolt(50,10000);
  fourBar.moveToAngle();
  hookPneum.set_value(false);
  moveForwardCoast(-10,-100);

}