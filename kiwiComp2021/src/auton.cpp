#include "auton.hpp"
#include "device_setup.hpp"
#include "drivetrain.hpp"


//this is the risky auton (~8/10 success rate):
//it grabs a goal from the neutral zone and also
//scores a ring in our alliance goal and moves it away
//from the bonus line

void wpAndGoalRight(){
  hookPneum.set_value(true);
  pros::lcd::print(1,">:(");
  moveForwardTest(44,true,1000);
  pros::delay(350);
  int goalCheck = opticalSens.get_proximity();
  if(goalCheck<200){
    moveForward(-27);
    turnForDegrees(-90);
    moveForwardCoast(-15,8000);
    goalHookPneum.set_value(true);
    pros::delay(100);
    goalLiftPneum.set_value(true);
    pros::delay(500);
    fourBar.setGoalAngleAndVolt(20,8000);
    fourBar.moveToAngle();
    intakeMotor.moveVelocity(500);
    moveForwardCoast(5,8000);
    turnForDegrees(90);
    moveForwardCoast(30,4000);
  }
  else{
    moveForward(-35);
    turnForDegrees(-90);
    moveForward(-12);
    goalHookPneum.set_value(true);
    pros::delay(100);
    goalLiftPneum.set_value(true);
    pros::delay(500);
    fourBar.setGoalAngleAndVolt(20,8000);
    fourBar.moveToAngle();
    intakeMotor.moveVelocity(500);
    moveForwardCoast(2,8000);
    turnForDegrees(90);
    moveForwardCoast(20,5000);
  }

  moveForward(-30);
  intakeMotor.moveVelocity(0);
  goalLiftPneum.set_value(false);
  pros::delay(250);
  goalHookPneum.set_value(false);  
  
}

//this is the other risky auton (~8/10 success rate):
//it grabs a goal from the neutral zone and also
//scores a ring in our alliance goal on the "left" side of the field
void wpAndGoalLeft(){
  hookPneum.set_value(true);
  //fourBar.setGoalAngleAndVolt(0, 8000);
  //pros::delay(1000);*/
  pros::lcd::print(1,">:(");
  moveForwardTest(46,true,1000);
  pros::delay(350);
  int goalCheck = opticalSens.get_proximity();
  if(goalCheck>200){
    moveForward(-40);
  }
  else{
    moveForward(-35);
  }
    
    turnForDegrees(-45);
    moveForward(-11);
    goalHookPneum.set_value(true);
    pros::delay(100);
    goalLiftPneum.set_value(true);
    pros::delay(500);
    moveForward(11);
    fourBar.setGoalAngleAndVolt(20,8000);
    fourBar.moveToAngle();
    intakeMotor.moveVelocity(600);
    turnForDegrees(80);
    moveForwardCoast(30,6000);
    pros::delay(500);
    moveForward(-20);
    goalLiftPneum.set_value(false);
    pros::delay(500);
    goalHookPneum.set_value(false);

}

//get the win point alone 
//only for use if partner does not have good auton
//and the opposing alliance is really good/really not that good
void fullWP(){
  //moveForwardCoast(2,12000);
  fourBar.setGoalAngleAndVolt(10,12000);
  fourBar.moveToAngle();
  moveForwardCoast(-5,12000);
  goalHookPneum.set_value(true);
  pros::delay(100);
  goalHookPneum.set_value(false);
  moveForward(15);
  turnForDegrees(90);
  moveForward(20);
  turnForDegrees(87);
  moveForward(77);
  turnForDegrees(180);
  moveForward(-20);
  goalHookPneum.set_value(true);
  pros::delay(100);
  goalLiftPneum.set_value(true);
  pros::delay(500);
  intakeMotor.moveVelocity(600);
  moveForward(15);
  pros::delay(1000);
  goalLiftPneum.set_value(false);

}


//120 (usually) or 160 (if really lucky) point auton skills
void autonSkills(){
  
  moveForwardCoast(-5.5,8000);
  goalLiftPneum.set_value(true);
  moveForwardCoast(10,8000);
  fourBar.setGoalAngleAndVolt(0, 12000);
  //fourBar.moveToAngle();
  turnForDegrees(90);
  moveForwardCoast(15,12000);
  struct Position goalPos = {-0.22,0.96,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos, true, 0.05);
  hookPneum.set_value(true);
  pros::delay(500);
  fourBar.setGoalAngleAndVolt(90,12000);
  //fourBar.moveToAngle();
  goalPos = {0.75,0.38};
  moveToPosition(goalPos,true,0.05);
  goalPos = {1.19,0.2,0};
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(45,12000);
  fourBar.moveToAngle();
  hookPneum.set_value(false);
  fourBar.setGoalAngleAndVolt(55,12000);
  fourBar.moveToAngle();
  moveForwardCoast(-13,12000);
  fourBar.setGoalAngleAndVolt(0,12000);
  fourBar.moveToAngle();
  goalLiftPneum.set_value(false);
  pros::delay(500);
  moveForwardCoast(9,12000);
  turnForDegrees(175);
  moveForwardCoast(11,12000);
  hookPneum.set_value(true);
  fourBar.setGoalAngleAndVolt(90,12000);
  fourBar.moveToAngle();
  //turnForDegrees(140);
  goalPos = {1.14,0.25,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  goalPos = {2,0.25,0};
  turnToFacePosition(goalPos);
  //moveForwardCoast(2,12000);
  fourBar.setGoalAngleAndVolt(50,12000);
  fourBar.moveToAngle();
  hookPneum.set_value(false);
  moveForwardCoast(-10,8000);
  goalPos = {0.96,1.36,0};
  fourBar.setGoalAngleAndVolt(0,12000);
  //fourBar.moveToAngle();
  turnToFacePosition(goalPos);
  moveToPosition(goalPos, true, 0);
  //moveForwardCoast(2,150);
  hookPneum.set_value(true);  
  goalPos = {-1.0,0.95,0};
  moveForwardCoast(-14,8000);
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  fourBar.setGoalAngleAndVolt(90,12000);
  //fourBar.moveToAngle();
  goalPos = {-1.14,0.07,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos,true,0);
  turnForDegrees(60);
  fourBar.setGoalAngleAndVolt(55,12000);
  fourBar.moveToAngle();
  hookPneum.set_value(false);
  goalPos = {-0.64,-0.54,0,true};
  fourBar.setGoalAngleAndVolt(0,12000);
  //turnForDegrees(-450);
  turnToFacePosition(goalPos);
  moveToPosition(goalPos,true,0);
  goalPos = {-0.15,-0.91,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos);
  hookPneum.set_value(true);
  fourBar.setGoalAngleAndVolt(90,12000);
  goalPos = {-1.14,-0.1,0};
  turnToFacePosition(goalPos);
  moveToPosition(goalPos,true,0);
  turnForDegrees(-60);
  moveForwardCoast(4,12000);
  fourBar.setGoalAngleAndVolt(55,12000);
  fourBar.moveToAngle();
  hookPneum.set_value(false);
  moveForwardCoast(-8,8000);

}

void noGpsSkills(){
  fourBar.setGoalAngleAndVolt(0, 8000);
  moveForwardTest(39, true, 5);
  moveForward(-25);
  fourBar.setGoalAngleAndVolt(20,12000);
  fourBar.moveToAngle();
  turnForDegrees(-90);
  moveForwardCoast(-11,8000);
  goalLiftPneum.set_value(true);
  pros::delay(500);
  moveForward(10);
  turnForDegrees(90);
  moveForward(64);
  turnForDegrees(-90);
  moveForward(38);
  fourBar.setGoalAngleAndVolt(90,12000);
  fourBar.moveToAngle();
  turnForDegrees(90);
  moveForwardCoast(6,12000);
  fourBar.setGoalAngleAndVolt(50,12000);
  fourBar.moveToAngle();
  hookPneum.set_value(false);
  goalLiftPneum.set_value(false);
  pros::delay(1500);
  moveForwardCoast(-14,12000);
  fourBar.setGoalAngleAndVolt(0,12000);
  fourBar.moveToAngle();
  moveForwardCoast(8,12000);
  turnForDegrees(180);
  moveForward(8,true);
  hookPneum.set_value(true);
  fourBar.setGoalAngleAndVolt(65,12000);
  fourBar.moveToAngle();
  turnForDegrees(180);
  moveForward(15);
  hookPneum.set_value(false);
  moveForwardCoast(-70,6000);
  turnForDegrees(-45);
  moveForwardCoast(64,12000);



}