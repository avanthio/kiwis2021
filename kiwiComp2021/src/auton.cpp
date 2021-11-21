#include "auton.hpp"
#include "device_setup.hpp"

//I made a function to put the hook down!
//it can break the loop if the hook gets stuck: 
//this means I don't have to worry about it getting jammed
//and blocking the rest of the code from running
void hookDrop(){
    hookMotor.tarePosition();
    double hookGoal = -60;
    double currHookEnc = 0;
    double lastHookEnc = 0;
    int hookVel = 80;
    int x = 0;

    while(abs(hookGoal-currHookEnc)>5){
        currHookEnc = hookMotor.getPosition();
        hookMotor.moveVelocity(-hookVel);

        if(abs(currHookEnc-lastHookEnc)<1){
            x+=1;
        }

        lastHookEnc = currHookEnc;

        if(x>100){
            break;
        }

        pros::delay(20);
    }

    hookMotor.moveVelocity(0);
}

//pulls the goal off the WP bonus line and/or scores a ring in it:
//if our alliance partner has a functional auton, this will allow us
//to consistently get the WP from autonomous
void bonusLineAuton(){
    moveForward(-25,-100);
    pros::delay(500);
    goalLiftPneum.set_value(true);
    moveForward(20,100);
}

//a somewhat adventurous auton that grabs a goal from the neutral zone:
//mostly consistent,
//but doesn't get the goal if someone's already stolen it
void goalGrabAuton(){
    moveForward(-52,-150);
    pros::delay(500);
    goalLiftPneum.set_value(true);
    moveForward(50,150);
}

//this is the risky auton (~8/10 success rate):
//it grabs a goal from the neutral zone and also
//scores a ring in our alliance goal and moves it away
//from the bonus line
void wpAndGoalRight(){
    moveForward(-42,-200);
    pros::delay(250);
    goalLiftPneum.set_value(true);
    pros::delay(200);
    pros::delay(0);
    moveForward(15,200);
    turnForDegrees(-65,50);
    moveForward(15,100);
    hookDrop();
    turnForDegrees(60,100);
    moveForward(15,150);
}

//I didn't write this auton because I didn't have time
//it would do a similar thing to the other auton above
//but on the left side of the field (so no bonus line stuff)
void wpAndGoalLeft(){
}