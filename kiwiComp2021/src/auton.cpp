#include "auton.hpp"
#include "device_setup.hpp"

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

void bonusLineAuton(){
    moveForward(-25,-100);
    hookDrop();
    moveForward(20,100);
}

void goalGrabAuton(){
    moveForward(-52,-150);
    hookDrop();
    moveForward(50,150);
}