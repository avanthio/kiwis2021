#include "drivetrain.hpp"
#include "device_setup.hpp"

const int loopDelay = 20;


const double tireCircumference = 3.25*M_PI;//Never, ever, under any circumstances mess with this



//- for calculating distance driving forward in inches
//- divides goal distance by tire circumference (calculate wheel rotations needed to move a given distance)
//- then multiply by 360 to get the degrees of motor rotation
double calcDriveDegrees( double targetDistanceInInches ) {
    return (targetDistanceInInches/tireCircumference)*360.0 ;
}

void moveForward(double targetDistance, int velo){
    double goal = calcDriveDegrees(targetDistance);
    double currEnc = 0;

    leftFrontMotor.tarePosition();
  	rightFrontMotor.tarePosition();
  	leftBackMotor.tarePosition();
  	rightBackMotor.tarePosition();
    pros::delay(loopDelay);

    while(1){
        currEnc = (leftFrontMotor.getPosition()+rightFrontMotor.getPosition()+rightBackMotor.getPosition()+leftBackMotor.getPosition())/4.0;

        if(abs(goal-currEnc)>15){
            leftFrontMotor.moveVelocity(velo);
            rightFrontMotor.moveVelocity(velo);
            leftBackMotor.moveVelocity(velo);
            rightBackMotor.moveVelocity(velo);
        }
        else{
            break;
        }
        pros::delay(loopDelay);
    }

    leftFrontMotor.moveVelocity(0);
    rightFrontMotor.moveVelocity(0);
    leftBackMotor.moveVelocity(0);
    rightBackMotor.moveVelocity(0);
}