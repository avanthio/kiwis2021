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

//move forward (or backward, if both values are negative)
//at a certain velocity (rpm), for a certain number of inches
//an inertial sensor allows the robot to drive in a mostly straight line,
//even if the chain is janky (under most circumstances)
void moveForward(double targetDistance, int velo){
    double goal = calcDriveDegrees(targetDistance);
    double currEnc = 0;
    int leftVelo = velo;
    int rightVelo = velo;
    int veloAdjust = 1;
    double initialInert = inertialSens.get_rotation();
    double currInert = initialInert;
    double headingErr = currInert-initialInert;
    leftFrontMotor.tarePosition();
  	rightFrontMotor.tarePosition();
  	leftBackMotor.tarePosition();
  	rightBackMotor.tarePosition();
    pros::delay(loopDelay);

    while(1){
        currEnc = (leftFrontMotor.getPosition()+rightFrontMotor.getPosition()+rightBackMotor.getPosition()+leftBackMotor.getPosition())/4.0;

        currInert = inertialSens.get_rotation();
        headingErr = currInert-initialInert;
        if(abs(goal-currEnc)>15){

            //only adjust the direction of the robot if it is off
            //by more than one degree
            if(abs(headingErr)>1){
                    //why does it only adjust one side, you ask?
                    //because I don't want the robot to pivot or slow down too much
                    //and it was being strange when I adjusted both sides
                    if(headingErr<0){
                        leftVelo +=veloAdjust;
                        //rightVelo veloAdjust;
                    }
                    else{
                        //leftVelo -=veloAdjust;
                        rightVelo +=veloAdjust;
                    }
            }
            else{
                leftVelo = velo;
                rightVelo = velo;
            }

            leftFrontMotor.moveVelocity(leftVelo);
            leftBackMotor.moveVelocity(leftVelo);
            rightFrontMotor.moveVelocity(rightVelo);
            rightBackMotor.moveVelocity(rightVelo);
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

//turn a certain number of degrees based on the inertial sensor's readings:
//only make the first value negative to make it turn the opposite direction,
//veloc (rpm) must always be a positive number!!!!!
//this was written in this way to make future PID implementation easier
void turnForDegrees(double turnAngle, int veloc){
    double currRotation = inertialSens.get_rotation();
    double goalRotation = currRotation+turnAngle;
    int leftVeloc;
    int rightVeloc;
    int x = 0;
    double rotationErr = currRotation - goalRotation;

    while(1){
        if(rotationErr>0){
            leftVeloc = -veloc;
            rightVeloc = veloc;
        }
        else if(rotationErr<0){
            leftVeloc = veloc;
            rightVeloc = -veloc;
        }

        leftFrontMotor.moveVelocity(leftVeloc);
        leftBackMotor.moveVelocity(leftVeloc);
        rightFrontMotor.moveVelocity(rightVeloc);
        rightBackMotor.moveVelocity(rightVeloc);
        

        if(abs(rotationErr)<1){
            x+=1;
        }

        if(x>=1){
            break;
        }

        currRotation = inertialSens.get_rotation();
        rotationErr = currRotation - goalRotation;
        pros::delay(loopDelay);
    }
    
    leftFrontMotor.moveVelocity(0);
    rightFrontMotor.moveVelocity(0);
    leftBackMotor.moveVelocity(0);
    rightBackMotor.moveVelocity(0);

}