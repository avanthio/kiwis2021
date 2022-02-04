#include "drivetrain.hpp"
#include "device_management.hpp"
#include "device_setup.hpp"
#include "okapi/api/filter/averageFilter.hpp"


const int loopDelay = 20;

KiwiPID turnPID(150,10,250);
KiwiPID straightPID(10000,180,390);//(500,10,20)//1000,1,20
KiwiPID angleAdjustPID((6.0/360.0),0,0);
KiwiPID forwardPID(900,8,475);

//This function manages the PIDs at the start of the program.
//It is called in the initialize function in main.cpp
void setUpPIDs(){


  turnPID.setMaxOutput(12000);
  turnPID.setMinOutput(-12000);
  straightPID.setMaxOutput(12000);
  straightPID.setMinOutput(-12000);
  forwardPID.setMaxOutput(12000);
  forwardPID.setMinOutput(-12000);
  angleAdjustPID.setMaxOutput(2500);
  angleAdjustPID.setMinOutput(-2500);

  turnPID.setSetpoint(0);
  straightPID.setSetpoint(0);
  angleAdjustPID.setSetpoint(0);
  forwardPID.setSetpoint(0);

  turnPID.setIMax(12000);
  turnPID.setIMin(1000);
  turnPID.setMaxErrForI(10);
  turnPID.setDeadzone(1);
  straightPID.setIMax(6000);
  straightPID.setIMin(0);
  straightPID.setMaxErrForI(10);
  straightPID.setDeadzone(0);
  angleAdjustPID.setIMax(6000);
  angleAdjustPID.setDeadzone(0);
  angleAdjustPID.setIMin(0);
  angleAdjustPID.setMaxErrForI(1000);
  forwardPID.setIMax(6000);
  forwardPID.setIMin(0);
  forwardPID.setMaxErrForI(10);
  forwardPID.setDeadzone(0);

}

const double tireCircumference = 3.25*M_PI;//Never, ever, under any circumstances mess with this

int getSign(double x){
  int sign = 0;
  if(x>0){
      sign = 1;
  }
  else if(x<0){
    sign = -1;
  }
  else{
    sign = 0;
  }
  return sign;
} 

double limitAngle(double angle) {
  
  while (angle >= 360 || angle < 0) {
    while (angle >= 360) {
      angle -= 360;
      pros::delay(10);
    }
    while (angle < 0) {
      angle += 360;
      pros::delay(10);
    }
    pros::delay(10);
  }
  
  return angle;
}

//calculate the necessary heading of the robot (in degrees) to reach a goal position,
//based on current position and goal position
//0 degrees is in the direction of positive y and the returned value is >=0 and <360
//it works in all quadrants and potential situations (y 0, x+- and x 0, y+-)
double calcHeadingToGoalPos(struct Position curr, struct Position goal) {

  double theta = 0;

  if ((goal.y - curr.y) != 0) {
    theta = atan((goal.x - curr.x)/(goal.y - curr.y));
    int signTheta = getSign(theta);
    if (goal.x > curr.x) {
      if (signTheta == -1) {
        theta += M_PI;
      }
    }
    else if (goal.x < curr.x) {
      if (signTheta == 1) {
        theta += M_PI;
      }
      else if (signTheta == -1) {
        theta += (2 * M_PI);
      }
    }
    else if (goal.y == curr.y) {
      if (goal.x > curr.x) {
        theta = 0;
      }
      else {
        theta = M_PI;
      }
    }
  }
  else {
    theta = 0;
    if (goal.y < curr.y) {
      theta = M_PI;
    }
  }
  

  theta = radiansToDegrees(theta);
  
  return theta;
}

//- for calculating distance driving forward in inches
//- divides goal distance by tire circumference (calculate wheel rotations needed to move a given distance)
//- then multiply by 360 to get the degrees of motor rotation
double calcDriveDegrees( double targetDistanceInInches ) {
  return (targetDistanceInInches/tireCircumference)*240;
}

double calcDriveDistance (double degreesOfEnc){
  return (degreesOfEnc/240.0)*tireCircumference;
}


double distanceToPoint(Position current, Position goal){
  Position error;
  error.x = goal.x-current.x;
  error.y = goal.y-current.y;

  return sqrt(error.x*error.x+error.y*error.y);
}


//move forward (or backward, if both values are negative)
//at a certain velocity (rpm), for a certain number of inches
//an inertial sensor allows the robot to drive in a mostly straight line,
//even if the chain is janky (under most circumstances)
void moveForward(double targetDistance, int veloc, bool hookBool){
  forwardPID.reset();

  int x = 0;
  int velo = veloc;
  double goal = calcDriveDegrees(targetDistance);
  double distanceErr;
  double currEnc = 0;
  int leftVelo = velo;
  int loopCount = 0;
  int rightVelo = velo;
  int veloAdjust = 60;
  double adjustVelocityPCT;
  double initialInert = inertialSens.get_rotation();
  double currInert = initialInert;
  double headingErr = currInert - initialInert;
  int sign;
  bool notSetAlready = true;


  pros::delay(20);

  leftFrontMotor.tarePosition();
  rightFrontMotor.tarePosition();
  leftBackMotor.tarePosition();
  rightBackMotor.tarePosition();

  pros::delay(loopDelay);

  while (1) {
    currEnc = (leftFrontMotor.getPosition() + rightFrontMotor.getPosition() +
               rightBackMotor.getPosition() + leftBackMotor.getPosition()) /
              4.0;

    
    
    if(loopCount%20 == 0||abs(currEnc-goal)<50){
      std::cout<<currEnc<<"\n";
    }
    distanceErr = calcDriveDistance(goal - currEnc);

    
    velo = forwardPID.getOutput(-distanceErr);

    if(hookBool == true){
        sign = getSign(velo);
        if(sign == -1 && notSetAlready == true){
          hookPneum.set_value(true);
          notSetAlready = false;
        }
    }

    currInert = inertialSens.get_rotation();
    headingErr = currInert - initialInert;
    //if (abs(goal - currEnc) > 15) {

      // only adjust the direction of the robot if it is off
      // by more than one degree
      if (abs(headingErr) > 1) {
        // why does it only adjust one side, you ask?
        // because I don't want the robot to pivot or slow down too much
        // and it was being strange when I adjusted both sides
        if (headingErr < 0) {
          leftVelo += veloAdjust;
          // rightVelo veloAdjust;
        } 
        else {
          // leftVelo -=veloAdjust;
          rightVelo += veloAdjust;
        }
      } 
      else {
        leftVelo = velo;
        rightVelo = velo;
      }

      leftFrontMotor.moveVoltage(leftVelo);
      leftMiddleMotor.moveVoltage(leftVelo);
      leftBackMotor.moveVoltage(leftVelo);
      rightFrontMotor.moveVoltage(rightVelo);
      rightMiddleMotor.moveVoltage(rightVelo);
      rightBackMotor.moveVoltage(rightVelo);
    

    if(abs(goal-currEnc)<10){
      x+=1;
    }
    else{
      x=0;
    }

    if(x>10){
      break;
    }

    pros::delay(loopDelay);
    loopCount +=1;
  }

  leftFrontMotor.moveVelocity(0);
  leftMiddleMotor.moveVelocity(0);
  rightMiddleMotor.moveVelocity(0);
  rightFrontMotor.moveVelocity(0);
  leftBackMotor.moveVelocity(0);
  rightBackMotor.moveVelocity(0);

}


void moveForwardCoast(double targetDistance, int veloc){
  forwardPID.reset();

  int x = 0;
  int velo = veloc;
  double goal = calcDriveDegrees(targetDistance);
  double distanceErr;
  double currEnc = 0;
  int leftVelo = velo;
  int loopCount = 0;
  int rightVelo = velo;
  int veloAdjust = 1;
  double adjustVelocityPCT;
  double initialInert = inertialSens.get_rotation();
  double currInert = initialInert;
  double headingErr = currInert - initialInert;




  pros::delay(20);

  leftFrontMotor.tarePosition();
  rightFrontMotor.tarePosition();
  leftBackMotor.tarePosition();
  rightBackMotor.tarePosition();

  pros::delay(loopDelay);

  while (1) {
    currEnc = (leftFrontMotor.getPosition() + rightFrontMotor.getPosition() +
               rightBackMotor.getPosition() + leftBackMotor.getPosition()) /
              4.0;

    
    
    if(loopCount%20 == 0||abs(currEnc-goal)<50){
      std::cout<<currEnc<<"\n";
    }
    distanceErr = calcDriveDistance(goal - currEnc);

    velo = forwardPID.getOutput(-distanceErr);

    currInert = inertialSens.get_rotation();
    headingErr = currInert - initialInert;
    //if (abs(goal - currEnc) > 15) {

      // only adjust the direction of the robot if it is off
      // by more than one degree
    if (abs(headingErr) > 1) {
        // why does it only adjust one side, you ask?
        // because I don't want the robot to pivot or slow down too much
        // and it was being strange when I adjusted both sides
        if (headingErr < 0) {
          leftVelo += veloAdjust;
          // rightVelo veloAdjust;
        } 
        else {
          // leftVelo -=veloAdjust;
          rightVelo += veloAdjust;
        }
      } 
    else {
        leftVelo = veloc;
        rightVelo = veloc;
    }

      leftFrontMotor.moveVelocity(leftVelo);
      leftMiddleMotor.moveVelocity(leftVelo);
      leftBackMotor.moveVelocity(leftVelo);
      rightFrontMotor.moveVelocity(rightVelo);
      rightMiddleMotor.moveVelocity(rightVelo);
      rightBackMotor.moveVelocity(rightVelo);
    

    if(abs(goal-currEnc)<10){
      break;
    }
    pros::delay(loopDelay);
    loopCount +=1;
  }

  leftFrontMotor.moveVelocity(0);
  leftMiddleMotor.moveVelocity(0);
  rightMiddleMotor.moveVelocity(0);
  rightFrontMotor.moveVelocity(0);
  leftBackMotor.moveVelocity(0);
  rightBackMotor.moveVelocity(0);

}

// turn a certain number of degrees based on the inertial sensor's readings:
// only make the first value negative to make it turn the opposite direction,
// veloc (rpm) must always be a positive number!!!!!
// this was written in this way to make future PID implementation easier
void turnForDegrees(double turnAngle) {

  turnPID.reset();
  double currRotation = inertialSens.get_rotation();
  double goalRotation = currRotation + turnAngle;
  int leftVeloc;
  int rightVeloc;
  int veloc = 0;
  int x = 0;
  double lastRotationErr;
  double rotationErr = currRotation - goalRotation;
  double avgRotationErrChange;
  int loopCount;
  while (1) {

    veloc = turnPID.getOutput(rotationErr);
    //std::cout<<veloc<<"\n";

    
    leftVeloc = veloc;
    rightVeloc = -veloc;
    

    leftFrontMotor.moveVoltage(leftVeloc);
    leftMiddleMotor.moveVoltage(leftVeloc);
    leftBackMotor.moveVoltage(leftVeloc);
    rightFrontMotor.moveVoltage(rightVeloc);
    rightMiddleMotor.moveVoltage(rightVeloc);
    rightBackMotor.moveVoltage(rightVeloc);

    if (abs(rotationErr) < 2) {
      x += 1;
    }
    else{
      x = 0;
    }

    if (x >= 10) {
      break;
    }

    /*if(loopCount%10==0){
      avgRotationErrChange = avgRotationErrChange/10;
      if(avgRotationErrChange<0.1){
        break;
      }
      else{
        avgRotationErrChange = 0;
      }
    }*/

    lastRotationErr = rotationErr;
    currRotation = inertialSens.get_rotation();
    rotationErr = currRotation - goalRotation;
    avgRotationErrChange += (rotationErr-lastRotationErr);
    //std::cout<<rotationErr<<"\n";
    loopCount+=1;
    pros::delay(loopDelay);
  }

  
  leftFrontMotor.moveVelocity(0);
  leftMiddleMotor.moveVelocity(0);
  rightFrontMotor.moveVelocity(0);
  rightMiddleMotor.moveVelocity(0);
  leftBackMotor.moveVelocity(0);
  rightBackMotor.moveVelocity(0);
}

struct Position getCurrXangY() {
  struct Position current;
  pros::c::gps_status_s_t currentStatus = gpsSens.get_status();
  current.x = currentStatus.x;
  current.y = currentStatus.y;
  return current;
}

struct Position getCurrXandY(double currAngl) {
  struct Position current;
  pros::c::gps_status_s_t currentStatus = gpsSens.get_status();
  current.x = currentStatus.x;
  current.y = currentStatus.y;
  double xOffset = 0.15;
  double yOffset = 0.13;
  double alpha = atan(yOffset/xOffset);
  alpha = radiansToDegrees(alpha);
  alpha = limitAngle(alpha);
  double dist = sqrt(xOffset*xOffset+yOffset*yOffset);
  double gpsx = current.x;
  double gpsy = current.y;
  //std::cout<<"Curr y = "<<current.y<<"\n";
  double radianDifference =  degreesToRadians(alpha-currAngl);
  current.x = gpsx+dist*cos(radianDifference);
  current.y = gpsy+dist*sin(radianDifference);
  /*std::cout<<"dist = "<<dist<<"\n";
  std::cout<<"sin(radianDifference) = "<<sin(radianDifference)<<"\n";*/
  return current;
}

//keep angle between -PI and PI.
double limitAngleVTwo(double angle) {
    while (angle > 180 || angle <= -180) {
        while (angle > 180) {
            angle -= 2 * 180;
            pros::delay(10);
        }
        while (angle < -180) {
            angle += 2 * 180;
            pros::delay(10);
        }
        pros::delay(10);
    }
    return angle;
}
void turnToFacePosition(struct Position goal){
  std::cout<<"in function";
  double currentAngle = gpsSens.get_heading()+180;
  currentAngle = limitAngle(currentAngle);
  std::cout<<currentAngle;
  pros::delay(10);

  struct Position current = getCurrXandY(currentAngle);
  std::cout<<"current x position is:"<<current.x<<"\n";
  std::cout<<"current y position is:"<<current.y<<"\n";


  double headingToGoalPos = calcHeadingToGoalPos(current, goal);
  

  if (goal.reversed == true) {
    headingToGoalPos = limitAngle(headingToGoalPos + 180);
  }

  double angleErr = limitAngleVTwo(headingToGoalPos-currentAngle);
  std::cout<<angleErr;

  turnForDegrees(angleErr);

}

// goal position must be in meters with center of field being (0,0)... and
// clockwise positive heading...
void moveToPosition(struct Position goal) {

  okapi::AverageFilter<3> headingFilter;
  okapi::AverageFilter<3> xFilter;
  okapi::AverageFilter<3> yFilter;

  straightPID.setMaxOutput(12000);
  straightPID.setMinOutput(-12000);

  straightPID.reset();
  angleAdjustPID.reset();


  struct Position error;

  double distanceTravelled = 0;
  double currentAngle;
  double distanceToGoalPos = 0;

  int angleAdjustment = 0;
  int baseVoltage = 0;

  int x;

  double pastThreeErrorSum = 0;
  double errAverage = 0;
  double previousErrAverage = 0;
  double angleAdjustInput = 0;

  int loopCount = 0;
  int count = 0;

  bool distTooSmall = false;


  currentAngle = gpsSens.get_heading()+180;
  currentAngle = limitAngle(currentAngle);
  std::cout<<currentAngle;
  pros::delay(10);

  struct Position current = getCurrXandY(currentAngle);
  std::cout<<"current x position is:"<<current.x<<"\n";
  std::cout<<"current y position is:"<<current.y<<"\n";
  struct Position startOfDrive = current;

  double headingToGoalPos = calcHeadingToGoalPos(current, goal);
  

  if (goal.reversed == true) {
    headingToGoalPos = limitAngle(headingToGoalPos + 180);
  }

  std::cout<<"goal heading is"<<headingToGoalPos<<"\n";

  double initialDistance = distanceToPoint(current, goal);

  std::cout<<"initial distance is:"<<initialDistance<<'\n';

  if (goal.reversed == true) {
    initialDistance *= -1;
  }

  
  if (abs(initialDistance) < 0.5) {
    distTooSmall = true;
  }

  std::cout<<"initial heading is:"<<currentAngle<<'\n';
  error.angle = headingToGoalPos - currentAngle;
  std::cout<<"initial heading error is:"<<error.angle<<"\n";
  error.angle = limitAngleVTwo(error.angle);
  pros::delay(10);

  while (1) {
    if(loopCount<4){
      headingFilter.filter(currentAngle);
    }
    else{
      currentAngle = headingFilter.filter(currentAngle);
    }

    current = getCurrXandY(currentAngle);
    
    if(loopCount<4){
      xFilter.filter(current.x);
      yFilter.filter(current.y);
    }
    else{
      current.x = xFilter.filter(current.x);
      current.y = yFilter.filter(current.y);
    }

    headingToGoalPos = calcHeadingToGoalPos(current, goal);
    if (goal.reversed == true) {
      headingToGoalPos = limitAngle(headingToGoalPos + 180);
    }

    distanceTravelled = distanceToPoint(startOfDrive, current);
    if (goal.reversed == true) {
      distanceTravelled *= -1;
    }

    distanceToGoalPos = initialDistance - distanceTravelled;

    baseVoltage = straightPID.getOutput(-distanceToGoalPos);

    if (abs(distanceToGoalPos) > 0.1) {
      if (distTooSmall) {
        angleAdjustment = 0;
      } 
      else {
        angleAdjustment = angleAdjustPID.getOutput(error.angle) * baseVoltage *
                          abs(distanceToGoalPos / initialDistance);
      }

      pastThreeErrorSum += distanceToGoalPos;

      if (loopCount % 15 == 0) {
        errAverage = pastThreeErrorSum / 3;
        if (abs(previousErrAverage - errAverage) < 0.01){
          break;
        }

        previousErrAverage = errAverage;
        errAverage = 0;
        pastThreeErrorSum = 0;
      }
      loopCount += 1;
    } 
    else {
      angleAdjustment = 0;
      pastThreeErrorSum += distanceToGoalPos;
      if (loopCount % 15 == 0) {
        errAverage = pastThreeErrorSum / 3;
        if (abs(previousErrAverage - errAverage) < 0.01) {
          break;
        }
        previousErrAverage = errAverage;
        errAverage = 0;
        pastThreeErrorSum = 0;
      }
      loopCount += 1;
    }



    rightFrontMotor.moveVoltage(baseVoltage + angleAdjustment);
    rightMiddleMotor.moveVoltage(baseVoltage + angleAdjustment);
    rightBackMotor.moveVoltage(baseVoltage + angleAdjustment);
    leftFrontMotor.moveVoltage(baseVoltage - angleAdjustment);
    leftMiddleMotor.moveVoltage(baseVoltage - angleAdjustment);
    leftBackMotor.moveVoltage(baseVoltage - angleAdjustment);

    if (abs(distanceToGoalPos) < 0.08) {
      x += 1;
    } 
    else {
      x = 0;
    }

    if (x > 10) {
      break;
    }

    pros::delay(10);
    currentAngle = gpsSens.get_heading() + 180;
    currentAngle = limitAngle(currentAngle);
    error.angle = headingToGoalPos - currentAngle;
    error.angle = limitAngleVTwo(error.angle);
    pros::delay(10);
    count += 1;
  }
  std::cout<<"final angle is:"<<currentAngle<<"\n";
  std::cout<<"final angle error is:"<<error.angle<<"\n";
  std::cout<<"final x and y are: "<<current.x<<","<<current.y<<"\n";
  std::cout<<"final distance error is:"<<distanceToGoalPos<<"\n";
  std::cout<<"done\n";
  leftFrontMotor.moveVoltage(0);
  leftMiddleMotor.moveVoltage(0);
  leftBackMotor.moveVoltage(0);
  rightFrontMotor.moveVoltage(0);
  rightMiddleMotor.moveVoltage(0);
  rightBackMotor.moveVoltage(0);
}
