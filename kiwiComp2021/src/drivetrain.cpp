#include "drivetrain.hpp"
#include "device_management.hpp"
#include "device_setup.hpp"
#include "okapi/api/filter/averageFilter.hpp"


const int loopDelay = 20;
  
KiwiPID turnPID(500,30,1000);//i was 20
KiwiPID turnPID2(500,40,1400);
KiwiPID straightPID(400,90,200);//40 integral was really good
KiwiPID angleAdjustPID((8.0/360.0),0,0);//going to coordinates
KiwiPID angleCorrectionPID(0.1,0,0);//going for a relative distance//was1/100(p)
KiwiPID stForwardPID(1500,150,2400);

//This function manages the PIDs at the start of the program.
//It is called in the initialize function in main.cpp
void setUpPIDs(){


  turnPID.setMaxOutput(12000);
  turnPID.setMinOutput(-12000);
  turnPID2.setMaxOutput(12000);
  turnPID2.setMinOutput(-12000);
  straightPID.setMaxOutput(12000);
  straightPID.setMinOutput(-12000);
  stForwardPID.setMaxOutput(12000);
  stForwardPID.setMinOutput(-12000);
  angleAdjustPID.setMaxOutput(2500);
  angleAdjustPID.setMinOutput(-2500);
  angleCorrectionPID.setMaxOutput(12000);
  angleCorrectionPID.setMinOutput(-12000);

  turnPID.setSetpoint(0);
  turnPID2.setSetpoint(0);
  straightPID.setSetpoint(0);
  angleAdjustPID.setSetpoint(0);
  stForwardPID.setSetpoint(0);
  angleCorrectionPID.setSetpoint(0);

  turnPID.setIMax(12000);
  turnPID.setIMin(0);
  turnPID.setMaxErrForI(5);
  turnPID.setDeadzone(0);
  turnPID2.setIMax(12000);
  turnPID2.setIMin(0);
  turnPID2.setMaxErrForI(5);
  turnPID2.setDeadzone(0);
  straightPID.setIMax(12000);
  straightPID.setIMin(0);
  straightPID.setMaxErrForI(2);
  straightPID.setDeadzone(0);
  angleAdjustPID.setIMax(6000);
  angleAdjustPID.setDeadzone(0);
  angleAdjustPID.setIMin(0);
  angleAdjustPID.setMaxErrForI(1000);
  angleCorrectionPID.setIMax(12000);
  angleCorrectionPID.setDeadzone(0);
  angleCorrectionPID.setIMin(0);
  angleCorrectionPID.setMaxErrForI(1000);
  stForwardPID.setIMax(12000);
  stForwardPID.setIMin(0);
  stForwardPID.setMaxErrForI(1);
  stForwardPID.setDeadzone(0);

}


//Never, ever, under any circumstances mess with this
//unless you change the wheel size of course
const double tireCircumference = 3.25*M_PI;


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
void moveForward(double targetDistance, bool hookBool){
  stForwardPID.reset();
  angleCorrectionPID.reset();
  //std::cout<<"loop count, heading error, heading correction, i output\n";
  pros::delay(20);
  int x = 0;

  int highestVelo = 0;
  int currVelo;
  int velo;
  int step = 3000;

  double goal = calcDriveDegrees(targetDistance);
  double distanceErr;
  double currEnc = 0;

  int leftVelo = velo;
  int rightVelo = velo;

  double iOutput;
  int loopCount = 0;

  int veloAdjust = 0;
  double initialInert = inertialSens.get_rotation();
  double currInert = initialInert;
  double headingErr = currInert - initialInert;
  angleCorrectionPID.setSetpoint(initialInert);
  std::cout<<"setpoint"<<angleCorrectionPID.getSetpoint()<<"\n";

  int sign;
  bool notSetAlready = true;
  bool notMaxed = true;

  if(targetDistance<0){
    step = -step;
  }

  leftFrontMotor.tarePosition();
  rightFrontMotor.tarePosition();
  leftBackMotor.tarePosition();
  rightBackMotor.tarePosition();

  pros::delay(loopDelay);

  while (1) {
    currEnc = (rightBackMotor.getPosition() + leftBackMotor.getPosition())/2.0;
    distanceErr = calcDriveDistance(goal - currEnc);

    if(abs(velo)<12000&&notMaxed){
      velo+=step;
    }
    else{
      notMaxed = false;
      velo = stForwardPID.getOutput(-distanceErr);
    }

    leftVelo = velo;
    rightVelo = velo;

    if(hookBool == true){
      sign = getSign(velo);
      if(sign == -1 && notSetAlready == true){
        hookPneum.set_value(true);
        notSetAlready = false;
        //std::cout<<"hook dropped at:"<<currEnc<<" which is this many loops:" <<loopCount<<'\n';
        break;
      }
    }

    currInert = inertialSens.get_rotation();

    headingErr = currInert - initialInert;

    veloAdjust = angleCorrectionPID.getOutput(currInert)*abs(velo);
    if(abs(veloAdjust)>abs(0.5*velo)){
      veloAdjust = 0.5*abs(velo)*getSign(veloAdjust);
    }

    leftVelo+=veloAdjust;
    rightVelo-=veloAdjust;  

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

  std::cout<<"final heading:"<<currInert<<"\n";
  //std::cout<<"finished at:"<<currEnc<<" which is this many loops:" <<loopCount<<'\n'<<"or this many inches:"<< calcDriveDistance(currEnc)<<'\n';
  leftFrontMotor.moveVelocity(0);
  leftMiddleMotor.moveVelocity(0);
  rightMiddleMotor.moveVelocity(0);
  rightFrontMotor.moveVelocity(0);
  leftBackMotor.moveVelocity(0);
  rightBackMotor.moveVelocity(0);
}

void liftMove(){//just call this function to start the lift
  fourBar.moveToAngle();
}

//a more complicated move forward function 
//which can drop the hook and move the lift asynchronously
void moveForwardTest(double targetDistance, bool hookBool, double distanceForArmMotion){
  stForwardPID.reset();
  angleCorrectionPID.reset();
  pros::delay(20);

  int x = 0;
  int velo;
  int step = 3000;

  double goal = calcDriveDegrees(targetDistance);
  double moveArmHere = calcDriveDegrees(distanceForArmMotion);
  double distanceErr;
  double currEnc = 0;
  
  int loopCount = 0;

  int leftVelo = velo;
  int rightVelo = velo;

  int veloAdjust = 0;
  double initialInert = inertialSens.get_rotation();
  double currInert = initialInert;
  double headingErr = currInert - initialInert;
  angleCorrectionPID.setSetpoint(initialInert);

  int sign;
  int goalProximity;
  int goalInRange = 255;
  int inRangeCount = 0;

  bool notSetAlready = true;
  bool armNotToldToMove = true;
  bool notMaxed = true;

  if(targetDistance<0){
    step = -step;
  }

  leftFrontMotor.tarePosition();
  rightFrontMotor.tarePosition();
  leftBackMotor.tarePosition();
  rightBackMotor.tarePosition();

  pros::delay(loopDelay);

  while (1) {
    currEnc = (rightBackMotor.getPosition() + leftBackMotor.getPosition())/2.0;
    distanceErr = calcDriveDistance(goal - currEnc);

    //if the arm hasn't been told to move and the robot is within ten encoder ticks
    //of the distance where it's supposed to start moving the lift, start the task which
    //will move the lift while the robot is moving
    if(abs(currEnc-moveArmHere)<10 && armNotToldToMove){
      pros::Task liftMotionTask(liftMove);
      armNotToldToMove = false;
      //std::cout<<"told arm to move!"<<"\n";
    }

    if(abs(velo)<12000&&notMaxed){
      velo+=step;
    }
    else{
      notMaxed = false;
      velo = stForwardPID.getOutput(-distanceErr);
    }

    leftVelo = velo;
    rightVelo = velo;

    if(hookBool == true){
      goalProximity = opticalSens.get_proximity();
      if(goalProximity>=goalInRange&&notSetAlready){
        hookPneum.set_value(false);
        notSetAlready = false;
        //std::cout<<"hook dropped at:"<<goalProximity<<"after this many loops:" <<loopCount<<'\n';
        break;
      }
    }

    currInert = inertialSens.get_rotation();
    headingErr = currInert - initialInert;
    veloAdjust = angleCorrectionPID.getOutput(currInert)*abs(velo);

    if(abs(veloAdjust)>abs(0.5*velo)){
      veloAdjust = 0.5*abs(velo)*getSign(veloAdjust);
    }

    leftVelo+=veloAdjust;
    rightVelo-=veloAdjust;

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

  std::cout<<"finished at:"<<currEnc<<" which is this many loops:" <<loopCount<<'\n'<<"or this many inches:"<< calcDriveDistance(currEnc)<<'\n';
  leftFrontMotor.moveVelocity(0);
  leftMiddleMotor.moveVelocity(0);
  rightMiddleMotor.moveVelocity(0);
  rightFrontMotor.moveVelocity(0);
  leftBackMotor.moveVelocity(0);
  rightBackMotor.moveVelocity(0);
}

//move forward and reach the goal
//at a speed that isn't the maximum
void moveForwardCoast(double targetDistance, int veloc){
  stForwardPID.reset();
  angleCorrectionPID.reset();
  pros::delay(20);

  int x = 0;

  int highestVelo = 0;
  int currVelo;
  int velo;

  double goal = calcDriveDegrees(targetDistance);
  double distanceErr;
  double lastDistanceErr = 0;
  double currEnc = 0;

  int leftVelo = velo;
  int rightVelo = velo;

  double iOutput;
  int loopCount = 0;

  int veloAdjust = 0;
  double initialInert = inertialSens.get_rotation();
  double currInert = initialInert;
  double headingErr = currInert - initialInert;
  angleCorrectionPID.setSetpoint(initialInert);

  int sign;
  bool notSetAlready = true;

  leftFrontMotor.tarePosition();
  rightFrontMotor.tarePosition();
  leftBackMotor.tarePosition();
  rightBackMotor.tarePosition();

  pros::delay(loopDelay);

  while (1) {
    currEnc = (leftBackMotor.getPosition()+rightBackMotor.getPosition())/2.0;
    distanceErr = calcDriveDistance(goal - currEnc);

    velo = stForwardPID.getOutput(-distanceErr)*veloc/12000;
    leftVelo = velo;
    rightVelo = velo;
    
    currInert = inertialSens.get_rotation();
    headingErr = currInert - initialInert;
    veloAdjust = angleCorrectionPID.getOutput(currInert)*abs(velo);

    if(abs(veloAdjust)>abs(0.5*velo)){
      veloAdjust = 0.5*abs(velo)*getSign(veloAdjust);
    }

    leftVelo+=veloAdjust;
    rightVelo-=veloAdjust;
    
    leftFrontMotor.moveVoltage(leftVelo);
    leftMiddleMotor.moveVoltage(leftVelo);
    leftBackMotor.moveVoltage(leftVelo);
    rightFrontMotor.moveVoltage(rightVelo);
    rightMiddleMotor.moveVoltage(rightVelo);
    rightBackMotor.moveVoltage(rightVelo);
    
    if(abs(goal-currEnc)<10||abs(distanceErr-lastDistanceErr)<0.1){
      x+=1;
    }
    else{
      x=0;
    }

    if(x>10){
      break;
    }

    lastDistanceErr = distanceErr;
    pros::delay(loopDelay);
    loopCount +=1;
  }

  //std::cout<<"finished at:"<<currEnc<<" which is this many loops:" <<loopCount<<'\n'<<"or this many inches:"<< calcDriveDistance(currEnc)<<'\n';
  leftFrontMotor.moveVelocity(0);
  leftMiddleMotor.moveVelocity(0);
  rightMiddleMotor.moveVelocity(0);
  rightFrontMotor.moveVelocity(0);
  leftBackMotor.moveVelocity(0);
  rightBackMotor.moveVelocity(0);
}



// turn a certain number of degrees based on the inertial sensor's readings:
// make the value negative to make it turn the opposite direction
void turnForDegrees(double turnAngle) {
  //std::cout<<"loop count, rotation error, pid output\n";
  turnPID.reset();
  pros::delay(20);
  double currRotation = inertialSens.get_rotation();
  double goalRotation = currRotation + turnAngle;
  int leftVeloc;
  int rightVeloc;
  int veloc = 0;
  int x = 0;
  double lastRotationErr;
  double rotationErr = currRotation - goalRotation;
  double avgRotationErrChange;
  int loopCount = 0;

  while (1) {

    veloc = turnPID.getOutput(rotationErr);
    if(loopCount%2 == 0&&loopCount<200){
      //std::cout<<loopCount<<","<<rotationErr<<","<<veloc<<"\n";
    }
    
    leftVeloc = veloc;
    rightVeloc = -veloc;
    

    leftFrontMotor.moveVoltage(leftVeloc);
    leftMiddleMotor.moveVoltage(leftVeloc);
    leftBackMotor.moveVoltage(leftVeloc);
    rightFrontMotor.moveVoltage(rightVeloc);
    rightMiddleMotor.moveVoltage(rightVeloc);
    rightBackMotor.moveVoltage(rightVeloc);

    if (abs(rotationErr) < 1) {
      x += 1;
    }
    else{
      x = 0;
    }

    if (x >= 10) {
      break;
    }

    /*if(loopCount%20==0){
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

  //std::cout<<"loopcount:"<<loopCount<<"rotationErr:"<<rotationErr<<"\n";
  leftFrontMotor.moveVelocity(0);
  leftMiddleMotor.moveVelocity(0);
  rightFrontMotor.moveVelocity(0);
  rightMiddleMotor.moveVelocity(0);
  leftBackMotor.moveVelocity(0);
  rightBackMotor.moveVelocity(0);
}


//turn for a certain number of degrees based on IMU readings
//make the value negative to turn in the opposite direction
//this function is better than the original for small turns
//(different pid)
void turnForDegrees2(double turnAngle) {
  //std::cout<<"loop count, rotation error, pid output\n";
  turnPID2.reset();
  double currRotation = inertialSens.get_rotation();
  double goalRotation = currRotation + turnAngle;
  int leftVeloc;
  int rightVeloc;
  int veloc = 0;
  int x = 0;
  double lastRotationErr;
  double rotationErr = currRotation - goalRotation;
  double avgRotationErrChange;
  int loopCount = 0;

  while (1) {

    veloc = turnPID2.getOutput(rotationErr);
    if(loopCount%4 == 0&&loopCount<150){
      //std::cout<<loopCount<<","<<rotationErr<<","<<veloc<<"\n";
    }
    
    leftVeloc = veloc;
    rightVeloc = -veloc;
    

    leftFrontMotor.moveVoltage(leftVeloc);
    leftMiddleMotor.moveVoltage(leftVeloc);
    leftBackMotor.moveVoltage(leftVeloc);
    rightFrontMotor.moveVoltage(rightVeloc);
    rightMiddleMotor.moveVoltage(rightVeloc);
    rightBackMotor.moveVoltage(rightVeloc);

    if (abs(rotationErr) < 0.5) {
      x += 1;
    }
    else{
      x = 0;
    }

    if (x >= 10) {
      break;
      //std::cout<<"in the zone\n";
    }

    if(loopCount%20==0){
      avgRotationErrChange = avgRotationErrChange/10;
      if(avgRotationErrChange<0.1&&loopCount>60){
        //std::cout<<"got stuck\n";
        break;
      }
      else{
        avgRotationErrChange = 0;
      }
    }

    lastRotationErr = rotationErr;
    currRotation = inertialSens.get_rotation();
    rotationErr = currRotation - goalRotation;
    avgRotationErrChange += (rotationErr-lastRotationErr);
    //std::cout<<rotationErr<<"\n";
    loopCount+=1;
    pros::delay(loopDelay);
  }

  //std::cout<<"loopcount:"<<loopCount<<"rotationErr:"<<rotationErr<<"\n";
  leftFrontMotor.moveVelocity(0);
  leftMiddleMotor.moveVelocity(0);
  rightFrontMotor.moveVelocity(0);
  rightMiddleMotor.moveVelocity(0);
  leftBackMotor.moveVelocity(0);
  rightBackMotor.moveVelocity(0);
}

//get the raw gps position
struct Position getCurrXangY() {
  struct Position current;
  pros::c::gps_status_s_t currentStatus = gpsSens.get_status();
  current.x = currentStatus.x;
  current.y = currentStatus.y;
  return current;
}

//get the gps position translated to the "center" of the robot
struct Position getCurrXandY(double currAngl) {
  struct Position current;
  pros::c::gps_status_s_t currentStatus = gpsSens.get_status();
  current.x = currentStatus.x;
  current.y = currentStatus.y;
  double xOffset = 0.15;
  double yOffset = 0.088;
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

//turn to face the goal position on the field
void turnToFacePosition(struct Position goal){
  
  //std::cout<<"in function";
  double currentAngle = gpsSens.get_heading()+180;
  currentAngle = limitAngle(currentAngle);
  //std::cout<<currentAngle;
  pros::delay(10);

  struct Position current = getCurrXandY(currentAngle);
  //std::cout<<"current x position is:"<<current.x<<"\n";
  //std::cout<<"current y position is:"<<current.y<<"\n";


  double headingToGoalPos = calcHeadingToGoalPos(current, goal);
  

  if (goal.reversed == true) {
    headingToGoalPos = limitAngle(headingToGoalPos + 180);
  }

  double angleErr = limitAngleVTwo(headingToGoalPos-currentAngle);
  //std::cout<<angleErr;

  turnForDegrees2(angleErr);

}

// goal position must be in meters with center of field being (0,0)... and
// clockwise positive heading...
//move in a somewhat straight line towards a goal position
//if needed, lift the arm asynchronously
//after a certain distance has been traveled
void moveToPosition(struct Position goal, bool liftArm, double distanceToStartLift) {

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
  bool liftAr = liftArm;


  currentAngle = gpsSens.get_heading()+180;
  currentAngle = limitAngle(currentAngle);
  //std::cout<<currentAngle;
  pros::delay(10);

  struct Position current = getCurrXandY(currentAngle);

  
  
  //std::cout<<"current x position is:"<<current.x<<"\n";
  //std::cout<<"current y position is:"<<current.y<<"\n";
  struct Position startOfDrive = current;

  double headingToGoalPos = calcHeadingToGoalPos(current, goal);
  

  if (goal.reversed == true) {
    headingToGoalPos = limitAngle(headingToGoalPos + 180);
  }

  //std::cout<<"goal heading is"<<headingToGoalPos<<"\n";

  double initialDistance = distanceToPoint(current, goal);

  //std::cout<<"initial distance is:"<<initialDistance<<'\n';

  if (goal.reversed == true) {
    initialDistance *= -1;
  }

  
  if (abs(initialDistance) < 0.5) {
    distTooSmall = true;
  }

  //std::cout<<"initial heading is:"<<currentAngle<<'\n';
  error.angle = headingToGoalPos - currentAngle;
  //std::cout<<"initial heading error is:"<<error.angle<<"\n";
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

     if(abs(initialDistance - distanceToGoalPos)>= distanceToStartLift && liftAr){
      pros::Task liftMotionTask(liftMove);
      liftAr = false;
      //std::cout<<"told arm to move!"<<"\n";
    }

    baseVoltage = straightPID.getOutput((-distanceToGoalPos*39.37));

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

    if (abs(distanceToGoalPos) < 0.04) {//was 0.08
      x += 1;
      //std::cout<<"in zone at:"<<count<<"\n";
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
  /*std::cout<<"final angle is:"<<currentAngle<<"\n";
  std::cout<<"final angle error is:"<<error.angle<<"\n";
  std::cout<<"final x and y are: "<<current.x<<","<<current.y<<"\n";
  std::cout<<"final distance error is:"<<distanceToGoalPos<<"\n";
  std::cout<<"count is"<<count<<"\n";
  std::cout<<"done\n";*/
  leftFrontMotor.moveVoltage(0);
  leftMiddleMotor.moveVoltage(0);
  leftBackMotor.moveVoltage(0);
  rightFrontMotor.moveVoltage(0);
  rightMiddleMotor.moveVoltage(0);
  rightBackMotor.moveVoltage(0);
}
