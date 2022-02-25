#include "lift.hpp"
#include "pros/misc.hpp"
using namespace kiwiLift;

pros::Rotation liftEnc(19);
Lift fourBar(16.5,13);

Lift::Lift(double towerH, double armL){//instantiate a 4 bar lift with a given height and arm length
    init();
    liftMutex.take(200);
    towerHeight = towerH;
    armLength = armL;
    liftMutex.give();
}

void Lift::init(){
    liftMutex.take(200);
    armLength = 0;
    towerHeight = 0;
    currVolt = 0;
    goalVolt = 0;  
    goalAngle = 0;
    startLift = false;
    liftMutex.give();
}

void Lift::reset(){
    liftMutex.take(200);
    liftEnc.set_position(0);
    liftMutex.give();
}  


double Lift::getHeight(){
    double currH;
    double currAng = getAngle();
    currH = calcHeight(currAng);
    return currH;
}

void Lift::setGoalAngle(double gAng){
    liftMutex.take(200);
    goalAngle = gAng;
    //std::cout<<"goal angle is:"<<goalAngle<<"\n";
    liftMutex.give();
}

void Lift::setGoalVolt(double gVolt){
    liftMutex.take(200);
    goalVolt = gVolt;
    //std::cout<<"goal voltage is:"<<goalVolt<<"\n";
    liftMutex.give();
}

void Lift::setGoalAngleAndVolt(double goalA, double goalV){
    setGoalAngle(goalA);
    setGoalVolt(goalV);
}

void Lift::setToStart(){
    liftMutex.take(200);
    startLift = true;
    //std::cout<<"starting lift..."<<startLift<<"\n";
    liftMutex.give();
}

void Lift::move(){
   bool shouldMove = false;
   while(pros::competition::is_autonomous()){
   liftMutex.take(200);
   if(startLift == true){
       shouldMove = true;
   }
   else{
       shouldMove = false;
   }
   liftMutex.give();
   if(shouldMove){
       moveToAngle();
       shouldMove = false;
       liftMutex.take(200);
       startLift = false;
       liftMutex.give();
   }
   else{
   }

   pros::delay(20);
   }
}

double Lift::getAngle(){
    double currAngl;
    liftMutex.take(200);
    currAngl = liftEnc.get_angle()/(100.0);
    if(currAngl>180){
        currAngl-=360;
    }
    liftMutex.give();
    return currAngl;
}

double Lift::calcHeight(double theta){
    liftMutex.take(200);
    theta = degreesToRadians(theta);
    double height;
    if(theta<M_PI_2){
        height = towerHeight - cos(theta)*armLength;
    }
    else{
        theta-=M_PI_2;
        height = towerHeight + sin(theta)*armLength;
    }
    liftMutex.give();
    return height;

}

double Lift::calcAngle(double height){
    liftMutex.take(200);
    double theta;
    double x;
    if(height<towerHeight){
        x = towerHeight-height;
        theta = acos(x/armLength);
    }
    else{
        x = height - towerHeight;
        theta = asin(x/armLength)+M_PI_2;
    }
    //std::cout<<"theta radians:"<<theta<<"\n";
    theta = radiansToDegrees(theta);
    //std::cout<<"theta degrees:"<<theta<<"\n";
    liftMutex.give();
    return theta;
}

void Lift::moveToAngle(){
    if(gpsErrOut){
      liftMotor.moveVelocity(0);
      return;
    }

    double goalAng;
    double goalVoltage;
    double currentAngle = getAngle();
    double errAverage = 0;
    double lastErrAvg = 0;
    double err = 0;
    int loopC = 0;

    liftMutex.take(200);
    goalAng = goalAngle;
    goalVoltage = abs(goalVolt);
    liftMutex.give();

    double currentHeight = calcHeight(currentAngle);
    //std::cout<<"goal angle is:"<<goalAng<<"\n";
    //std::cout<<"current angle is:"<<currentAngle<<"\n";
    //std::cout<<"currentHeight is:"<<currentHeight<<"\n";

    while(1){

        if(currentAngle<goalAng){
            liftMotor.moveVoltage(goalVoltage);
        }
        if(currentAngle>goalAng){
            liftMotor.moveVoltage(-goalVoltage);
        }
   
        if(abs(currentAngle-goalAng)<2.5&&goalAng!=0){
            //std::cout<<"in range\n";
            //std::cout<<"final angle is:"<<currentAngle<<'\n';
            break;
        }
        
        if(limitSwitch.get_value()==true&&goalAng==0){
          break;
        }
      

        if(loopC%25 == 0 && loopC>25){
          errAverage = err/25;
          err = 0;
          //std::cout<<"errAverage is:"<<errAverage<<" and lastErrAvg is:"<<lastErrAvg<<"\n";
          if(abs(errAverage-lastErrAvg)<0.75){
            //std::cout<<"stuck and leaving loop\n";
            break;
          }
          else{
            lastErrAvg = errAverage;
          }
          //std::cout<<"not stuck\n";
        }

        err = err + abs(currentAngle - goalAng);
        loopC+=1;
        pros::delay(20);
        currentAngle = getAngle();
    }

    liftMotor.moveVelocity(0);
}