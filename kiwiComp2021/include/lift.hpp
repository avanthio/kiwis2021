#pragma once
#include "main.h"
namespace kiwiLift{
    class Lift{
    public:

        Lift(double towerHeigh,double armLengt);//instance of a  4 bar lift with a given tower height/arm length
        void reset();//reset the encoder for the lift
        void init();//initialize all variables for the lift

        double getHeight();//get the current height of the lift
        void setGoalAngle(double);//set the goal height of the lift
        void setGoalVolt(double);//set the goal voltage of the lift
        void setGoalAngleAndVolt(double Gheight, double Gvoltage);//set both the goal height and the goal voltage of the lift
        void moveToAngle();//move the lift to its goal height at its goal voltage
        double getAngle();//get the current angle of the lift
        
    private:
        double armLength;
        double towerHeight;
        double currVolt;
        double goalVolt;  
        double goalAngle;
        bool startLift;   
        double calcHeight(double);//calculate the height of the lift based on its current angle
        double calcAngle(double);//calculate the angle of the lift based on its current height
        pros::Mutex liftMutex;//threadsafety - use when accessing any lift variables/encoders :)
    };
}

extern kiwiLift::Lift fourBar;