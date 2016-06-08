#include "RobotSteering.h"
#include <cmath>
#include <iostream>

RobotSteering::RobotSteering(){
    maxSpeed = 1;//do poprawy
    maxAng2forward = 0.26;//maksymalny kat, przy którym robot będzie jechał już do przodu
    acceleration = 0.3;//do poprawy
    posTolerance = 0.1;//do zabawy
    rotConst = 0.5;//do zabawy
}

float cropAngle(float a){
    while(a>M_PI){
        a-=2*M_PI;
    }
    while(a<-M_PI){
        a+=2*M_PI;
    }
    std::cout<<"znormalizowany kat: "<<a<<std::endl;
    return a;
}


//Rot_dist is {angle between robot forward vector and vector (closest target position - robot position)} in radians
float RobotSteering::calculateAngularSpeed(float dist, float rot_dist){
    float speed;
    rot_dist = cropAngle(rot_dist);

    speed = (rotConst*std::abs(rot_dist));

    if(speed>1||rot_dist>3.f*maxAng2forward){
        speed = 1;
    }

    speed= copysign(speed, rot_dist);


    return speed;
}

float RobotSteering::calculateLinearSpeed(float dist, float rot_dist){
    float speed;
    rot_dist = cropAngle(rot_dist);
    rot_dist = std::abs(rot_dist);

    if(rot_dist<maxAng2forward){
        speed = (1-std::abs(rot_dist))*dist;
    }else{
        speed = 0;
    }
    if(speed>maxSpeed)
        speed = maxSpeed;
    if(/*dist<speed*speed/acceleration ||*/ dist<posTolerance){
       speed = 0;
    }
    if(speed<0)speed=0;
    if(speed<0.1&&speed>0.01)
        speed = 0.1;
    return speed;
}
