#include "RobotSteering.h"
#include <cmath>


RobotSteering::RobotSteering(){
    maxSpeed = 1;//do poprawy
    maxAng2forward = 0.26;//maksymalny kat, przy którym robot będzie jechał już do przodu
    acceleration = 0.3;//do poprawy
    posTolerance = 0.1;//do zabawy
    rotConst = 1;//do zabawy
}

float cropAngle(float a){
    while(a>M_PI){
        a-=2*M_PI;
    }
    while(a<-M_PI){
        a+=2*M_PI;
    }
    return a;
}


//Rot_dist is {angle between robot forward vector and vector (closest target position - robot position)} in radians
float RobotSteering::calculateAngularSpeed(float dist, float rot_dist){
    float speed;
    rot_dist = cropAngle(rot_dist);
    speed = (exp(rotConst*std::abs(rot_dist))-1);
    if(speed>1){
        speed = 1;
    }
    speed= copysign(speed, rot_dist);
    return speed;
}

float RobotSteering::calculateLinearSpeed(float dist, float rot_dist){
    float speed;
    rot_dist = cropAngle(rot_dist);
    rot_dist = std::abs(rot_dist);

    if(rot_dist<maxAng2forward&&rot_dist<1){
        speed = 1-rot_dist;
    }

    float act_speed = speed*maxSpeed;
    if(dist<act_speed*act_speed/acceleration || dist<posTolerance){
       speed = 0;
    }

    return speed;
}
