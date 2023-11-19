#ifndef DEF_H
#define DEF_H



struct Pose{
    Pose():x(0),y(0), theta(0){}
    Pose(float x, float y , float theta):x(x), y(y), theta(theta){}
    float x , y , theta;
};

struct motor_struct{
    motor_struct(){}
    motor_struct(float w1, float w2, float w3, float w4):w1_(w1), w2_(w2), w3_(w3), w4_(w4){}
    float w1_, w2_, w3_, w4_;
};



#endif