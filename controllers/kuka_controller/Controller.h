#ifndef CONTROL_H
#define CONTROL_H

#include "def.h"
#include "iostream"
class PID
{
public:
    PID(){};
    PID(float DTime) : dTime(DTime){};

    void setParam_PID(float kp_, float ki_, float kd_)
    {
        kp = kp_;
        ki = ki_;
        kd = kd_;
    };

    void set_max_output(float max = 15.0)
    {
        max_output = max;
    };
    // Return Velocity In X, Y, and Theta
    Pose PID_calculate(Pose &Targetpos, Pose &currentPosition);

    double PID_rotation(float error, float kp, float ki, float kd);

    double get_distance(float &a, float &b);
    double get_angle(float &a, float &b);

    double reset()
    {
        e_integral = e_integral_rot = e_prev_rot = 0;
    }

private:
    float kp, ki, kd, dTime;
    float e_integral, e_prev, e_integral_rot, e_prev_rot;
    float output_trans, output_rot;
    float max_output;
    Pose vel_new{0, 0, 0};
};

Pose PID::PID_calculate(Pose &targetPos, Pose &currentPos)
{
    Pose ret_;
    float errX = targetPos.x - currentPos.x;
    float errY = targetPos.y - currentPos.y;

    float errTheta = targetPos.theta - (currentPos.theta * 180.0 / M_PI);
    if (errTheta > 180)
        errTheta -= 360;
    if (errTheta < -180)
        errTheta += 360;

    double error = get_distance(errX, errY);
    // IC(errX, errY, error, errTheta);
    // IC(targetPos.x, targetPos.y, currentPos.x, currentPos.y);
    float propotional = kp * error;
    float integral = ki * e_integral;
    float derivative = kd * (error - e_prev) / dTime;

    double errorThetaTarget = get_angle(errX, errY);

    // IC(propotional, integral, derivative, errorThetaTarget);
    output_trans = propotional + integral + derivative;
    // if (error < 0.09)
    output_rot = PID_rotation((errTheta * M_PI / 180.0), 1.9, 0, 0.5);
    // else
    // output_rot = 0;
    // IC(output_trans, output_rot);
    if (e_integral > max_output)
        e_integral = max_output;
    else if (e_integral < -(max_output))
        e_integral = -max_output;
    else
        e_integral += error * dTime;

    if (output_trans > max_output)
        output_trans = max_output;
    else if (output_trans < -max_output)
        output_trans = -max_output;

    IC(ret_.x, ret_.y);

    ret_.x = sin(errorThetaTarget) * output_trans;
    ret_.y = cos(errorThetaTarget) * output_trans;
    ret_.theta = output_rot;

    // Step to get Vector from max velocity
    IC(ret_.x, ret_.y);

    // 1. Normalize vector
    double vel_magnitude = get_distance(targetPos.x,targetPos.y);
    // // 2. divedev with the magnitude
    double x = ret_.x / vel_magnitude;
    double y = ret_.y / vel_magnitude;

    // // // 3. if our new_vel is greated than max vel we want for example we want  80cm/s desired that with the new component x/y vel
    ret_.x = 25 * x;
    ret_.y = 25 * y;
    double new_vel_magnitude = get_distance(ret_.x, ret_.y);

    IC(vel_magnitude, x, y, new_vel_magnitude);
    IC(ret_.x, ret_.y);
    e_prev = error;

    // IC(ret_.theta, ret_.x, ret_.y, errorThetaTarget);
    fprintf(stderr, "error: %.2f\noutput X: %.2f \noutput Y: %.2f\noutput Theta: %.2f\n", error, ret_.x, ret_.y, ret_.theta);

    return ret_;
}

double PID::PID_rotation(float error, float kp, float ki, float kd)
{
    double out;
    // IC(error);
    out = error * kp + ki * e_integral_rot + kd * (error - e_prev_rot) / dTime;

    if (e_integral_rot > max_output)
        e_integral_rot = max_output;
    else if (e_integral_rot < -max_output)
        e_integral_rot = -max_output;
    else
        e_integral_rot += error * dTime;

    if (out > 10)
        out = max_output;
    e_prev_rot = error;

    return out;
}

double PID::get_distance(float &a, float &b)
{
    return sqrt(a * a + b * b);
}

double PID::get_angle(float &a, float &b)
{
    return atan2(a, b);
}

#endif