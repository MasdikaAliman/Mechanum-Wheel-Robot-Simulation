// File:          my_controller.cpp
// Date:
// Description:
// Author: masdikaaliman@gmail.com
// Modifications:

#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Motor.hpp>
#include "iostream"
#include "icecream.hpp"

#define MAX_SPEED 10
#define TIME_STEP 0.01 // 10ms

using namespace webots;

float e_integral, e_prev;

double PID_control(float error, float kp, float ki, float kd)
{
  double output;

  float proportional = kp * error;
  float Integral = ki * e_integral;
  float Derivative = kd * (error - e_prev) / TIME_STEP;

  if (e_integral > MAX_SPEED)
    e_integral = MAX_SPEED;
  else if (e_integral < -MAX_SPEED)
    e_integral = -MAX_SPEED;
  else
    e_integral += error * TIME_STEP;

  output = proportional + Integral + Derivative;

  if (output > MAX_SPEED)
    output = MAX_SPEED;
  else if (output < -MAX_SPEED)
    output = -MAX_SPEED;

  e_prev = error;

  return double(output);
}

int main(int argc, char **argv)
{
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  char motor_device[6][20] = {"left wheel motor", "right wheel motor"};
  Motor *motor[2];

  for (int i = 0; i < 2; i++)
  {
    motor[i] = robot->getMotor(motor_device[i]);
    motor[i]->setPosition(INFINITY);
    motor[i]->setVelocity(0);
  }

  GPS *gps_device = robot->getGPS("gps");
  gps_device->enable(timeStep);

  // SET TARGET ROBOT;
  float targetX = 1.0;
  float targetY = 1.0;
  float targetTheta = 0;

  float speed_right = 0;
  float speed_left = 0;

  while (robot->step(timeStep) != -1)
  {
    const double *gps_val = gps_device->getValues();

    std::cout << "GPS val x: " << gps_val[0] << std::endl;
    std::cout << "GPS val y: " << gps_val[1] << std::endl;
    std::cout << "GPS val theta: " << gps_val[2] << std::endl;

    float errorX = targetX - gps_val[0];
    float errorY = targetY - gps_val[1];
    float errorTheta = targetTheta - (gps_val[2] * 180.0 / M_PI);

    if (errorTheta > 180)
      errorTheta -= 360;
    if (errorTheta < -180)
      errorTheta += 360;

    float error = sqrt(pow(errorX, 2) + pow(errorY, 2));

    double outputMotor = PID_control(error, 10, 0, 0.5);
    IC(outputMotor);
    if (error < 0.07)
    {
      speed_left = 0;
      speed_right = 0;
    }

    speed_left = cos(atan2(errorY, errorX)) * outputMotor;
    speed_right = cos(atan2(errorY, errorX)) * outputMotor;

    IC(speed_right);
    IC(speed_left);
    motor[0]->setVelocity(speed_left);
    motor[1]->setVelocity(speed_right);

  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
