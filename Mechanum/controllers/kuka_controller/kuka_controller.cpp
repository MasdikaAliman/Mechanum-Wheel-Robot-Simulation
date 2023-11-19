// File:          kuka_controller.cpp
// Date:          18 - 11 - 2023
// Description:   Kinematic Mechanum wheels , Path Tracking using PID controller
// Author:        masdikaaliman@gmail.com
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "icecream.hpp"
#include "vector"
#include "def.h"
#include "webots/InertialUnit.hpp"

#define radius_wheel 5.0
#define a 22.8 // jarak roda dari titik pusat ke titik pusat roda (in x cordinate)
#define b 15.8 // jarak roda dari titik pusat ke titik pusat roda (in y cordinate)

using namespace webots;

Motor *wheel[4];
PositionSensor *enc[4];

void setVelocity_Robot(motor_struct &outMotor);

//Method forward kinematic it will return velocity in each cordinate
Pose forwardKinematic(double venc1, double venc2, double venc3, double venc4); 

//Method Invers Kinematic it will give velocity each wheel
motor_struct getVelocityBase(Pose &targetVel);

int main(int argc, char **argv)
{

  Robot *robot = new Robot();
  InertialUnit *imu;

  int timeStep = (int)robot->getBasicTimeStep();

  // name devices in Kuka robot
  char wheel_device[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  char encoder_device[4][8] = {"enc1", "enc2", "enc3", "enc4"};

  for (int i = 0; i < 4; i++)
  {
    // wheel set
    wheel[i] = robot->getMotor(wheel_device[i]);
    wheel[i]->setPosition(INFINITY); // set the rotation Wheel rad to INFINITY

    wheel[i]->setVelocity(0); // set Velocity 0 rad/s
  }

  for (int i = 0; i < 4; i++)
  {
    // encoder set
    enc[i] = robot->getPositionSensor(encoder_device[i]);
    enc[i]->enable(timeStep); // set time step;
  }

  imu = robot->getInertialUnit("IMU");
  imu->enable(timeStep);

  std::vector<double> pulse = {0, 0, 0, 0};
  std::vector<double> prev_pulse = {0, 0, 0, 0};
  std::vector<double> Venc = {0, 0, 0, 0}; // velocity translation in encoder

  // variable in use for robot
  Pose targetVel = {0,0, 1};
  Pose pos_robot;
  Pose pos_global_robot;
  motor_struct outputMotor;

  while (robot->step(timeStep) != -1)
  {
    //IMU acces
    const double *imu_val = imu->getRollPitchYaw();
    double yaw = imu_val[2];
    IC(yaw);

    Pose vel_local;
    vel_local.x = cos(yaw) * targetVel.x + sin(yaw) * targetVel.y;
    vel_local.y = -sin(yaw) * targetVel.x + cos(yaw) * targetVel.y;
    vel_local.theta = targetVel.theta;

    outputMotor = getVelocityBase(vel_local);
    IC(outputMotor.w1_, outputMotor.w2_, outputMotor.w3_, outputMotor.w4_);
    setVelocity_Robot(outputMotor);

    for (int i = 0; i < 4; i++)
    {

      pulse[i] = enc[i]->getValue();
      double tick = (pulse[i] - prev_pulse[i]);
      Venc[i] = tick; //Get Encoder pulse
    }
    IC(Venc[0],Venc[1],Venc[2],Venc[3]);

    pos_global_robot = forwardKinematic(Venc[0], Venc[1], Venc[2], Venc[3]);  //speed each cycle 
    
    pos_global_robot.x = cos(yaw) * pos_global_robot.x - sin(yaw) * pos_global_robot.y;
    pos_global_robot.y = sin(yaw) * pos_global_robot.x + cos(yaw) * pos_global_robot.y;

    pos_robot.x += pos_global_robot.x/100;
    pos_robot.y += pos_global_robot.y/100;
    pos_robot.theta += pos_global_robot.theta;

    IC(pos_robot.x, pos_robot.y, pos_robot.theta);

    prev_pulse = pulse;
  };

  // Cleanup a code

  delete robot;
  return 0;
}

void setVelocity_Robot(motor_struct &outMotor)
{
  wheel[0]->setVelocity(outMotor.w1_);
  wheel[1]->setVelocity(outMotor.w2_);
  wheel[2]->setVelocity(outMotor.w3_);
  wheel[3]->setVelocity(outMotor.w4_);
}

motor_struct getVelocityBase(Pose &targetVel)
{
  // Invers Kinematic equation
  //  |w1|           |1   1   a(+b)|   |Vx    |
  //  |w2|  = 1/r *  |1  -1  -a(+b)| * |Vy    |
  //  |w3|           |1  -1   a(+b)|   |Vtheta|
  //  |w4|           |1   1  -a(+b)|
  motor_struct outMotor;

  outMotor.w1_ = 1 / radius_wheel * (targetVel.x + targetVel.y + ((a + b) * targetVel.theta));
  outMotor.w2_ = 1 / radius_wheel * (targetVel.x - targetVel.y - ((a + b) * targetVel.theta));
  outMotor.w3_ = 1 / radius_wheel * (targetVel.x - targetVel.y + ((a + b) * targetVel.theta));
  outMotor.w4_ = 1 / radius_wheel * (targetVel.x + targetVel.y - ((a + b) * targetVel.theta));
  return outMotor;
}

Pose forwardKinematic(double venc1, double venc2, double venc3, double venc4)
{
  //Forward Kinematic equation
  // |xdot|            |  1          1        1        1   |     |w1|
  // |ydot|  =  r/4  * |  1         -1       -1        1   |  *  |w2|
  // |zdot|            |1/(a+b)  -1/(a+b)  1/(a+b) -1/(a+b)|     |w3|
  //                   |                                   |     |w4|
  Pose velocity;
  velocity.x = (radius_wheel / 4.0) * (venc1 + venc2 + venc3 + venc4);
  velocity.y = (radius_wheel / 4.0) * (venc1 - venc2 - venc3 + venc4);
  velocity.theta = (radius_wheel / (4.0 * (a + b))) * (venc1 - venc2 + venc3 - venc4);

  return velocity;
}
