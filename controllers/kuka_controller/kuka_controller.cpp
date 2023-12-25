// File:          kuka_controller.cpp
// Date:          18 - 11 - 2023
// Description:   Kinematic Mechanum wheels , Path Tracking using PID controller and Fuzzy Logic
// Author:        masdikaaliman@gmail.com
// Modifications:

#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "icecream.hpp"
#include "vector"
// #include "def.h"
#include "Controller.h"
#include "webots/InertialUnit.hpp"
#include "webots/GPS.hpp"
#include "webots/Compass.hpp"

#define radius_wheel 5.0
#define a 22.8 // jarak roda dari titik pusat ke titik pusat roda (in x cordinate)
#define b 15.8 // jarak roda dari titik pusat ke titik pusat roda (in y cordinate)

#define TIME_STEP 0.01

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
enum LOGIC_TYPE
{
  OR,
  AND,
  NOT
};

enum SHAPE
{
  TRIANGLE,
  TRAPESIUM
};

struct membership_func
{
  double x1, x2, x3, x4, centeroid;
  int membership_shape;
  void make_shape(double x1_ = 0, double x2_ = 0, double x3_ = 0, double x4_ = 0, SHAPE shaped = TRIANGLE)
  {
    x1 = x1_;
    x2 = x2_;
    x3 = x3_;
    x4 = x4_;
    membership_shape = shaped;
    switch (shaped)
    {
    case TRIANGLE:
      centeroid = (x1 + x2 + x3) / 3.0;
      break;

    case TRAPESIUM:
      centeroid = (x1 + x2 + x3 + x4) / 4.0;
      break;
    default:
      break;
    }
  }
};

double calc_membership_func(membership_func func, double input)
{
  double output;

  switch (func.membership_shape)
  {
  case TRIANGLE:
    if ((input <= func.x1) || (input >= func.x3))
    {
      output = 0;
    }
    else if ((input >= func.x1) && (input <= func.x2))
    {
      output = (input - func.x1) / (func.x2 - func.x1);
    }
    else if ((input >= func.x2) && (input <= func.x3))
    {
      output = (func.x3 - input) / (func.x3 - func.x2);
    }
    else
    {
      output = 0;
    }
    break;

  case TRAPESIUM:
    if ((input <= func.x1) || (input >= func.x4))
    {
      output = 0;
    }
    else if ((input >= func.x1) && (input <= func.x2))
    {
      output = (input - func.x1) / (func.x2 - func.x1);
    }
    else if ((input >= func.x2) && (input <= func.x3))
    {
      output = 1;
    }
    else if ((input >= func.x3) && (input <= func.x4))
    {
      output = (func.x4 - input) / (func.x4 - func.x3);
    }
    else
    {
      output = 0;
    }
    break;
  default:
    break;
  }

  return output;
}

struct RuleBase
{
  double velocity_right, velocity_left;
  double valueFind;

  void define_rule(double in1, double in2, double centroidVelRight, double centroidVelLeft, LOGIC_TYPE logic)
  {
    switch (logic)
    {
    case OR:
      valueFind = max(in1, in2);
      break;

    case AND:
      valueFind = min(in1, in2);
      break;

    default:
      valueFind = 0;
      break;
    }
    velocity_left = centroidVelLeft;
    velocity_right = centroidVelRight;
  }
};

std::vector<Pose> createCircularTrajectory(double centerX, double centerY, double radius, int numPoints) {
    std::vector<Pose> trajectory;

    for (int i = 0; i <= numPoints; ++i) {
        double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(numPoints);
        Pose point;
        point.x = centerX + radius * std::cos(angle);
        point.y = centerY + radius * std::sin(angle);
        point.theta = atan2(point.y, point.x) * 180.0 / M_PI;
        trajectory.push_back(point);
    }

    return trajectory;
}

using namespace webots;

Motor *wheel[4];
PositionSensor *enc[4];

void setVelocity_Robot(motor_struct &outMotor);

// Method forward kinematic it will return velocity in each cordinate
Pose forwardKinematic(double venc1, double venc2, double venc3, double venc4);

// Method Invers Kinematic it will give velocity each wheel
motor_struct InversKinematic(Pose &targetVel);

const int maxNumberCord = 5000;
const int refresh_factor = 4.0;

void create_line(Node *root, Supervisor *super);

int main(int argc, char **argv)
{

  Supervisor *robot = new Supervisor();
  PID *control = new PID(TIME_STEP);
  InertialUnit *imu;
  GPS *gps_dev;
  Compass *compass_dev;

  int timeStep = (int)robot->getBasicTimeStep();
  timeStep *= refresh_factor;
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

  // Create Line Track
  Node *target_line = robot->getFromDef("YOUBOT");

  // Create Line
  create_line(robot->getRoot(), robot);

  // create_target_line();

  Node *trail_line_set = robot->getFromDef("TRACK_LINE");
  Field *coord_field = trail_line_set->getField("coord");

  Node *coordinate_node = coord_field->getSFNode();
  Field *pointField = coordinate_node->getField("point");
  Field *coord_index_field = trail_line_set->getField("coordIndex");

  int index = 0;
  bool first_step = true;

  std::vector<double> pulse = {0, 0, 0, 0};
  std::vector<double> prev_pulse = {0, 0, 0, 0};
  std::vector<double> Venc = {0, 0, 0, 0}; // velocity translation in encoder

  // variable in use for robot
  Pose targetVel = {0, 5, 0};
  int count = 0;
  std::vector<Pose> targetPos = {Pose(3, 0, 0), Pose(3, 3, 0), Pose(0, -2, 0), Pose(-2, 1, 0)};
  // Pose targetPos(0, 0, 90);
  Pose pos_robot;
  Pose pos_global_robot;
  motor_struct outputMotor;

  // control->set_max_output(100);
  // control->setParam_PID(0.5, 0.1, 0.5);
  // control->setParam_PID(0.5, 0, 0);

  // fuzzy variable
  // Distance Function Struct
  membership_func VS_dist, S_dist, M_dist, B_dist, VB_dist;
  // Make A shape function Distance
  VS_dist.make_shape(-1.25, -1.25, 0, 1.25, TRAPESIUM);
  S_dist.make_shape(0, 1.25, 2.5, TRIANGLE);
  M_dist.make_shape(1.25, 2.5, 3.75, TRIANGLE);
  B_dist.make_shape(2.5, 3.75, 5.0, TRIANGLE);
  VB_dist.make_shape(3.75, 5.0, 10.0, 10.0, TRAPESIUM);

  // angle function struct
  membership_func NB_ang, NM_ang, NS_ang, Z, PS_ang, PM_ang, PB_ang;
  NB_ang.make_shape(-300, 0, -180, -120, TRAPESIUM);
  NM_ang.make_shape(-180, -120, -60, TRIANGLE);
  NS_ang.make_shape(-120, -60, 0, TRIANGLE);
  Z.make_shape(-60, 0, 60, TRIANGLE);
  PS_ang.make_shape(0, 60, 120, TRIANGLE);
  PM_ang.make_shape(60, 120, 180, TRIANGLE);
  PB_ang.make_shape(120, 180, 300, 300, TRAPESIUM);

  membership_func Z_Vel, F_Vel, M_Vel, B_Vel, VB_Vel, N_Vel;
  // Z_Vel.make_shape(0, 0, 0, TRIANGLE);
  // N_Vel.make_shape(0, 10, 20, TRIANGLE);
  // F_Vel.make_shape(20, 30, 40, TRIANGLE);
  // M_Vel.make_shape(30, 40, 50, TRIANGLE);
  // B_Vel.make_shape(40, 50, 60, TRIANGLE);
  // VB_Vel.make_shape(50, 60, 70, TRIANGLE);
//Using Singleton 
  Z_Vel.centeroid = 0;
  N_Vel.centeroid = 15;
  F_Vel.centeroid = 30;
  M_Vel.centeroid = 40;
  B_Vel.centeroid = 50;
  VB_Vel.centeroid = 70;

  std::vector<RuleBase> fuzzyRule(35);

  double VS_value, S_value, M_value, B_value, VB_value;
  double NB_value, NM_value, NS_value, Z_value, PS_value, PM_value, PB_value;

  double final_compute = 0;

  double right_vel_numerator, left_vel_numerator, right_vel_denominator, left_vel_denominator;
  double vel_right, vel_left;

  std::vector<Pose> circTraj = createCircularTrajectory(0,0,1.0, 3);

  // for (int i = 0; i < 370; i += 10)
  // {
  //   Pose point;
  //   point.x = 0 + cos((float)i * M_PI / 180.0);
  //   point.y = 0 + sin((float)i * M_PI / 180.0);
  //   point.theta = atan2(point.y, point.x) * 180.0 / M_PI;
  //   circTraj.push_back(point);
  // }

  while (robot->step(timeStep) != -1)
  {
    // IMU acces
    const double *imu_val = imu->getRollPitchYaw();
    double yaw = imu_val[2];

    const double *robotTranslations = target_line->getPosition();

    pointField->setMFVec3f(index, robotTranslations);

    // // update line track
    if (index > 0)
    {
      coord_index_field->setMFInt32(3 * (index - 1), index - 1);
      coord_index_field->setMFInt32(3 * (index - 1) + 1, index);
    }
    else if (index == 0 && first_step == false)
    {
      coord_index_field->setMFInt32(3 * (maxNumberCord - 1), 0);
      coord_index_field->setMFInt32(3 * (maxNumberCord - 1) + 1, (maxNumberCord - 1));
    }

    // float errX = (targetPos[count].x - pos_robot.x);
    // float errY = (targetPos[count].y - pos_robot.y);
    // double error = control->get_distance(errX, errY);
    // double errorTheta = ((atan2((errY), (errX)) * 180) / M_PI) - (pos_robot.theta * 180 / M_PI);
    // double errorTheta = targetPos[count].theta - (pos_robot.theta * 180 / M_PI);

    float errX = (circTraj[count].x - pos_robot.x);
    float errY = (circTraj[count].y - pos_robot.y);
    double error = control->get_distance(errX, errY);
    double errorTheta = circTraj[count].theta - (pos_robot.theta * 180 / M_PI);

    while (errorTheta > 180)
      errorTheta -= 360;
    while (errorTheta < -180)
      errorTheta += 360;

    // errorTheta = 0;
    fprintf(stderr, "target Posisi X: %.2f\ntarget Posisi Y: %.2f\ntarget Posisi theta: %.2f\n", targetPos[count].x, targetPos[count].y, targetPos[count].theta);
    IC(errorTheta);

    // targetVel = control->PID_calculate(targetPos[count], pos_robot);

    // Fuzzy area
    VS_value = calc_membership_func(VS_dist, error);
    S_value = calc_membership_func(S_dist, error);
    M_value = calc_membership_func(M_dist, error);
    B_value = calc_membership_func(B_dist, error);
    VB_value = calc_membership_func(VB_dist, error);

    NB_value = calc_membership_func(NB_ang, errorTheta);
    NM_value = calc_membership_func(NM_ang, errorTheta);
    NS_value = calc_membership_func(NS_ang, errorTheta);
    Z_value = calc_membership_func(Z, errorTheta);
    PS_value = calc_membership_func(PS_ang, errorTheta);
    PM_value = calc_membership_func(PM_ang, errorTheta);
    PB_value = calc_membership_func(PB_ang, errorTheta);

    IC(VS_value, S_value, M_value, B_value, VB_value);
    IC(NB_value, NM_value, NS_value, Z_value, PS_value, PM_value, PB_value);
    IC(Z_Vel.centeroid, N_Vel.centeroid, F_Vel.centeroid, M_Vel.centeroid, B_Vel.centeroid, VB_Vel.centeroid);
    // Rule Fuzzy
    fuzzyRule[0].define_rule(VS_value, NB_value, B_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[1].define_rule(VS_value, NM_value, M_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[2].define_rule(VS_value, NS_value, F_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[3].define_rule(VS_value, Z_value, N_Vel.centeroid, N_Vel.centeroid, AND);
    fuzzyRule[4].define_rule(VS_value, PS_value, Z_Vel.centeroid, N_Vel.centeroid, AND);
    fuzzyRule[5].define_rule(VS_value, PM_value, Z_Vel.centeroid, N_Vel.centeroid, AND);
    fuzzyRule[6].define_rule(VS_value, PB_value, Z_Vel.centeroid, M_Vel.centeroid, AND);

    fuzzyRule[7].define_rule(S_value, NB_value, VB_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[8].define_rule(S_value, NM_value, B_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[9].define_rule(S_value, NS_value, M_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[10].define_rule(S_value, Z_value, F_Vel.centeroid, F_Vel.centeroid, AND);
    fuzzyRule[11].define_rule(S_value, PS_value, Z_Vel.centeroid, M_Vel.centeroid, AND);
    fuzzyRule[12].define_rule(S_value, PM_value, Z_Vel.centeroid, B_Vel.centeroid, AND);
    fuzzyRule[13].define_rule(S_value, PB_value, Z_Vel.centeroid, VB_Vel.centeroid, AND);

    fuzzyRule[14].define_rule(M_value, NB_value, VB_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[15].define_rule(M_value, NM_value, VB_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[16].define_rule(M_value, NS_value, B_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[17].define_rule(M_value, Z_value, M_Vel.centeroid, M_Vel.centeroid, AND);
    fuzzyRule[18].define_rule(M_value, PS_value, Z_Vel.centeroid, B_Vel.centeroid, AND);
    fuzzyRule[19].define_rule(M_value, PM_value, Z_Vel.centeroid, VB_Vel.centeroid, AND);
    fuzzyRule[20].define_rule(M_value, PB_value, Z_Vel.centeroid, VB_Vel.centeroid, AND);

    fuzzyRule[21].define_rule(B_value, NB_value, VB_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[22].define_rule(B_value, NM_value, VB_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[23].define_rule(B_value, NS_value, VB_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[24].define_rule(B_value, Z_value, B_Vel.centeroid, B_Vel.centeroid, AND);
    fuzzyRule[25].define_rule(B_value, PS_value, Z_Vel.centeroid, VB_Vel.centeroid, AND);
    fuzzyRule[26].define_rule(B_value, PM_value, Z_Vel.centeroid, VB_Vel.centeroid, AND);
    fuzzyRule[27].define_rule(B_value, PB_value, Z_Vel.centeroid, VB_Vel.centeroid, AND);

    fuzzyRule[28].define_rule(VB_value, NB_value, VB_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[29].define_rule(VB_value, NM_value, VB_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[30].define_rule(VB_value, NS_value, VB_Vel.centeroid, Z_Vel.centeroid, AND);
    fuzzyRule[31].define_rule(VB_value, Z_value, VB_dist.centeroid, VB_Vel.centeroid, AND);
    fuzzyRule[32].define_rule(VB_value, PS_value, Z_Vel.centeroid, VB_Vel.centeroid, AND);
    fuzzyRule[33].define_rule(VB_value, PM_value, Z_Vel.centeroid, VB_Vel.centeroid, AND);
    fuzzyRule[34].define_rule(VB_value, PB_value, Z_Vel.centeroid, VB_Vel.centeroid, AND);

    left_vel_numerator = 0;
    left_vel_denominator = 0;
    right_vel_numerator = 0;
    right_vel_denominator = 0;

    // Defuzzyfication
    for (int i = 0; i < fuzzyRule.size(); i++)
    {
      right_vel_numerator += (fuzzyRule[i].valueFind) * fuzzyRule[i].velocity_right;
      right_vel_denominator += (fuzzyRule[i].valueFind);
      left_vel_numerator += (fuzzyRule[i].valueFind) * fuzzyRule[i].velocity_left;
      left_vel_denominator += (fuzzyRule[i].valueFind);
    }

    // ouput Fuzzy
    vel_left = left_vel_numerator / left_vel_denominator;
    vel_right = right_vel_numerator / right_vel_denominator;

    targetVel.x = ((vel_left + vel_right) / 2.0) * cos(atan2(errY, errX));
    targetVel.y = ((vel_left + vel_right) / 2.0) * sin(atan2(errY, errX));
    targetVel.theta = ((vel_left - vel_right) / (2.0 * a));
    IC(targetVel.x, targetVel.y, targetVel.theta);

    Pose vel_local;
    vel_local.x = cos(yaw) * targetVel.x + sin(yaw) * targetVel.y;
    vel_local.y = -sin(yaw) * targetVel.x + cos(yaw) * targetVel.y;
    vel_local.theta = targetVel.theta < -2.0 ? -2.0 : (targetVel.theta > 2.0 ? 2.0 : targetVel.theta);
    IC(vel_local.x, vel_local.y, vel_local.theta);

    if (error < 0.03 && fabs(errorTheta) < 20.0)
    {
      IC("next");
      // control->reset();
      count++;
    }

    // Do Inverskinematic to get each wheel vel
    outputMotor = InversKinematic(vel_local);

    // if (count > targetPos.size() - 1)
    // {
    //   IC("stop");
    //   outputMotor.w1_ = 0;
    //   outputMotor.w2_ = 0;
    //   outputMotor.w3_ = 0;
    //   outputMotor.w4_ = 0;
    // }

   if (count > circTraj.size() - 1)
    {
      IC("stop");
      outputMotor.w1_ = 0;
      outputMotor.w2_ = 0;
      outputMotor.w3_ = 0;
      outputMotor.w4_ = 0;
    }


    // IC(outputMotor.w1_, outputMotor.w2_, outputMotor.w3_, outputMotor.w4_);
    setVelocity_Robot(outputMotor);

    for (int i = 0; i < 4; i++)
    {
      pulse[i] = enc[i]->getValue();
      double tick = (pulse[i] - prev_pulse[i]);
      Venc[i] = tick; // Get Encoder pulse
    }
    // IC(Venc[0], Venc[1], Venc[2], Venc[3]);

    // do Calculate Odometry
    pos_global_robot = forwardKinematic(Venc[0], Venc[1], Venc[2], Venc[3]); // speed each cycle
    // IC(pos_global_robot.x, pos_global_robot.y, pos_global_robot.theta);

    Pose Vel_global;
    Vel_global.x = cos(yaw) * pos_global_robot.x - sin(yaw) * pos_global_robot.y;
    Vel_global.y = sin(yaw) * pos_global_robot.x + cos(yaw) * pos_global_robot.y;
    // IC(Vel_global.x, Vel_global.y, pos_global_robot.theta);

    pos_robot.x += Vel_global.x / 100;
    pos_robot.y += Vel_global.y / 100;
    pos_robot.theta = yaw;

    fprintf(stderr, "posisi robot X: %.2f\nposisi robot Y: %.2f\nposisi robot theta: %.2f\n", pos_robot.x, pos_robot.y, pos_robot.theta);
    // IC(pos_robot.x, pos_robot.y, pos_robot.theta);

    // unset next indices
    coord_index_field->setMFInt32(3 * index, index);
    coord_index_field->setMFInt32(3 * index + 1, index);

    if (robot->step(robot->getBasicTimeStep()) == -1)
      break;
    first_step = false;
    index++;
    index = index % maxNumberCord;

    prev_pulse = pulse;
  };
  robot->simulationQuit(EXIT_SUCCESS);
  // Cleanup a code

  delete robot;
  delete control;

  return 0;
}

void setVelocity_Robot(motor_struct &outMotor)
{
  wheel[0]->setVelocity(outMotor.w1_);
  wheel[1]->setVelocity(outMotor.w2_);
  wheel[2]->setVelocity(outMotor.w3_);
  wheel[3]->setVelocity(outMotor.w4_);
}

motor_struct InversKinematic(Pose &targetVel)
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
  // Forward Kinematic equation
  //  |xdot|            |  1          1        1        1   |     |w1|
  //  |ydot|  =  r/4  * |  1         -1       -1        1   |  *  |w2|
  //  |zdot|            |1/(a+b)  -1/(a+b)  1/(a+b) -1/(a+b)|     |w3|
  //                    |                                   |     |w4|
  Pose velocity;
  velocity.x = (radius_wheel / 4.0) * (venc1 + venc2 + venc3 + venc4);
  velocity.y = (radius_wheel / 4.0) * (venc1 - venc2 - venc3 + venc4);
  velocity.theta = (radius_wheel / (4.0 * (a + b))) * (venc1 - venc2 + venc3 - venc4);

  return velocity;
}

void create_line(Node *root, Supervisor *super)
{
  Node *existing_line = super->getFromDef("TRACK_ROBOT");
  // Node *myRobot = super->getFromDef("OMNI_WHEELS");
  // Field *translation_field = myRobot->getField("translation");

  if (existing_line)
    existing_line->remove();

  int i;
  std::string track_string = ""; // Initialize a big string which will contain the TRAIL node.
  // Create the TRAIL Shape.
  track_string += "DEF TRACK_ROBOT Shape {\n";
  track_string += "  appearance Appearance {\n";
  track_string += "    material Material {\n";
  track_string += "      diffuseColor 0 0 0\n";
  track_string += "      emissiveColor 0 0 0\n";
  track_string += "    }\n";
  track_string += "  }\n";
  track_string += "  geometry DEF TRACK_LINE IndexedLineSet {\n";
  track_string += "    coord Coordinate {\n";
  track_string += "      point [\n";
  for (i = 0; i < maxNumberCord; ++i)
    track_string += "      0 0 0\n";
  track_string += "      ]\n";
  track_string += "    }\n";
  track_string += "    coordIndex [\n";
  for (i = 0; i < maxNumberCord; ++i)
    track_string += "      0 0 -1\n";
  track_string += "    ]\n";
  track_string += "  }\n";
  track_string += "}\n";

  // Import TRAIL and append it as the world root nodes.

  Field *field_children = root->getField("children");

  field_children->importMFNodeFromString(-1, track_string);
}
