#ifndef SIMULATION_FLY_H
#define SIMULATION_FLY_H

#include <QObject>
#include <QVector>
#include <QMatrix3x3>
#include <QVector3D>
#include <qmath.h>
#include <Eigen/Dense>
#include <quaternion.h>

#define pow2(a) ((a) * (a))
#define DEGTORAD 0.0174532925199432957
#define RADTODEG 57.295779513082320876
#define HALFPI 1.5707963267948966192313216916398
#define PI 3.1415926535897932384626433832795
#define TWOPI 6.283185307179586476925286766559

typedef struct
{
  float SimulationStep = 0.01f;
  float simulationTotalTome = 1000.0f;
  float raelTimeFactor = 1.0f;
  double mass = 0.063;
  double motorTrustCoef = 3.9865 * pow(10, -8);
  double motorResistCoef = 7.5 * pow(10, -7);
  double lengthOfFlayerArms = 0.17;
  float numberofRotors = 4.0f;

  double Ixx = 5.828570  * pow(10, -5);
  double Iyy = 7.169140  * pow(10, -5);
  double Izz = 0.0001                 ;

  double maxVelocityRotor = 2631.0f;
  double minVelocityRotor = 0.0f;

} init_vec;

typedef struct
{

  Eigen::Vector3d angular_position = {0.0, 0.0, 0.0};

  Eigen::Vector3d angular_velocity = {0.0, 0.0, 0.0};

  Eigen::Vector3d angular_acceleration = {0.0, 0.0, 0.0};

  Eigen::Vector3d position = {0.0, 0.0, 0.0};

  Eigen::Vector3d velocity = {0.0, 0.0, 0.0};

  Eigen::Vector3d acceleration = {0.0, 0.0, 0.0};

  imu::Quaternion q_angular_position = {1.0, 0.0, 0.0, 0.0};

} state_vec;

typedef struct
{
float variable_1 = 0.0f;
float variable_2 = 0.0f;
} vec_2;

typedef struct
{
float variable_1 = 0.0f;
float variable_2 = 0.0f;
float variable_3 = 0.0f;
float variable_4 = 0.0f;
} vec_4;

class Simulation_fly
{
public:
    Simulation_fly();

   state_vec Dynemic_model (state_vec s_data, vec_4 r_data);

   state_vec calculateStateVector(vec_4 r_data, float dt);

   Eigen::Matrix3d rotation_from_euler(double roll, double pitch, double yaw);

   Eigen::Vector3d Quaternion_rotate(imu::Quaternion q, Eigen::Vector3d v);

   vec_2 rk4 (float accel, int i, float dt);

private:

   init_vec i_data;

   float buffer[6][12];

public:

   state_vec data;
};

#endif // SIMULATION_FLY_H
