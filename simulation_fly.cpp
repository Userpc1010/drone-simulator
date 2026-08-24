#include "simulation_fly.h"
#include <QDebug>

Simulation_fly::Simulation_fly()
{

}

state_vec Simulation_fly::Dynemic_model(state_vec s_data, vec_4 r_data)
{
  Eigen::Matrix3d inertial_tensor;
  Eigen::Vector3d MomentTrustRotor;
  Eigen::Vector3d norm (0.0, 0.0, 1.0);

  //inertial_tensor(0,0) = i_data.Ixx; inertial_tensor(1, 1) = i_data.Iyy; inertial_tensor(2,2) = i_data.Izz;

  inertial_tensor<<i_data.Ixx,  0.0,              0.0,
                   0.0,         i_data.Iyy,       0.0,
                   0.0,         0.0,       i_data.Izz;

  double SumRotorAngularVelocity = 0.0;

  SumRotorAngularVelocity += pow2(r_data.variable_1); SumRotorAngularVelocity += pow2(r_data.variable_2);
  SumRotorAngularVelocity += pow2(r_data.variable_3); SumRotorAngularVelocity += pow2(r_data.variable_4);

 /*pitch*/ MomentTrustRotor[0] = i_data.lengthOfFlayerArms * (i_data.motorTrustCoef * (pow2(r_data.variable_1) - pow2(r_data.variable_3)));
 /*roll*/ MomentTrustRotor[1] = i_data.lengthOfFlayerArms * (i_data.motorTrustCoef * (pow2(r_data.variable_4) - pow2(r_data.variable_2)));
                                                          //(pow2(r_data.variable_4) + pow2(r_data.variable_2)) - (pow2(r_data.variable_1) + pow2(r_data.variable_3))
  /*yaw*/ MomentTrustRotor[2] = i_data.motorResistCoef * (pow2(r_data.variable_4) + pow2(r_data.variable_2) - pow2(r_data.variable_1) - pow2(r_data.variable_3));

  //s_data.acceleration = ((i_data.motorResistCoef * SumRotorAngularVelocity * norm.transpose()) * rotation_from_euler(s_data.angular_position[1], s_data.angular_position[0], s_data.angular_position[2]) + i_data.mass * (-9.81 * norm.transpose())) / i_data.mass;

  s_data.acceleration = (Quaternion_rotate(s_data.q_angular_position, ((i_data.motorResistCoef * SumRotorAngularVelocity * norm))) + i_data.mass * (-9.81 * norm)) / i_data.mass;

  s_data.angular_acceleration = inertial_tensor.inverse() * (MomentTrustRotor - s_data.angular_velocity.cross(inertial_tensor * s_data.angular_velocity));

  return s_data;
}

state_vec Simulation_fly::calculateStateVector(vec_4 r_data, float dt)
{

data = Dynemic_model(data, r_data);

for (uint8_t i = 0; i < 3; i++){

vec_2 angular_vec; vec_2 linear_vec;

data.angular_velocity[i] += data.angular_acceleration[i] * dt;

if (data.angular_velocity[i] > 20) data.angular_velocity[i] = 20;
if (data.angular_velocity[i] < -20) data.angular_velocity[i] = -20;

//data.angular_position[i] += data.angular_velocity[i] * dt;

//angular_vec = rk4(data.angular_acceleration[i], i, dt); //get angular velosity & angular position
//data.angular_velocity[i] = angular_vec.variable_1;  data.angular_position[i] = angular_vec.variable_2;

//if (data.angular_position[i] > M_PI) data.angular_position[i] -= TWOPI;
//if (data.angular_position[i] < -M_PI) data.angular_position[i] += TWOPI;


data.velocity[i] += data.acceleration[i] * dt;
data.position[i] += data.velocity[i] * dt;


if (data.position[i] > 1000) {data.position[0] = 0; data.position[1] = 0; data.position[2] = 0; data.velocity[0] = 0; data.velocity[1] = 0; data.velocity[2] = 0;  data.acceleration[0] = 0; data.acceleration[1] = 0; data.acceleration[2] = 0;}
if (data.position[i] < -1000) {data.position[0] = 0; data.position[1] = 0; data.position[2] = 0; data.velocity[0] = 0; data.velocity[1] = 0; data.velocity[2] = 0; data.acceleration[0] = 0; data.acceleration[1] = 0; data.acceleration[2] = 0;}
//linear_vec = rk4(data.acceleration[i], i + 3, dt); //get linear velocity & linear position
//data.velocity[i] = linear_vec.variable_1; data.position[i] = linear_vec.variable_2;

}

data.q_angular_position.integral_quternion(data.angular_velocity, dt);

//data.q_angular_position = {0.9848, -0.1737, 0.0, 0.0};

//qDebug()<<"Angular Acceleration: "<<" x: "<<data.angular_acceleration[0] * RADTODEG<<" y: "<<data.angular_acceleration[1] * RADTODEG<<" z: "<<data.angular_acceleration[2] * RADTODEG;
//qDebug()<<"Angular Velocity: "<<" x: "<<data.angular_velocity[0] * RADTODEG<<" y: "<<data.angular_velocity[1] * RADTODEG<<" z: "<<data.angular_velocity[2] * RADTODEG;
//qDebug()<<"Angular Position: "<<" x: "<<data.angular_position[0] * RADTODEG<<" y: "<<data.angular_position[1] * RADTODEG<<" z: "<<data.angular_position[2] * RADTODEG;

imu::Vector<3> Euler = data.q_angular_position.ToEuler();
                                         //roll         //pitch             //yaw
qDebug()<<"Quat to Euler RPY:      "<<Euler.y()<<"    "<<Euler.z()<<"    "<<Euler.x();

//qDebug()<<"Acceleration: "<<" x: "<<data.acceleration[0]<<" y: "<<data.acceleration[1]<<" z: "<<data.acceleration[2];
//qDebug()<<"Velocity: "<<" x: "<<data.velocity[0]<<" y: "<<data.velocity[1]<<" z: "<<data.velocity[2];
qDebug()<<"Position: "<<" x: "<<data.position[0]<<" y: "<<data.position[1]<<" z: "<<data.position[2];

return data;
}

Eigen::Matrix3d Simulation_fly::rotation_from_euler(double roll, double pitch, double yaw)
{
    // roll and pitch and yaw in radians
        double su = sin(roll);
        double cu = cos(roll);
        double sv = sin(pitch);
        double cv = cos(pitch);
        double sw = sin(yaw);
        double cw = cos(yaw);

        Eigen::Matrix3d Rot_matrix;

        Rot_matrix<<
        cv*cw,  su*sv*cw - cu*sw,   su*sw + cu*sv*cw,
        cv*sw,  cu*cw + su*sv*sw,   cu*sv*sw - su*cw,
        -sv,               su*cv,              cu*cv;

        return Rot_matrix.transpose();
}

Eigen::Vector3d Simulation_fly::Quaternion_rotate(imu::Quaternion q, Eigen::Vector3d v)
   {
       Eigen::Vector3d output;

       //assert(output != NULL);

       float ww = q.w() * q.w();
       float xx = q.x() * q.x();
       float yy = q.y() * q.y();
       float zz = q.z() * q.z();
       float wx = q.w() * q.x();
       float wy = q.w() * q.y();
       float wz = q.w() * q.z();
       float xy = q.x() * q.y();
       float xz = q.x() * q.z();
       float yz = q.y() * q.z();

       // Formula from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
       // p2.x = w*w*p1.x + 2*y*w*p1.z - 2*z*w*p1.y + x*x*p1.x + 2*y*x*p1.y + 2*z*x*p1.z - z*z*p1.x - y*y*p1.x;
       // p2.y = 2*x*y*p1.x + y*y*p1.y + 2*z*y*p1.z + 2*w*z*p1.x - z*z*p1.y + w*w*p1.y - 2*x*w*p1.z - x*x*p1.y;
       // p2.z = 2*x*z*p1.x + 2*y*z*p1.y + z*z*p1.z - 2*w*y*p1.x - y*y*p1.z + 2*w*x*p1.y - x*x*p1.z + w*w*p1.z;

       output.x() = (ww*v.x() + 2*wy*v.z() - 2*wz*v.y() + xx*v.x() + 2*xy*v.y() + 2*xz*v.z() - zz*v.x() - yy*v.x());

       output.y() = (2*xy*v.x() + yy*v.y() + 2*yz*v.z() + 2*wz*v.x() - zz*v.y() + ww*v.y() - 2*wx*v.z() - xx*v.y());

       output.z() = (2*xz*v.x() + 2*yz*v.y() + zz*v.z() - 2*wy*v.x() - yy*v.z() + 2*wx*v.y() - xx*v.z() + ww*v.z());

//       qDebug()<<"Rot_Vec: "<<output.x()<<"  "<<output.y()<<"   "<<output.z();

       return output;
   }

vec_2 Simulation_fly::rk4(float accel, int i, float dt)
{
vec_2 output;

/*velocity */ buffer[i][0] = buffer[i][1] /*velocity_l */;  buffer[i][2]/* k1_v */ = accel;

/*velocity */ buffer[i][0] = buffer[i][1] /*velocity_l */ + buffer[i][2]/* k1_v */ * (dt * 0.5f); buffer[i][3]/* k2_v */ = accel;

/*velocity */ buffer[i][0] = buffer[i][1] /*velocity_l */ + buffer[i][3]/* k2_v */ * (dt * 0.5f); buffer[i][4]/* k3_v*/ = accel;

/*velocity */ buffer[i][0] = buffer[i][1] /*velocity_l */ + buffer[i][4]/* k3_v*/ * dt; buffer[i][5]/* k4_v*/ = accel;

    float v_dt = 1.0f / 6.0f * ( buffer[i][2]/* k1_v */ + 2.0f * ( buffer[i][3]/* k2_v */ + buffer[i][4]/* k3_v*/ ) + buffer[i][5]/* k4_v*/);

    buffer[i][1] /*velocity_l */ += v_dt * dt; output.variable_1 = buffer[i][1]; //Аккумулируем скорость

/*position */ buffer[i][6] = buffer[i][7] /*position_l */;  buffer[i][8] /* k1_p*/ = buffer[i][1] /*velocity_l */;

/*position */ buffer[i][6] = buffer[i][7] /*position_l */ + buffer[i][8] /* k1_p*/ * (dt *0.5f); buffer[i][9] /* k2_p*/ = buffer[i][1] /*velocity_l */;

/*position */ buffer[i][6] = buffer[i][7] /*position_l */ + buffer[i][9] /* k2_p*/ * (dt *0.5f); buffer[i][10]/* k3_p*/ = buffer[i][1] /*velocity_l */;

/*position */ buffer[i][6] = buffer[i][7] /*position_l */ + buffer[i][10]/* k3_p*/ * dt; buffer[i][11]/* k4_p*/ = buffer[i][1] /*velocity_l */;

    float p_dt = 1.0f / 6.0f * ( buffer[i][8] /* k1_p*/ + 2.0f * ( buffer[i][9] /* k2_p*/ + buffer[i][10]/* k3_p*/ ) + buffer[i][11]/* k4_p*/ );

     buffer[i][7]/*position_l */ = p_dt * dt; output.variable_2 = buffer[i][7];

     return output; /*position_l */;
}



