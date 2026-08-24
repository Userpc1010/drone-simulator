#include "oglwidget.h"
#include "simpleobject3d.h"
#include "transformational.h"
#include "camera_3d.h"
#include "cube.h"
#include "skybox.h"
#include <QMouseEvent>
#include <QOpenGLContext>
#include <QtMath>
#include <QDir>


OGLWidget::OGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
   m_camera = new Camera_3D;
   m_cube = new Cube;

   setup();
}

OGLWidget::~OGLWidget()
{
   makeCurrent();
   for(auto o: m_Objects) delete o;
   delete m_camera;

}

void OGLWidget::initializeGL()
{
      glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_CULL_FACE);

      initShaders();

    if(initDrone(":/Drone 2/drone2.obj", QImage(":/Drone 2/drone2_Atlas.png"))) Drone->translate(QVector3D(0.0f, 0.0f, 0.0f));

    if(initTerrain(1000.0, 1.0, 1000.0, QImage(":/Cube.png")))Terrain->translate(QVector3D(0.0f, -1.0f, 0.0f));

    m_cube->add_cube();

    for(uint8_t i = 0; i < rout.length(); i++ ) m_cube->Add_cube(rout[i]);

    m_SkyBox = new SkyBox(1000.0f,
                          QImage(":/sky/sky_forward.png"),
                          QImage(":/sky/sky_top.png"),
                          QImage(":/sky/Terrain_3.jpg"),
                          QImage(":/sky/sky_left.png"),
                          QImage(":/sky/sky_right.png"),
                          QImage(":/sky/sky_back.png"));

    if(!sim_timer.isActive()) sim_timer.start(1, this);
}

void OGLWidget::timerEvent(QTimerEvent *event)
{

  Q_UNUSED(event)

    vec_4 r_data;

  if (((rout[routcounter] - data.position).norm() < 9)) { if(rout.length() > routcounter + 1) routcounter++; qDebug()<<"Points reached: "<<routcounter + 1;}

  calculate_PID_position(rout[routcounter]);
//  calculate_PID_attitude(); // SET POINT
  calculate_quat_PID_attitude();

//  for (uint8_t i = 0; i < 3; i++) buffer[i] = Pff[i].compute_pff(coef[i], data.angular_position[i] * RADTODEG, Setup[i], dt);
//  for (uint8_t i = 0; i < 3; i++) buffer[i] = DEGTORAD * Pff[i + 3].compute_pff(coef[i + 3], data.angular_velocity[i] * RADTODEG, buffer[i], dt);
                               /*yaw*/
  r_data.variable_1 = buffer[3] - buffer[2] + buffer[0]; /*pitch*/
  r_data.variable_2 = buffer[3] + buffer[2] - buffer[1]; /*roll*/
  r_data.variable_3 = buffer[3] - buffer[2] - buffer[0]; /*pitch*/
  r_data.variable_4 = buffer[3] + buffer[2] + buffer[1]; /*roll*/

//  r_data.variable_1 = 500.0 - 0.0 + 0.0; /*pitch*/
//  r_data.variable_2 = 500.0 + 0.0 - 0.0; /*roll*/
//  r_data.variable_3 = 500.0 - 0.0 - 0.0; /*pitch*/
//  r_data.variable_4 = 500.0 + 0.0 + 0.0; /*roll*/

  data = Sim.calculateStateVector(r_data, dt);
                                                                  /* Pitch                                Yaw                                 roll*/
//  Drone->rotate_to(QQuaternion::fromEulerAngles(QVector3D(data.angular_position[0] * RADTODEG, data.angular_position[2] * RADTODEG, data.angular_position[1] * RADTODEG)));

  Drone->rotate_to(data.q_angular_position.Quaternion_to_QQuaternion_for_OGL());

//  emit Draw(data.angular_position[0] * RADTODEG, data.angular_position[2] * RADTODEG, data.angular_position[1] * RADTODEG, dt);

  emit Draw(data.q_angular_position.ToEuler(), dt);

  /*Right            UP             Back*/
  Drone->translate_to(QVector3D(data.position[0], data.position[2], data.position[1]));

  update();
}

void OGLWidget::setup()
{
   //Route
  rout.push_back(Eigen::Vector3d {40,40,10});
  rout.push_back(Eigen::Vector3d {80,80,10});
  rout.push_back(Eigen::Vector3d {140,0,10});
  rout.push_back(Eigen::Vector3d {0,140,10});
  rout.push_back(Eigen::Vector3d {0,0,15});
}

void OGLWidget::calculate_PID_position(Eigen::Vector3d pos)
{
   Eigen::Vector2d directon = {0.0, 0.0};

  float Error_z = pos[2] - data.position[2];
  float Error_y = pos[1] - data.position[1];
  float Error_x = pos[0] - data.position[0];

  if (Error_z < -10) Error_z = -10;
  if (Error_y < -10) Error_y = -10;
  if (Error_x < -10) Error_x = -10;

  if (Error_z > 10) Error_z =  10;
  if (Error_y > 10) Error_y =  10;
  if (Error_x > 10) Error_x =  10;

//  filterUpdateFIR( shiftBuf_alt, data.position[2]);
//  filterUpdateFIR( shiftBuf_front, data.position[1]);
//  filterUpdateFIR( shiftBuf_right, data.position[0]);

//  buffer[3] = 1.0 * Error_z + filterApplyFIR ( shiftBuf_alt, data.position[2], -0.01 / dt);
//   directon[1] = 1.0 * Error_y + filterApplyFIR ( shiftBuf_front, data.position[1], -0.01 / dt);
//    directon[0] = 1.0 * Error_x + filterApplyFIR ( shiftBuf_right, data.position[0], -0.01 / dt);

  float Delta_X = (Error_x - Last_error_x) / dt; Last_error_x = Error_x;
  float Delta_Y = (Error_y - Last_error_y) / dt; Last_error_y = Error_y;
  float Delta_Z = (Error_z - Last_error_z) / dt; Last_error_z = Error_z;

  directon[0] = 1.0 * Error_x + Delta_X * 2.7;
  directon[1] = 1.0 * Error_y + Delta_Y * 2.7;
    buffer[3] = 1.0 * Error_z + Delta_Z * 0.5;

  if ( directon[1] > 40.0f )  directon[1] = 40.0f;
  if ( directon[0] > 40.0f )  directon[0] = 40.0f;
  if ( buffer[3] > 30.0f )    buffer[3] = 30.0f;

  if ( buffer[3] < -30.0f )    buffer[3] = -30.0f;
  if ( directon[1] < -40.0f )  directon[1] = -40.0f;
  if ( directon[0] < -40.0f )  directon[0] = -40.0f;

  Error_z = buffer[3] - data.velocity[2];
  Error_y = directon[1] - data.velocity[1];
  Error_x = directon[0] - data.velocity[0];

  if (Error_z < -10) Error_z = -10;
  if (Error_y < -10) Error_y = -10;
  if (Error_x < -10) Error_x = -10;

  if (Error_z > 10) Error_z =  10;
  if (Error_y > 10) Error_y =  10;
  if (Error_x > 10) Error_x =  10;

//  filterUpdateFIR( shiftBuf_alt_1, data.velocity[2]);
//  filterUpdateFIR( shiftBuf_front_1, data.velocity[1]);
//  filterUpdateFIR( shiftBuf_right_1, data.velocity[0]);

//  buffer[3] = 100.0 * Error_z + filterApplyFIR ( shiftBuf_alt_1, data.velocity[2], -0.5 / dt);
//  directon[1] = 1.0 * Error_y + filterApplyFIR ( shiftBuf_front_1, data.velocity[1], -0.0005 / dt);
//  directon[0] = 1.0 * Error_x + filterApplyFIR ( shiftBuf_right_1, data.velocity[0], -0.0005 / dt);

  float Delta_x = (Error_x - Last_error_vel_x) / dt; Last_error_vel_x = Error_x;
  float Delta_y = (Error_y - Last_error_vel_y) / dt; Last_error_vel_y = Error_y;
  float Delta_z = (Error_z - Last_error_vel_z) / dt; Last_error_vel_z = Error_z;

  directon[0] = 1.0 * Error_x + Delta_x * 2.7;
  directon[1] = 1.0 * Error_y + Delta_y * 2.7;
  buffer[3] = 200.0 * Error_z + Delta_z * 0.2;


  if ( buffer[3] > 500.0f ) buffer[3] = 500.0f;
  if ( directon[1] > 45.0f ) directon[1] = 45.0f;
  if ( directon[0] > 45.0f ) directon[0] = 45.0f;


  if ( buffer[3] < -500.0f ) buffer[3] = -500.0f;
  if ( directon[1] < -45.0f ) directon[1] = -45.0f;
  if ( directon[0] < -45.0f ) directon[0] = -45.0f;

  Eigen::Matrix2d rotation;

  imu::Vector<3> r; r = data.q_angular_position.ToEuler();

  rotation<<
  cos( r.x() * DEGTORAD),  -sin( r.x() * DEGTORAD),
  sin( r.x() * DEGTORAD),   cos( r.x() * DEGTORAD);

  directon = directon.transpose() * rotation;

  setpoint[0] = directon[0]; setpoint[1] = -directon[1];

  pos = pos - data.position;

  //setpoint[2] = atan2(pos[0], pos[1]) * RADTODEG;
}

void OGLWidget::calculate_quat_PID_attitude()
{

    imu::Quaternion setpoint_ = {1.0, 0.0, 0.0, 0.0};

    //qDebug()<<"Setpoint: "<<setpoint[0]<<"  "<<setpoint[1]<<"  "<<setpoint[2];

    setpoint_.EulerToQuaternion(setpoint[2], setpoint[0], setpoint[1]); //  x = yaw y = roll z = pitch

    imu::Quaternion Error = setpoint_ * data.q_angular_position.Inverse();

    imu::Quaternion Delta = Error * Last_Error.Inverse(); Last_Error = Error;

    imu::Vector<3> Error_Euler; imu::Vector<3> Delta_Euler;

    Error_Euler = Error.ToEuler();   Delta_Euler = Delta.ToEuler();

    buffer[0] = 5.0 * Error_Euler.z() + (Delta_Euler.z() / dt) * 7.2;
    buffer[1] = 5.0 * Error_Euler.y() + (Delta_Euler.y() / dt) * 7.2;
    buffer[2] = 5.0 * Error_Euler.x() + (Delta_Euler.x() / dt) * 7.2;

    if ( buffer[0] > 60 )  buffer[0] = 60;
    if ( buffer[1] > 60 )  buffer[1] = 60;
    if ( buffer[2] > 60 )  buffer[2] = 60;

    if ( buffer[0] < -60 )  buffer[0] = -60;
    if ( buffer[1] < -60 )  buffer[1] = -60;
    if ( buffer[2] < -60 )  buffer[2] = -60;


    float Error_Vel_Pitch = buffer[0] - data.angular_velocity[0] * RADTODEG;
    float Error_Vel_Roll = buffer[1] - data.angular_velocity[1]  * RADTODEG;
    float Error_Vel_Yaw = buffer[2] - data.angular_velocity[2]  * RADTODEG;

    if (Error_Vel_Roll < -30.0) Error_Vel_Roll = -30.0;
    if (Error_Vel_Pitch < -30.0) Error_Vel_Pitch = -30.0;
    if (Error_Vel_Yaw < -30.0) Error_Vel_Yaw = -30.0;

    if (Error_Vel_Roll > 30.0) Error_Vel_Roll = 30.0;
    if (Error_Vel_Pitch > 30.0) Error_Vel_Pitch = 30.0;
    if (Error_Vel_Yaw > 30.0) Error_Vel_Yaw = 30.0;


    float Delta_Pitch_vel = (Error_Vel_Pitch - Last_error_vel_pitch) / dt; Last_error_vel_pitch = Error_Vel_Pitch;
    float Delta_Roll_vel = (Error_Vel_Roll - Last_error_vel_roll) / dt; Last_error_vel_roll = Error_Vel_Roll;
    float Delta_Yaw_vel = (Error_Vel_Yaw - Last_error_vel_yaw) / dt; Last_error_vel_yaw = Error_Vel_Yaw;

    buffer[0] = 10.0 * Error_Vel_Pitch + Delta_Pitch_vel * 0.2;
    buffer[1] = 10.0 * Error_Vel_Roll + Delta_Roll_vel * 0.2;
    buffer[2] = 10.0 * Error_Vel_Yaw + Delta_Yaw_vel * 0.1;

    if ( buffer[0] > 60.0 )  buffer[0] = 60.0;
    if ( buffer[1] > 60.0 )  buffer[1] = 60.0;
    if ( buffer[2] > 20.0 )  buffer[2] = 20.0;

    if ( buffer[0] < -60.0 )  buffer[0] = -60.0;
    if ( buffer[1] < -60.0 )  buffer[1] = -60.0;
    if ( buffer[2] < -20.0 )  buffer[2] = -20.0;

    buffer[0] *= DEGTORAD;
    buffer[1] *= DEGTORAD;
    buffer[2] *= DEGTORAD;
}


void OGLWidget::calculate_PID_attitude()
{
  float Error_Pitch = setpoint[0] - data.angular_position[0] * RADTODEG;
  float Error_Roll = setpoint[1] - data.angular_position[1]  * RADTODEG;
  float Error_Yaw =  setpoint[2] - data.angular_position[2]  * RADTODEG;

  if (Error_Yaw < -180.0f) Error_Yaw += 360.0f; // Делаем результат вычетания вида -180 +180
  if (Error_Yaw >  180.0f) Error_Yaw -= 360.0f;

  if (Error_Pitch < -180.0f) Error_Pitch += 360.0f; // Делаем результат вычетания вида -180 +180
  if (Error_Pitch >  180.0f) Error_Pitch -= 360.0f;

  if (Error_Roll < -180.0f) Error_Roll += 360.0f; // Делаем результат вычетания вида -180 +180
  if (Error_Roll >  180.0f) Error_Roll -= 360.0f;

  if (Error_Yaw < -30) Error_Yaw = -30;
  if (Error_Roll < -30) Error_Roll = -30;
  if (Error_Pitch < -30) Error_Pitch = -30;

  if (Error_Yaw > 30) Error_Yaw =  30;
  if (Error_Roll > 30) Error_Roll =  30;
  if (Error_Pitch > 30) Error_Pitch = 30;

  float Delta_Pitch = (Error_Pitch - Last_error_pitch) / dt; Last_error_pitch = Error_Pitch;
  float Delta_Roll = (Error_Roll - Last_error_roll) / dt; Last_error_roll = Error_Roll;
  float Delta_Yaw = (Error_Yaw - Last_error_yaw) / dt; Last_error_yaw = Error_Yaw;

  buffer[0] = 10.0 * Error_Pitch + Delta_Pitch * 7.2;
  buffer[1] = 10.0 * Error_Roll + Delta_Roll * 7.2;
  buffer[2] = 10.0 * Error_Yaw + Delta_Yaw * 7.2;

//  filterUpdateFIR( shiftBuf_pitch, data.angular_position[0] * RADTODEG);
//  filterUpdateFIR( shiftBuf_roll, data.angular_position[1] * RADTODEG);
//  filterUpdateFIR( shiftBuf_yaw, data.angular_position[2] * RADTODEG);

//  buffer[0] = 1.0 * Error_Pitch + filterApplyFIR ( shiftBuf_pitch, data.angular_position[0] * RADTODEG, -0.01 / dt);
//  buffer[1] = 1.0 * Error_Roll + filterApplyFIR ( shiftBuf_roll, data.angular_position[1] * RADTODEG, -0.01 / dt);
//  buffer[2] = 1.0 * Error_Yaw + filterApplyFIR ( shiftBuf_yaw, data.angular_position[2] * RADTODEG, -0.002 / dt);

  if ( buffer[0] > 60 )  buffer[0] = 60;
  if ( buffer[1] > 60 )  buffer[1] = 60;
  if ( buffer[2] > 60 )  buffer[2] = 60;

  if ( buffer[0] < -60 )  buffer[0] = -60;
  if ( buffer[1] < -60 )  buffer[1] = -60;
  if ( buffer[2] < -60 )  buffer[2] = -60;

  float Error_Vel_Pitch = buffer[0] - data.angular_velocity[0] * RADTODEG;
  float Error_Vel_Roll = buffer[1] - data.angular_velocity[1]  * RADTODEG;
  float Error_Vel_Yaw = buffer[2] - data.angular_velocity[2]  * RADTODEG;

  if (Error_Vel_Roll < -30.0) Error_Vel_Roll = -30.0;
  if (Error_Vel_Pitch < -30.0) Error_Vel_Pitch = -30.0;
  if (Error_Vel_Yaw < -30.0) Error_Vel_Yaw = -30.0;

  if (Error_Vel_Roll > 30.0) Error_Vel_Roll = 30.0;
  if (Error_Vel_Pitch > 30.0) Error_Vel_Pitch = 30.0;
  if (Error_Vel_Yaw > 30.0) Error_Vel_Yaw = 30.0;

//  filterUpdateFIR( shiftBuf_vel_pitch, data.angular_velocity[0] * RADTODEG);
//  filterUpdateFIR( shiftBuf_vel_roll, data.angular_velocity[1] * RADTODEG);
//  filterUpdateFIR( shiftBuf_vel_yaw, data.angular_velocity[2] * RADTODEG);

//  buffer[0] = 1.0 * Error_Vel_Pitch + filterApplyFIR ( shiftBuf_vel_pitch, data.angular_velocity[0] * RADTODEG, -0.002 / dt);
//  buffer[1] = 1.0 * Error_Vel_Roll + filterApplyFIR ( shiftBuf_vel_roll, data.angular_velocity[1] * RADTODEG, -0.002 / dt);
//  buffer[2] = 1.0 * Error_Vel_Yaw + filterApplyFIR ( shiftBuf_vel_yaw, data.angular_velocity[2] * RADTODEG, -0.0005/ dt);

  float Delta_Pitch_vel = (Error_Vel_Pitch - Last_error_vel_pitch) / dt; Last_error_vel_pitch = Error_Vel_Pitch;
  float Delta_Roll_vel = (Error_Vel_Roll - Last_error_vel_roll) / dt; Last_error_vel_roll = Error_Vel_Roll;
  float Delta_Yaw_vel = (Error_Vel_Yaw - Last_error_vel_yaw) / dt; Last_error_vel_yaw = Error_Vel_Yaw;

  buffer[0] = 10.0 * Error_Vel_Pitch + Delta_Pitch_vel * 0.4;
  buffer[1] = 10.0 * Error_Vel_Roll + Delta_Roll_vel * 0.4;
  buffer[2] = 10.0 * Error_Vel_Yaw + Delta_Yaw_vel * 0.1;

  if ( buffer[0] > 60.0 )  buffer[0] = 60.0;
  if ( buffer[1] > 60.0 )  buffer[1] = 60.0;
  if ( buffer[2] > 20.0 )  buffer[2] = 20.0;

  if ( buffer[0] < -60.0 )  buffer[0] = -60.0;
  if ( buffer[1] < -60.0 )  buffer[1] = -60.0;
  if ( buffer[2] < -20.0 )  buffer[2] = -20.0;

  buffer[0] *= DEGTORAD;
  buffer[1] *= DEGTORAD;
  buffer[2] *= DEGTORAD;
}



void OGLWidget::resizeGL(int w, int h)
{
    float aspect = w / (h? static_cast<float>(h) : 1);
    m_PojectionMatrix.setToIdentity();
    m_PojectionMatrix.perspective(88, aspect, 0.01f, 1000.0f);
}

void OGLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_ProgramSkyBox.bind();
    m_ProgramSkyBox.setUniformValue("u_projectionMatrix", m_PojectionMatrix);
    m_camera->draw(&m_ProgramSkyBox);
    m_SkyBox->draw(&m_ProgramSkyBox, context()->functions());
    m_ProgramSkyBox.release();

    m_Program.bind();
    m_Program.setUniformValue("u_projectionMatrix", m_PojectionMatrix);
    m_camera->draw(&m_Program);
    m_cube->draw(&m_Program,context()->functions());
    Drone->draw(&m_Program, context()->functions());
    Terrain->draw(&m_Program, context()->functions());
    for(auto o: m_Objects) o->draw(&m_Program, context()->functions());
    for(auto o: m_OBJ_Objects) o->draw(&m_Program, context()->functions());

    m_Program.release();
}

void OGLWidget::mousePressEvent(QMouseEvent *event)
{
    if(event->buttons() == Qt::LeftButton)
    {
     m_cube->add_cube();
    }

    if(event->buttons() == Qt::MidButton)
    {
     m_cube->delete_cube();
    }

    if(event->buttons() == Qt::RightButton)
    {
     lastX =  event->localPos().x();
     lastY =  event->localPos().y();
    }

    event->accept();
}

void OGLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->buttons() == Qt::RightButton)
    {
     camera = true;
    }
    event->accept();
}


void OGLWidget::mouseMoveEvent(QMouseEvent *event)
{

    if(event->buttons() == Qt::RightButton)
    {
        if (camera){

          lastX = event->localPos().x();
          lastY = event->localPos().y();
          camera = false;
        }
         double xoffset = event->localPos().x() - lastX;
         double yoffset = lastY - event->localPos().y();

         lastX = event->localPos().x();
         lastY = event->localPos().y();

         m_camera->rotate_camera(xoffset, yoffset);
    }

    event->accept();
}

void OGLWidget::wheelEvent(QWheelEvent *event)
{
  if ( event->delta() > 0 ) m_camera->camera_zoom(true);
  if ( event->delta() < 0 ) m_camera->camera_zoom(false);

  event->accept();
}

void OGLWidget::keyPressEvent(QKeyEvent *event)
{
     //qDebug()<< event->key();
     if (event->key() == 1062) m_camera->Front_move();
     if (event->key() == 1067) m_camera->Back_move();
     if (event->key() == 1060) m_camera->right_move();
     if (event->key() == 1042) m_camera->left_move();
     if (event->key() == 32) m_camera->Up_move();
     if (event->key() == 16777250) m_camera->Down_move();

     event->accept();
}

void OGLWidget::initShaders()
{
    if(! m_Program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vertshader.vsh"))
        close();
    if(! m_Program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fragshader.fsh"))
        close();
    if(! m_Program.link())
        close();

    if(! m_ProgramSkyBox.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/skybox.vsh"))
        close();
    if(! m_ProgramSkyBox.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/skybox.fsh"))
        close();
    if(! m_ProgramSkyBox.link()) close();

}

bool OGLWidget::initDrone(const QString &path, const QImage &img)
{
    QFile objfile(path);

    if(! objfile.exists()) { qCritical() << "File not exist:" << path; return false; }
    if(! objfile.open(QFile::ReadOnly))  { qCritical() << "File not opened:" << path; return false; }

    QTextStream input(&objfile);
    QVector<QVector3D> coords;
    QVector<QVector2D> texturcoords;
    QVector<QVector3D> normals;

    QVector<VertexData> vertexes;
    QVector<GLuint> indexes;

    qDebug() << "Reading" << path << "...";

    bool ok = true;
    while(!input.atEnd() && ok)
    {
        auto str = input.readLine(); if(str.isEmpty()) continue;
        auto strlist = str.split(' '); strlist.removeAll("");
        auto key = strlist.at(0);

        if (key == "#") { qDebug() << str; }
        else if(key == "mtllib")
        {
            qDebug() << str;
            // материал
        }
        else if(key.toLower() == "o")
        {
            qDebug() << str;
        }
        else if(key.toLower() == "g")
        {
            qDebug() << str;
        }
        else if(key.toLower() == "s")
        {
            qDebug() << str;
        }
        else if(key.toLower() == "v")
        {
            if(strlist.size() > 3)
            {
                coords.append(QVector3D(strlist.at(1).toFloat(&ok),
                                        strlist.at(2).toFloat(&ok),
                                        strlist.at(3).toFloat(&ok)));
                if(!ok) { qCritical() << "Error at line (format):" << str; }
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
        else if(key.toLower() == "vt")
        {
            if(strlist.size() > 2)
            {
                texturcoords.append(QVector2D(strlist.at(1).toFloat(&ok),
                                              strlist.at(2).toFloat(&ok)));
                if(!ok) { qCritical() << "Error at line (format):" << str; }
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
        else if(key.toLower() == "vn")
        {
            if(strlist.size() == 4)
            {
                normals.append(QVector3D(strlist.at(1).toFloat(&ok),
                                         strlist.at(2).toFloat(&ok),
                                         strlist.at(3).toFloat(&ok)));
                if(!ok) { qCritical() << "Error at line (format):" << str; }
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
        else if(key.toLower() == "f")
        {
            for(int i = 1; i < strlist.size(); i++)
            {
                auto v = strlist.at(i).split('/');
                if(v.size() == 3 && !v.at(1).isEmpty() && !v.at(2).isEmpty())
                {
                    vertexes.append(VertexData(coords.at(v.at(0).toInt(&ok, 10) - 1),
                                               texturcoords.at(v.at(1).toInt(&ok, 10) - 1),
                                               normals.at(v.at(2).toInt(&ok, 10) - 1)));
                    indexes.append(static_cast<GLuint>(indexes.size()));
                }
                else
                {
                    qCritical() << "Unsupported OBJ data format:" << strlist.at(i);
                    ok = false; break;
                }
            }
            if(!ok) { qCritical() << "Error at line (format):" << str; }
        }
    }

    objfile.close();
    qDebug() <<  "... done";
    if(!ok) return false;

    Drone = new SimpleObject3D(vertexes, indexes, img, 0);
    return true;
}

void OGLWidget::calculateTBN(QVector<VertexData> &vertdata)
{
    for(int i = 0; i < vertdata.size(); i += 3)
    {
        auto v1 = vertdata.at(i).position;
        auto v2 = vertdata.at(i + 1).position;
        auto v3 = vertdata.at(i + 2).position;

        auto uv1 = vertdata.at(i).textcoord;
        auto uv2 = vertdata.at(i + 1).textcoord;
        auto uv3 = vertdata.at(i + 2).textcoord;

        // схема вычисления T и B
        // deltaPos1 = deltaUV1.x * T + deltaUV1.y * B;
        // deltaPos2 = deltaUV2.x * T + deltaUV2.y * B;

        auto deltaPos1 = v2 - v1; auto deltaPos2 = v3 - v1;
        auto deltaUV1 = uv2 - uv1; auto deltaUV2 = uv3 - uv1;
        float r = 0.1f / (deltaUV1.x() * deltaUV2.y() - deltaUV1.y() * deltaUV2.x());
        QVector3D tangent = (deltaPos1 * deltaUV2.y() - deltaPos2 * deltaUV1.y()) * r;
        QVector3D bitangent = (deltaPos2 * deltaUV1.x() - deltaPos1 * deltaUV2.x()) * r;

        vertdata[i].tangent = tangent;
        vertdata[i + 1].tangent = tangent;
        vertdata[i + 2].tangent = tangent;

        vertdata[i].bitangent = bitangent;
        vertdata[i + 1].bitangent = bitangent;
        vertdata[i + 2].bitangent = bitangent;
    }
}

bool OGLWidget::load_OBJ(const QString &filename, const QImage &texturemap)
{
    if(m_OBJ_Objects.size()) {qCritical() << "EngineObject not empty!"; return false; }

    QFile objfile(filename);

    if(! objfile.exists()) { qCritical() << "File not exist:" << filename; return false; }
    if(! objfile.open(QFile::ReadOnly))  { qCritical() << "File not opened:" << filename; return false; }

    QTextStream input(&objfile);
    QVector<QVector3D> coords;
    QVector<QVector2D> texturcoords;
    QVector<QVector3D> normals;

    QVector<VertexData> vertexes;
    QVector<GLuint> indexes;
    QString mtlName;

    qDebug() << "Reading" << filename << "...";

    bool ok = true;
    while(!input.atEnd() && ok)
    {
        auto str = input.readLine(); if(str.isEmpty()) continue;
        auto strlist = str.split(' '); strlist.removeAll("");
        auto key = strlist.at(0).toLower();

        if (key == "#") { qDebug() << str; }
        else if(key == "mtllib")
        {
            if(strlist.size() > 1)
            {
                auto file = QFileInfo(filename).absolutePath() + QDir::separator() + strlist.at(1);
                if(QFile(file).exists()) ok = m_MatLibrary.load(file);
                else { qCritical() << "File not exists:" << file; ok = false; }
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
        else if(key == "v")
        {
            if(strlist.size() > 3)
            {
                coords.append(QVector3D(strlist.at(1).toFloat(&ok),
                                        strlist.at(2).toFloat(&ok),
                                        strlist.at(3).toFloat(&ok)));
                if(!ok) { qCritical() << "Error at line (format):" << str; }
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
        else if(key == "vt")
        {
            if(strlist.size() > 2)
            {
                texturcoords.append(QVector2D(strlist.at(1).toFloat(&ok),
                                              strlist.at(2).toFloat(&ok)));
                if(!ok) { qCritical() << "Error at line (format):" << str; }
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
        else if(key == "vn")
        {
            if(strlist.size() == 4)
            {
                normals.append(QVector3D(strlist.at(1).toFloat(&ok),
                                         strlist.at(2).toFloat(&ok),
                                         strlist.at(3).toFloat(&ok)));
                if(!ok) { qCritical() << "Error at line (format):" << str; }
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
        else if(key == "f")
        {
            for(int i = 1; i < strlist.size(); i++)
            {
                auto v = strlist.at(i).split('/');
                if(v.size() == 3 && !v.at(1).isEmpty() && !v.at(2).isEmpty())
                {
                    vertexes.append(VertexData(coords.at(v.at(0).toInt(&ok, 10) - 1),
                                               texturcoords.at(v.at(1).toInt(&ok, 10) - 1),
                                               normals.at(v.at(2).toInt(&ok, 10) - 1)));
                    indexes.append(static_cast<GLuint>(indexes.size()));
                }
                else
                {
                    qCritical() << "Unsupported OBJ data format:" << strlist.at(i);
                    ok = false; break;
                }
            }
            if(!ok) { qCritical() << "Error at line (format):" << str; }
        }
        else if(key == "usemtl")
        {
            if(strlist.size() > 1)
            {

                calculateTBN(vertexes);
                m_OBJ_Objects.append(new SimpleObject3D(vertexes, indexes, texturemap, 1, m_MatLibrary.get(mtlName)));

                mtlName = strlist.at(1);

                vertexes.clear();
                indexes.clear();
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
    }

    objfile.close(); qDebug() <<  "... done" << filename;

    if(!ok) return false;


     calculateTBN(vertexes);
     m_OBJ_Objects.append(new SimpleObject3D(vertexes, indexes, texturemap, 1, m_MatLibrary.get(mtlName)));


    qDebug() << "Object is loaded successfully:" << m_OBJ_Objects.size() << "elements";
    return true;
}

void OGLWidget::add(SimpleObject3D *obj)
{
    if(! obj) return;
    for(auto o: m_OBJ_Objects) if (o == obj) return;
    m_OBJ_Objects.append(obj);
}

bool OGLWidget::initTerrain(float width, float height, float depth, const QImage &texturemap)
{
    float width_div_2 = width / 2.0f;
    float height_div_2 = height / 2.0f;
    float depth_div_2 = depth / 2.0f;
    float repeat = (height * width) / 100;

    QVector<VertexData> vertexes;
    vertexes.append(VertexData(QVector3D(-width_div_2, height_div_2, depth_div_2), QVector2D(0.0f, repeat), QVector3D(0.0f, 0.0f, 1.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, -height_div_2, depth_div_2), QVector2D(0.0f, 0.0f), QVector3D(0.0f, 0.0f, 1.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, height_div_2, depth_div_2), QVector2D(repeat, repeat), QVector3D(0.0f, 0.0f, 1.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, height_div_2, depth_div_2), QVector2D(repeat, repeat), QVector3D(0.0f, 0.0f, 1.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, -height_div_2, depth_div_2), QVector2D(0.0f, 0.0f), QVector3D(0.0f, 0.0f, 1.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, -height_div_2, depth_div_2), QVector2D(repeat, 0.0f), QVector3D(0.0f, 0.0f, 1.0f)));

    vertexes.append(VertexData(QVector3D(width_div_2, height_div_2, depth_div_2), QVector2D(0.0f, repeat), QVector3D(1.0f, 0.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, -height_div_2, depth_div_2), QVector2D(0.0f, 0.0f), QVector3D(1.0f, 0.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, height_div_2, -depth_div_2), QVector2D(repeat, repeat), QVector3D(1.0f, 0.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, height_div_2, -depth_div_2), QVector2D(repeat, repeat), QVector3D(1.0f, 0.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, -height_div_2, depth_div_2), QVector2D(0.0f, 0.0f), QVector3D(1.0f, 0.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, -height_div_2, -depth_div_2), QVector2D(repeat, 0.0f), QVector3D(1.0f, 0.0f, 0.0f)));

    vertexes.append(VertexData(QVector3D(width_div_2, height_div_2, depth_div_2), QVector2D(0.0f, repeat), QVector3D(0.0f, 1.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, height_div_2, -depth_div_2), QVector2D(0.0f, 0.0f), QVector3D(0.0f, 1.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, height_div_2, depth_div_2), QVector2D(repeat, repeat), QVector3D(0.0f, 1.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, height_div_2, depth_div_2), QVector2D(repeat, repeat), QVector3D(0.0f, 1.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, height_div_2, -depth_div_2), QVector2D(0.0f, 0.0f), QVector3D(0.0f, 1.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, height_div_2, -depth_div_2), QVector2D(repeat, 0.0f), QVector3D(0.0f, 1.0f, 0.0f)));

    vertexes.append(VertexData(QVector3D(width_div_2, height_div_2, -depth_div_2), QVector2D(0.0f, repeat), QVector3D(0.0f, 0.0f, -1.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, -height_div_2, -depth_div_2), QVector2D(0.0f, 0.0f), QVector3D(0.0f, 0.0f, -1.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, height_div_2, -depth_div_2), QVector2D(repeat, repeat), QVector3D(0.0f, 0.0f, -1.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, height_div_2, -depth_div_2), QVector2D(repeat, repeat), QVector3D(0.0f, 0.0f, -1.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, -height_div_2, -depth_div_2), QVector2D(0.0f, 0.0f), QVector3D(0.0f, 0.0f, -1.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, -height_div_2, -depth_div_2), QVector2D(repeat, 0.0f), QVector3D(0.0f, 0.0f, -1.0f)));

    vertexes.append(VertexData(QVector3D(-width_div_2, height_div_2, depth_div_2), QVector2D(0.0f, repeat), QVector3D(-1.0f, 0.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, height_div_2, -depth_div_2), QVector2D(0.0f, 0.0f), QVector3D(-1.0f, 0.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, -height_div_2, depth_div_2), QVector2D(repeat, repeat), QVector3D(-1.0f, 0.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, -height_div_2, depth_div_2), QVector2D(repeat, repeat), QVector3D(-1.0f, 0.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, height_div_2, -depth_div_2), QVector2D(0.0f, 0.0f), QVector3D(-1.0f, 0.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, -height_div_2, -depth_div_2), QVector2D(repeat, 0.0f), QVector3D(-1.0f, 0.0f, 0.0f)));

    vertexes.append(VertexData(QVector3D(-width_div_2, -height_div_2, depth_div_2), QVector2D(0.0f, repeat), QVector3D(0.0f, -1.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, -height_div_2, -depth_div_2), QVector2D(0.0f, 0.0f), QVector3D(0.0f, -1.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, -height_div_2, depth_div_2), QVector2D(repeat, repeat), QVector3D(0.0f, -1.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, -height_div_2, depth_div_2), QVector2D(repeat, repeat), QVector3D(0.0f, -1.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(-width_div_2, -height_div_2, -depth_div_2), QVector2D(0.0f, 0.0f), QVector3D(0.0f, -1.0f, 0.0f)));
    vertexes.append(VertexData(QVector3D(width_div_2, -height_div_2, -depth_div_2), QVector2D(repeat, 0.0f), QVector3D(0.0f, -1.0f, 0.0f)));

    QVector<GLuint> indexes;
    for(GLuint i = 0; i < 36; ++i) indexes.append(i);

    Terrain = new SimpleObject3D(vertexes, indexes, texturemap, 2);
    return true;
}

void OGLWidget::filterUpdateFIR(float *shiftBuf, float newSample)
{
    // Shift history buffer and push new sample
    for (int16_t i = fir_filterLength - 1; i > 0; i--)

    shiftBuf[i] = shiftBuf[i - 1];

    shiftBuf[0] = newSample;
}

float OGLWidget::filterApplyFIR(float *shiftBuf, float current_atti, float commonMultiplier)
{
    float accum = 0.0f;

    for (int16_t i = 0; i < fir_filterLength; i++)

    accum += (current_atti - shiftBuf[i]) * coeffBuf[i];

    return (accum / fir_filterLength) * commonMultiplier;
}


bool OGLWidget::initObj(const QString &path, const QImage &img)
{
    QFile objfile(path);

    if(! objfile.exists()) { qCritical() << "File not exist:" << path; return false; }
    if(! objfile.open(QFile::ReadOnly))  { qCritical() << "File not opened:" << path; return false; }

    QTextStream input(&objfile);
    QVector<QVector3D> coords;
    QVector<QVector2D> texturcoords;
    QVector<QVector3D> normals;

    QVector<VertexData> vertexes;
    QVector<GLuint> indexes;

    qDebug() << "Reading" << path << "...";

    bool ok = true;
    while(!input.atEnd() && ok)
    {
        auto str = input.readLine(); if(str.isEmpty()) continue;
        auto strlist = str.split(' '); strlist.removeAll("");
        auto key = strlist.at(0);

        if (key == "#") { qDebug() << str; }
        else if(key == "mtllib")
        {
            qDebug() << str;
            // материал
        }
        else if(key.toLower() == "o")
        {
            qDebug() << str;
        }
        else if(key.toLower() == "g")
        {
            qDebug() << str;
        }
        else if(key.toLower() == "s")
        {
            qDebug() << str;
        }
        else if(key.toLower() == "v")
        {
            if(strlist.size() > 3)
            {
                coords.append(QVector3D(strlist.at(1).toFloat(&ok),
                                        strlist.at(2).toFloat(&ok),
                                        strlist.at(3).toFloat(&ok)));
                if(!ok) { qCritical() << "Error at line (format):" << str; }
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
        else if(key.toLower() == "vt")
        {
            if(strlist.size() > 2)
            {
                texturcoords.append(QVector2D(strlist.at(1).toFloat(&ok),
                                              strlist.at(2).toFloat(&ok)));
                if(!ok) { qCritical() << "Error at line (format):" << str; }
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
        else if(key.toLower() == "vn")
        {
            if(strlist.size() == 4)
            {
                normals.append(QVector3D(strlist.at(1).toFloat(&ok),
                                         strlist.at(2).toFloat(&ok),
                                         strlist.at(3).toFloat(&ok)));
                if(!ok) { qCritical() << "Error at line (format):" << str; }
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
        else if(key.toLower() == "f")
        {
            for(int i = 1; i < strlist.size(); i++)
            {
                auto v = strlist.at(i).split('/');
                if(v.size() == 3 && !v.at(1).isEmpty() && !v.at(2).isEmpty())
                {
                    vertexes.append(VertexData(coords.at(v.at(0).toInt(&ok, 10) - 1),
                                               texturcoords.at(v.at(1).toInt(&ok, 10) - 1),
                                               normals.at(v.at(2).toInt(&ok, 10) - 1)));
                    indexes.append(static_cast<GLuint>(indexes.size()));
                }
                else
                {
                    qCritical() << "Unsupported OBJ data format:" << strlist.at(i)<< " str: "<< i;
                    ok = false; break;
                }
            }
            if(!ok) { qCritical() << "Error at line (format):" << str; }
        }
    }

    objfile.close();
    qDebug() <<  "... done";
    if(!ok) return false;

    m_Objects.append(new SimpleObject3D(vertexes, indexes, img, 0));
    return true;
}
