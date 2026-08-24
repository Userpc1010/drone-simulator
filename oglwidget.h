#ifndef OGLWIDGET_H
#define OGLWIDGET_H

#include <QOpenGLWidget>
#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions_ES2>
#include <QBasicTimer>
#include <QTimer>
#include "pff.h"
#include "simulation_fly.h"
#include "simpleobject3d.h"

class SimpleObject3D;
class Camera_3D;
class Cube;
class SkyBox;

class OGLWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    OGLWidget(QWidget *QOpenGLWidget = nullptr);
    ~OGLWidget();

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent* event);
    void keyPressEvent(QKeyEvent* event);
    void timerEvent(QTimerEvent* event);

    void setup ();
    void calculate_PID_position(Eigen::Vector3d pos);
    void calculate_PID_attitude();
    void calculate_quat_PID_attitude();


    void initShaders();
    bool initObj(const QString &path, const QImage &img);
    bool initDrone(const QString &path, const QImage &img);
    void calculateTBN(QVector<VertexData> &vertdata);
    bool load_OBJ(const QString &filename, const QImage &texturemap);
    void add(SimpleObject3D* obj);
    bool initTerrain(float width, float height, float depth, const QImage &texturemap);

    void filterUpdateFIR( float * shiftBuf, float newSample);
    float filterApplyFIR( float * shiftBuf, float current_atti ,float commonMultiplier);

public: signals:

  void Draw (double x, double y, double z, double dt);
  void Draw (imu::Vector<3>, double dt);


private:
    QMatrix4x4 m_PojectionMatrix;
    QOpenGLShaderProgram m_Program;
    QOpenGLShaderProgram m_ProgramSkyBox;
    QVector2D m_MousePosition;
    QQuaternion m_Rotation;
    QBasicTimer sim_timer;

    Simulation_fly Sim;

    QVector<SimpleObject3D*> m_Objects;
    QVector<SimpleObject3D*> m_OBJ_Objects;
    SimpleObject3D *Drone;
    SimpleObject3D *Terrain;
    Camera_3D *m_camera;
    Cube  *m_cube;
    SkyBox* m_SkyBox;
    MaterialLibrary m_MatLibrary;

    float s_w, s_h;

    bool camera;

    double lastX = 0.0, lastY = 0.0;

    float dt = 0.01f;
    state_vec data;
    coef_pff coef[6];
    PFF pff[6];

    QVector<Eigen::Vector3d>rout;

    uint8_t routcounter = 0;

    float buffer[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    float setpoint[3] = {0.0f, 0.0f, 0.0f};

    const int8_t coeffBuf [fir_filterLength] = {5, 2, -8, -2, 3};

    float shiftBuf_pitch [fir_filterLength] = {0, 0, 0, 0, 0};
    float shiftBuf_roll [fir_filterLength] = {0, 0, 0, 0, 0};
    float shiftBuf_yaw [fir_filterLength] = {0, 0, 0, 0, 0};

    float shiftBuf_vel_pitch [fir_filterLength] = {0, 0, 0, 0, 0};
    float shiftBuf_vel_roll [fir_filterLength] = {0, 0, 0, 0, 0};
    float shiftBuf_vel_yaw [fir_filterLength] = {0, 0, 0, 0, 0};

    float Last_error_pitch = 0;
    float Last_error_roll = 0;
    float Last_error_yaw = 0;

    float Last_error_vel_pitch = 0;
    float Last_error_vel_roll = 0;
    float Last_error_vel_yaw = 0;

    float Last_error_x = 0;
    float Last_error_y = 0;
    float Last_error_z = 0;

    float Last_error_vel_x = 0;
    float Last_error_vel_y = 0;
    float Last_error_vel_z = 0;

    imu::Quaternion Last_Error = {1.0, 0.0, 0.0, 0.0};

    float shiftBuf_alt [fir_filterLength] = {0, 0, 0, 0, 0};
    float shiftBuf_alt_1 [fir_filterLength] = {0, 0, 0, 0, 0};
    float shiftBuf_front [fir_filterLength] = {0, 0, 0, 0, 0};
    float shiftBuf_front_1 [fir_filterLength] = {0, 0, 0, 0, 0};
    float shiftBuf_right [fir_filterLength] = {0, 0, 0, 0, 0};
    float shiftBuf_right_1 [fir_filterLength] = {0, 0, 0, 0, 0};
};

#endif // OGLWIDGET_H
