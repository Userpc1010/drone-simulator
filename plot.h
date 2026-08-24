#ifndef PLOT_H
#define PLOT_H

#include <QWidget>
#include <QVector>
#include "qcustomplot.h"
#include <quaternion.h>

namespace Ui {
class Plot;
}

class Plot : public QWidget
{
    Q_OBJECT

public:
    explicit Plot(QWidget *parent = nullptr);
    ~Plot();

    void setupQuadraticDemo(QCustomPlot *customPlot);

public slots:

void Draw (double x, double y, double z, double dt);

void Draw (imu::Vector<3> r, double dt);

private:
    Ui::Plot *ui;

    int i = 0;
};

#endif // PLOT_H
