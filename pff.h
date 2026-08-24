#ifndef PFF_H
#define PFF_H

#include <QObject>

#define fir_filterLength 5

typedef struct
{
 float Error_Min = 0.0f;
 float Error_Max = 0.0f;

 float Output_Min = 0.0f;
 float Output_Max = 0.0f;

 float Proportional = 0.0f;
 float Derivation   = 0.0f;

} coef_pff;

class PFF
{
public:

    PFF();

    void filterUpdateFIR( float * shiftBuf, float newSample);

    float filterApplyFIR( float * shiftBuf, float current_atti ,float commonMultiplier);

    float compute_pff (coef_pff data, float input, float setpoint, float dt);

private:

 //FIR фильтр производной расчитывается по пяти точкам взят из INAV 1.1
  const int8_t coeffBuf [fir_filterLength] = {5, 2, -8, -2, 3};

  float shiftBuf_data [fir_filterLength] = {0, 0, 0, 0, 0};

};

#endif // PFF_H
