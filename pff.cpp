#include "pff.h"

PFF::PFF()
{

}

void PFF::filterUpdateFIR(float *shiftBuf, float newSample)
{
    // Shift history buffer and push new sample
    for (int16_t i = fir_filterLength - 1; i > 0; i--)

    shiftBuf[i] = shiftBuf[i - 1];

    shiftBuf[0] = newSample;
}

float PFF::filterApplyFIR(float *shiftBuf, float current_atti, float commonMultiplier)
{
    float accum = 0.0f;

    for (int16_t i = 0; i < fir_filterLength; i++)

    accum += (current_atti - shiftBuf[i]) * coeffBuf[i];

    return (accum / fir_filterLength) * commonMultiplier;
}

float PFF::compute_pff(coef_pff data,float input, float setpoint, float dt)
{
    float Error = input - setpoint;

    if (Error <  data.Error_Min) Error =  data.Error_Min;
    if (Error >  data.Error_Max) Error =  data.Error_Max;

    filterUpdateFIR( shiftBuf_data, input);

    float Output = Error * data.Proportional + filterApplyFIR( shiftBuf_data, input, data.Derivation / dt );

    if (Output > data.Output_Max) Output = data.Output_Max;
    if (Output < data.Output_Min) Output = data.Output_Min;

    return Output;
}
