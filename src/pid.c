#include "pid.h"

static float calc_error(float reference, float currentOutput)
{
}

static float calc_proportional(PIDTypeDef_t *pidObject)
{
}

static float calc_integral(PIDTypeDef_t *pidObject)
{
}

static float saturate_output(PIDTypeDef_t *pidObject, float unsatOutput)
{
}

float calc_pid_output(PIDTypeDef_t *pidObject, float currentOutput)
{
    calc_error(pidObject->referencePoint, currentOutput);
    float proportional = calc_proportional(pidObject);
    float integral = calc_integral(pidObject);

    float ret = 0;

    ret = proportional + integral;

    ret = saturate_output(pidObject, ret);
}