#include "pid.h"

static float calc_error(float reference, float currentOutput)
{
    return reference - currentOutput;
}

static float saturate_output(PIDTypeDef_t *pidObject, float unsatOutput)
{
}

static float calc_proportional(PIDTypeDef_t *pidObject)
{
    return (pidObject->KP * pidObject->error);
}

static float calc_integral(PIDTypeDef_t *pidObject)
{
    float newIntegral = pidObject->kI * pidObject->error;

    float newOutput = newIntegral + pidObject->previousError + pidObject->previousOutput;

    newOutput = saturate_output(pidObject, newOutput);
    return newOutput;
}

/**
 * @brief performs proportional and integral calculation based on a given pidObject and the current
 *        output.
 *
 * @param pidObject representing a generic type which can be the voltage or the current stage
 * @param currentOutput representing the current system output
 * @return float
 */
float calc_pid_output(PIDTypeDef_t *pidObject, float currentOutput)
{
    float error = calc_error(pidObject->referencePoint, currentOutput);
    pidObject->error = error;

    float proportional = calc_proportional(pidObject);

    float integral = calc_integral(pidObject);

    return (proportional + integral);

    // calc_error(pidObject->referencePoint, currentOutput);
    // float proportional = calc_proportional(pidObject);
    // float integral = calc_integral(pidObject);

    // float ret = 0;

    // ret = proportional + integral;

    // ret = saturate_output(pidObject, ret);
}