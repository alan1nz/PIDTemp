#include "pid.h"

static float calc_error(float reference, float currentOutput)
{
    return reference - currentOutput;
}

static float saturate_output(PIDTypeDef_t *pidObject, float unsatOutput)
{
    float ret = 0;
    if (unsatOutput > pidObject->upperLimit)
    {
        ret = pidObject->upperLimit;
    }
    else if (unsatOutput < pidObject->lowerLimit)
    {
        ret = pidObject->lowerLimit;
    }
    else
    {
        ret = unsatOutput;
    }

    return ret;
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

    // Update the integral memories
    pidObject->previousError = newIntegral;
    pidObject->previousOutput = newOutput;

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
    float sum = proportional + integral;
    sum = saturate_output(pidObject, sum);

    return sum;
}

/**
 * @brief This function resets the memory elements of the integral controller
 *
 * @param pidObject representing a generic type which can be the voltage or the current stage
 * @note may want to manually reset the current stage reference
 */
void reset_pid_memory(PIDTypeDef_t *pidObject)
{
    if (pidObject == NULL)
    {
        return;
    }

    pidObject->error = 0;
    pidObject->previousOutput = 0;
    pidObject->previousError = 0;
}