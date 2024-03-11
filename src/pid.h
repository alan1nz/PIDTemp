#include <stdint.h>
#include <stdio.h>

typedef struct
{
    float kI;
    float KP;
    float kD;
    uint32_t upperLimit;
    uint32_t lowerLimit;
    float error;
    float referencePoint;
    float previousError;
    float previousOutput;
} PIDTypeDef_t;

float calc_pid_output(PIDTypeDef_t *pidObject, float currentOutput);