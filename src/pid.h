#include <stdint.h>
#include <stdio.h>

typedef struct
{
    float kI;
    float KP;
    float kD;
    float upperLimit;
    float lowerLimit;
    float error;
    float referencePoint;
    float previousError;
    float previousOutput;
} PIDTypeDef_t;

float calc_pid_output(PIDTypeDef_t *pidObject, float currentOutput);
void reset_pid_memory(PIDTypeDef_t *pidObject);