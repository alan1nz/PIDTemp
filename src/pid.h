#include <stdint.h>

// typedef struct
// {
//     float previousError;
//     float previousOutput;
// } PIDMemoryTypeDef_t;

// typedef struct
// {
//     float upperLimit;
//     float lowerLimit;
// } LimitsTypeDef_t;

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
