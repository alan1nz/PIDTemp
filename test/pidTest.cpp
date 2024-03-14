#include "gtest/gtest.h"

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

extern "C"
{
#include "pid.h"
}

/**
 * @brief Equivalent partitioning testing of calculating the positive error for the current PID stage.
 * @param reference 1.5
 * @param operating 0
 * @result 1.5
 *
 */
TEST(CALC_ERROR, EP_CURRENT_1)
{
    PIDTypeDef_t pidObject;
    pidObject.referencePoint = 1.5;

    float ret = 0;

    ret = calc_pid_output(&pidObject, 0);

    EXPECT_EQ(1.5, pidObject.error);
}

/**
 * @brief Equivalence partitioning testing of calculating the negative error for the current PID stage.
 * @param reference 0
 * @param operating 1.5
 * @result -1.5
 *
 */
TEST(CALC_ERROR, EP_CURRENT_2)
{
    PIDTypeDef_t pidObject;
    pidObject.referencePoint = 0;

    float ret = 0;

    ret = calc_pid_output(&pidObject, 1.5);

    EXPECT_EQ(-1.5, pidObject.error);
}

/**
 * @brief Equivalence partitioning testing of calculating zero error for the current PID stage.
 * @param reference 1.5
 * @param operating 1.5
 * @result 0
 */
TEST(CALC_ERROR, EQ_CURRENT_3)
{
    PIDTypeDef_t pidObject;
    pidObject.referencePoint = 1.5;

    calc_pid_output(&pidObject, 1.5);

    EXPECT_EQ(0, pidObject.error);
}

/**
 * @brief Fault injection testing for when the reference goes outside the expected range.
 * @param reference 2
 * @param operating 10
 * @result -8
 */
TEST(CALC_ERROR, FAULT_INJECTION_1)
{
    PIDTypeDef_t pidObject;
    pidObject.referencePoint = 2;

    calc_pid_output(&pidObject, 10);

    EXPECT_EQ(-8, pidObject.error);
}
/**
 * @brief Fault injection testing for when the operating goes outside the expected range.
 * @param reference 3
 * @param operating 10
 * @result 7
 */

TEST(CALC_ERROR, FAULT_INJECTION_2)
{
    PIDTypeDef_t pidObject;
    pidObject.referencePoint = 10;

    calc_pid_output(&pidObject, 3);

    EXPECT_EQ(7, pidObject.error);
}

/**
 * @brief Simple test to calculate positive proportional output
 *
 */
TEST(CALC_PROPORTIONAL, PROPORTIONAL_POSITIVE_TEST)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 4;
    pidObject.kI = 0;
    pidObject.referencePoint = 50.4;
    pidObject.upperLimit = 2000;
    pidObject.lowerLimit = -2000;
    pidObject.previousError = 0;
    pidObject.previousOutput = 0;
    float ret = 0;

    ret = calc_pid_output(&pidObject, 50);
    EXPECT_FLOAT_EQ(1600, uint32_t(ret * 1000)); // Comparing float directly has rounding issues.
}

/**
 * @brief Simple test to calculate negative proportional output
 *
 */
TEST(CALC_PROPORTIONAL, PROPORTIONAL_NEGATIVE_TEST)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 4;
    pidObject.kI = 0;
    pidObject.upperLimit = 2000;
    pidObject.lowerLimit = -2000;
    pidObject.previousError = 0;
    pidObject.previousOutput = 0;
    pidObject.referencePoint = 50.4;

    float ret = 0;

    ret = calc_pid_output(&pidObject, 54);
    EXPECT_FLOAT_EQ(-14399, int32_t(ret * 1000)); // Theorectically it should be 14400 but there is rounding issue
}

/**
 * @brief Simple test to calculate positive integral output
 */
TEST(CALC_INTEGRAL, INTEGRAL_POSITIVE_TEST)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.5;
    pidObject.referencePoint = 46.8;
    pidObject.upperLimit = 2000;
    pidObject.lowerLimit = -2000;

    float ret = calc_pid_output(&pidObject, 44.4);
    EXPECT_FLOAT_EQ(uint32_t(ret * 1000), 1199); // rounding issue
}

/**
 * @brief Simple test to calculate negative integral output
 */
TEST(CALC_INTEGRAL, INTEGRAL_NEGATIVE_TEST)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.5;
    pidObject.upperLimit = 2000;
    pidObject.lowerLimit = -2000;
    pidObject.referencePoint = 44.4;

    float ret = calc_pid_output(&pidObject, 46.8);
    EXPECT_FLOAT_EQ(int32_t(ret * 1000), -1199); // rounding issue
}

/**
 * @brief Test whether if the integral will saturate the output to the defined upper limit
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_POSITIVE_SATURATION)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.5;
    pidObject.referencePoint = 46.8;

    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 3;

    float ret = calc_pid_output(&pidObject, 30);

    EXPECT_EQ(3, ret);
}

/**
 * @brief Test wheter if the integral will saturate the output to the defined lower limit
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_NEGATIVE_SATURATION)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.5;
    pidObject.referencePoint = 46.8;

    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 3;

    float ret = calc_pid_output(&pidObject, 100);

    EXPECT_EQ(0, ret);
}

/**
 * @brief Test whether if the memory of the integral is accumulating in the positive direction
 * @note Refer to the excel document for a full list of test cases
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_POSITIVE_ACCUMULATION_PARTITION_1)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.5;
    pidObject.referencePoint = 50.4;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 3;

    float ret = 0;
    for (uint8_t i = 0; i < 10; i++)
    {
        ret = calc_pid_output(&pidObject, 50.3);
    }

    EXPECT_EQ((uint32_t)(ret * 1000), 950);
}

/**
 * @brief Test whether if the memory of the integral is accumulating in the positive direction
 * @note Refer to the excel document for a full list of test cases
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_POSITIVE_ACCUMULATION_PARTITION_2)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.5;
    pidObject.referencePoint = 50.4;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 3;

    float ret = 0;
    for (uint8_t i = 0; i < 20; i++)
    {
        ret = calc_pid_output(&pidObject, 50.3);
    }

    EXPECT_EQ((uint32_t)(ret * 1000), 1950);
}

/**
 * @brief Test whether if the memory of the integral is accumulating in the positive direction
 * @note Refer to the excel document for a full list of test cases
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_POSITIVE_ACCUMULATION_PARTITION_3)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.5;
    pidObject.referencePoint = 50.4;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 3;

    float ret = 0;
    for (uint8_t i = 0; i < 30; i++)
    {
        ret = calc_pid_output(&pidObject, 50.3);
    }

    EXPECT_EQ((uint32_t)(ret * 1000), 2950);
}

/**
 * @brief Test whether if the memory of the integral is accumulating in the positive direction as well as saturate
 *        at the upper limit
 * @note Refer to the excel document for a full list of test cases
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_POSITIVE_ACCUMULATION_SATURATION_1)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.5;
    pidObject.referencePoint = 50.4;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 3;

    float ret = 0;
    for (uint8_t i = 0; i < 50; i++)
    {
        ret = calc_pid_output(&pidObject, 50.3);
    }

    EXPECT_EQ((uint32_t)(ret * 1000), 3000);
}

/**
 * @brief Test whether if the memory of the integral is accumulating in the positive direction
 * @note Refer to the excel document for a full list of test cases
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_POSITIVE_ACCUMULATION_PARTITION_PHASE_1)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.75;
    pidObject.referencePoint = 3;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 100;

    float ret = 0;
    for (uint8_t i = 0; i < 10; i++)
    {
        ret = calc_pid_output(&pidObject, 0);
    }

    EXPECT_EQ((uint32_t)(ret * 1000), 42750);
}

/**
 * @brief Test whether if the memory of the integral is accumulating in the positive direction
 * @details Phase Stage
 * @note Refer to the excel document for a full list of test cases
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_POSITIVE_ACCUMULATION_PARTITION_PHASE_2)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.75;
    pidObject.referencePoint = 3;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 100;

    float ret = 0;
    for (uint8_t i = 0; i < 20; i++)
    {
        ret = calc_pid_output(&pidObject, 0);
    }

    EXPECT_EQ((uint32_t)(ret * 1000), 87750);
}

/**
 * @brief Test whether if the memory of the integral is accumulating in the positive direction
 * @details Phase Stage
 * @note Refer to the excel document for a full list of test cases
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_POSITIVE_ACCUMULATION_PARTITION_SATURATION_PHASE_1)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.75;
    pidObject.referencePoint = 3;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 100;

    float ret = 0;
    for (uint8_t i = 0; i < 30; i++)
    {
        ret = calc_pid_output(&pidObject, 0);
    }

    EXPECT_EQ((uint32_t)(ret * 1000), 100000);
}

/**
 * @brief Test whether if the memory of the integral is accumulating in the negative direction
 * @details Phase Stage
 * @note Refer to the excel document for a full list of test cases
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_NEGATIVE_ACCUMULATION_PARTITION_1)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.5;
    pidObject.referencePoint = 0;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 3;
    pidObject.previousOutput = 3;

    float ret = 0;
    for (uint8_t i = 0; i < 10; i++)
    {
        ret = calc_pid_output(&pidObject, 0.1);
    }

    EXPECT_EQ((uint32_t)(ret * 1000), 2050);
}

/**
 * @brief Test whether if the memory of the integral is accumulating in the negative direction
 * @details Phase Stage
 * @note Refer to the excel document for a full list of test cases
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_NEGATIVE_ACCUMULATION_PARTITION_2)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.5;
    pidObject.referencePoint = 0;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 3;
    pidObject.previousOutput = 3;

    float ret = 0;
    for (uint8_t i = 0; i < 20; i++)
    {
        ret = calc_pid_output(&pidObject, 0.1);
    }

    EXPECT_EQ((uint32_t)(ret * 1000), 1050);
}

/**
 * @brief Test whether if the memory of the integral is accumulating in the negative direction
 * @details Phase Stage
 * @note Refer to the excel document for a full list of test cases
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_NEGATIVE_ACCUMULATION_PARTITION_3)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.5;
    pidObject.referencePoint = 0;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 3;
    pidObject.previousOutput = 3;

    float ret = 0;
    for (uint8_t i = 0; i < 30; i++)
    {
        ret = calc_pid_output(&pidObject, 0.1);
    }

    EXPECT_EQ((uint32_t)(ret * 1000), 50

    );
}

/**
 * @brief boudary test for calcualting the current reference point using edge cases on reference point and operating point
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_POSITIVE_BOUNDARY_1)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 4;
    pidObject.kI = 0.5;
    pidObject.referencePoint = 50.4;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 3;

    float ret = 0;
    ret = calc_pid_output(&pidObject, 39.6);

    EXPECT_EQ((uint32_t)(ret * 1000), 3000);
}

/**
 * @brief boudary test for calcualting the phase output using edge cases on reference point and operating point
 *
 */
TEST(CALC_INTEGRAL, INTEGRAL_POSITIVE_BOUNDARY_2)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.75;
    pidObject.referencePoint = 3;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 100;

    float ret = 0;
    ret = calc_pid_output(&pidObject, 0);

    EXPECT_EQ((uint32_t)(ret * 1000), 2250);
}

/**
 * @brief Simple test to see whether if the sum of proportional and integral will saturate at the upper limit
 *
 */
TEST(CALC_PROPORTIONAL_INTEGRAL, SATURATION_POSITIVE_TEST)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 4;
    pidObject.kI = 0.5;
    pidObject.upperLimit = 3;
    pidObject.lowerLimit = 0;
    pidObject.referencePoint = 50.4;
    pidObject.error = 0;
    pidObject.previousError = 0;

    float ret = calc_pid_output(&pidObject, 42);

    EXPECT_EQ(pidObject.upperLimit, ret);
}

/**
 * @brief Simple test to see whether if the sum of proportional and integral will saturate at the upper limit
 *
 */
TEST(CALC_PROPORTIONAL_INTEGRAL, SATURATION_FAULT_INJECTION)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 4;
    pidObject.kI = 0.5;
    pidObject.upperLimit = 3;
    pidObject.lowerLimit = 0;
    pidObject.referencePoint = 50.4;
    pidObject.error = 0;
    pidObject.previousError = 0;

    float ret = calc_pid_output(&pidObject, -42);

    EXPECT_EQ(pidObject.upperLimit, ret);
}

/**
 * @brief Simple test to see whether if the sum of proportional and integral will saturate at the lower limit
 *
 */
TEST(CALC_PROPORTIONAL_INTEGRAL, SATURATION_NEGATIVE_TEST)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 4;
    pidObject.kI = 0.5;
    pidObject.upperLimit = 3;
    pidObject.lowerLimit = 0;
    pidObject.referencePoint = 50.4;
    pidObject.error = 0;
    pidObject.previousError = 0;

    float ret = calc_pid_output(&pidObject, 100);

    EXPECT_EQ(pidObject.lowerLimit, ret);
}

/**
 * @brief Simple test to see whether if the sum of proportional and integral will saturate at the lower limit
 *
 */
TEST(CALC_PROPORTIONAL_INTEGRAL, SATURATION_NEGATIVE_FAULT_INJECTION)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 4;
    pidObject.kI = 0.5;
    pidObject.upperLimit = 3;
    pidObject.lowerLimit = 0;
    pidObject.referencePoint = 50.4;
    pidObject.error = 0;
    pidObject.previousError = 0;

    float ret = calc_pid_output(&pidObject, 1000);

    EXPECT_EQ(pidObject.lowerLimit, ret);
}

/**
 * @brief Simple PID integration test executing only one single control loop
 *
 */
TEST(PID_INTEGRATION_TEST, SIMPLE_1_LOOP_TEST)
{
    PIDTypeDef_t voltageStage;
    PIDTypeDef_t currentStage;

    voltageStage = {
        .kI = 0.5,
        .KP = 4,
        .upperLimit = 3,
        .lowerLimit = 0,
        .referencePoint = 50.4,
        .previousError = 0,
        .previousOutput = 0,
    };

    currentStage = {
        .kI = 0.75,
        .KP = 0,
        .upperLimit = 100,
        .lowerLimit = 0,
        .referencePoint = 0,
        .previousError = 0,
        .previousOutput = 0,
    };

    float currentReference = calc_pid_output(&voltageStage, 39.6);

    EXPECT_EQ(currentReference, 3);

    currentStage.referencePoint = currentReference;
    float phase = calc_pid_output(&currentStage, 0);

    EXPECT_EQ(phase, 2.25);
}

/**
 * @brief Simple PID integration test executing only one single control loop
 *
 */
TEST(PID_INTEGRATION_TEST, SIMPLE_10_LOOP_TEST)
{
    PIDTypeDef_t voltageStage;
    PIDTypeDef_t currentStage;

    voltageStage = {
        .kI = 0.75,
        .KP = 4,
        .upperLimit = 3,
        .lowerLimit = 0,
        .referencePoint = 49.6,
        .previousError = 0,
        .previousOutput = 0,
    };

    currentStage = {
        .kI = 0.75,
        .KP = 0,
        .upperLimit = 100,
        .lowerLimit = 0,
        .referencePoint = 0,
        .previousError = 0,
        .previousOutput = 0,
    };

    float operatingPoint = 39.6;

    float phase = 0;
    float currentReference = 0;
    printf(ANSI_COLOR_YELLOW "=====================================\r\n");

    for (uint8_t i = 0; i < 10; i++)
    {
        currentReference = calc_pid_output(&voltageStage, (operatingPoint + (0.1 * i)));
        currentStage.referencePoint = currentReference;
        phase = calc_pid_output(&currentStage, 0);
        printf("current ref:%f , %f\r\n", currentReference, phase);
    }
    printf("=====================================\r\n");

    EXPECT_EQ(currentReference, 3);
    EXPECT_EQ(phase, 42.75);
}

/**
 * @brief Simple PID integration test executing only one single control loop
 *
 */
TEST(PID_INTEGRATION_TEST, SIMPLE_50_LOOP_TEST)
{
    PIDTypeDef_t voltageStage;
    PIDTypeDef_t currentStage;

    voltageStage = {
        .kI = 0.75,
        .KP = 4,
        .upperLimit = 3,
        .lowerLimit = 0,
        .referencePoint = 49.6,
        .previousError = 0,
        .previousOutput = 0,
    };

    currentStage = {
        .kI = 0.75,
        .KP = 0,
        .upperLimit = 100,
        .lowerLimit = 0,
        .referencePoint = 0,
        .previousError = 0,
        .previousOutput = 0,
    };

    float operatingPoint = 39.6;

    float phase = 0;
    float currentReference = 0;
    printf(ANSI_COLOR_YELLOW "=====================================\r\n");

    for (uint8_t i = 0; i < 50; i++)
    {
        currentReference = calc_pid_output(&voltageStage, (operatingPoint + (0.1 * i)));
        currentStage.referencePoint = currentReference;
        phase = calc_pid_output(&currentStage, 0);
        printf("current ref:%f , %f\r\n", currentReference, phase);
    }
    printf("=====================================\r\n");

    EXPECT_EQ(currentReference, 3);
    EXPECT_EQ(phase, 100);
}

/**
 * @brief This test is designed to simulate the transition from CC to CV charging as the battery voltage reaches 49.6V (4.1V/Cell)
 *
 */
TEST(PID_INTEGRATION_TEST, CC_CV_TRANSITION_TEST)
{
    PIDTypeDef_t voltageStage;
    PIDTypeDef_t currentStage;

    voltageStage = {
        .kI = 0.75,
        .KP = 4,
        .upperLimit = 3,
        .lowerLimit = 0,
        .referencePoint = 49.6,
        .previousError = 0.3,
        .previousOutput = 3,
    };

    currentStage = {
        .kI = 0.75,
        .KP = 0,
        .upperLimit = 100,
        .lowerLimit = 0,
        .referencePoint = 0,
        .previousError = 2.25,
        .previousOutput = 100,
    };

    float operatingPoint = 49.3;

    float phase = 0;
    float currentReference = 0;
    printf(ANSI_COLOR_YELLOW "=====================================\r\n");

    for (uint8_t i = 0; i < 10; i++)
    {
        currentReference = calc_pid_output(&voltageStage, (operatingPoint));
        currentStage.referencePoint = currentReference;
        phase = calc_pid_output(&currentStage, 3);
        printf("current ref:%f , %f\r\n", currentReference, phase);

        if (operatingPoint < 49.6)
        {
            operatingPoint += 0.1;
        }
    }
    printf("=====================================\r\n");

    EXPECT_EQ(uint32_t(currentReference * 1000), 1775);
    EXPECT_EQ(uint32_t(1000 * phase), 93269); // Theorectically 93.268% but just decimal rounding
}

/**
 * @brief This test injects negative voltage to the PID to check its response
 *
 */
TEST(FAULT_INJECTION, FAULT_NEGATIVE_VOLTAGE)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 4;
    pidObject.kI = 0.75;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 3;
    pidObject.referencePoint = 50.4;

    float currentReference = calc_pid_output(&pidObject, -3);
    EXPECT_EQ(currentReference, 3);
}

/**
 * @brief This test injects negative voltage and current to the PID to check its response
 *
 */
TEST(FAULT_INJECTION, FAULT_NEGATIVE_VOLTAGE_1)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 4;
    pidObject.kI = 0.75;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 3;
    pidObject.referencePoint = 50.4;

    PIDTypeDef_t currentStage;
    currentStage.kI = 0.5;
    currentStage.lowerLimit = 0;
    currentStage.upperLimit = 100;

    float currentReference = calc_pid_output(&pidObject, -3);

    currentStage.referencePoint = currentReference;

    float phase = calc_pid_output(&currentStage, -3);

    EXPECT_EQ(phase, 3);
}

/**
 * @brief This test injects negative voltage and current to the PID to check its response
 *
 */
TEST(INTEGRAL_RESET, INTEGRAL_RESET_1)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 4;
    pidObject.kI = 0.75;
    pidObject.lowerLimit = 0;
    pidObject.upperLimit = 3;
    pidObject.referencePoint = 50.4;

    float currentReference;

    for (uint8_t i = 0; i < 10; i++)
    {
        calc_pid_output(&pidObject, -3);
    }

    reset_pid_memory(&pidObject);
    EXPECT_EQ(pidObject.error, 0);
    EXPECT_EQ(pidObject.previousError, 0);
    EXPECT_EQ(pidObject.previousOutput, 0);
}