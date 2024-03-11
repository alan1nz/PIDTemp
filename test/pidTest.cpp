#include "gtest/gtest.h"

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

    float ret = calc_pid_output(&pidObject, 44.4);
    EXPECT_FLOAT_EQ(uint32_t(ret * 1000), 1199); // rounding issue
}

/**
 * @brief Simple test to calculate negatice integral output
 */
TEST(CALC_INTEGRAL, INTEGRAL_NEGATIVE_TEST)
{
    PIDTypeDef_t pidObject;
    pidObject.KP = 0;
    pidObject.kI = 0.5;
    pidObject.referencePoint = 44.4;

    float ret = calc_pid_output(&pidObject, 46.8);
    EXPECT_FLOAT_EQ(int32_t(ret * 1000), -1199); // rounding issue
}

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