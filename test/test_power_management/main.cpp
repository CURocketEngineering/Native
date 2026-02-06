// test_power_management/main.cpp
#include "unity.h"
#include "PowerManagement.h"
#include "ArduinoHAL.h"
#include <cmath>

// Simple analogRead mock with settable return value
static int g_analog_value = 0;

void setUp(void)  { g_analog_value = 0; }
void tearDown(void) {}

// Test 1: check numeric voltage conversion
void test_readvoltage(void)
{
    // 12-bit ADC mid value
    g_analog_value = 4095;
    const float factor = 134.33333f;
    BatteryVoltage bv(/*adcPin*/192, factor, /*numAdcBits*/12, /*voltageThreshold*/3.0f);

    const float measured = bv.readVoltage();
    const float vPin = (static_cast<float>(g_analog_value) / (static_cast<float>((1<<12) - 1))) * 3.3f;
    const float expected = vPin * factor;

    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected, measured);
}

// Test 2: isLow() returns true at 0V and false at full-scale
void test_isLow_behavior(void)
{
    // Zero reading -> 0V battery
    BatteryVoltage bv_low(193, 3.3f, 12, /*voltageThreshold*/0.1f);
    TEST_ASSERT_TRUE(bv_low.isLow());

    // Full-scale reading -> large battery voltage; threshold small -> not low
    BatteryVoltage bv_high(192, 3.3f, 12, /*voltageThreshold*/0.1f);
    TEST_ASSERT_FALSE(bv_high.isLow());
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_mid_scale_voltage);
    RUN_TEST(test_isLow_behavior);
    return UNITY_END();
}
