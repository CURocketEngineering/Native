#define DEBUG 
#include "unity.h"
#include "state_estimation/FastLaunchDetector.h"
#include "data_handling/CircularArray.h"
#include "data_handling/DataPoint.h"
#include "ArduinoHAL.h" 
#include "state_estimation/StateEstimationTypes.h"

// Use a mock Serial for debug prints in tests
MockSerial Serial;

void setUp(void) {
    Serial.clear();
}

void tearDown(void) {
    Serial.clear();
}

/*
    Test that once launch is detected, further updates are ignored
*/
void test_already_launched(void){
    FastLaunchDetector fld(10.0);
    TEST_ASSERT_FALSE(fld.hasLaunched());
    //give point that exceeds threshold
    DataPoint dp1(1000, 100.0);
    AccelerationTriplet accel1 = { dp1, dp1, dp1 };
    fld.update(accel1);
    TEST_ASSERT_TRUE(fld.hasLaunched());
    //new update
    int ret = fld.update(accel1);
    TEST_ASSERT_EQUAL_INT(ALREADY_LAUNCHED, ret);
}

/*
    Test that resetting fld will reset launched and launchtime vars
*/
void test_reset(void){
    FastLaunchDetector fld(10.0);
    DataPoint dp1(1000, 100.0);
    AccelerationTriplet accel1 = { dp1, dp1, dp1 };
    fld.update(accel1);
    TEST_ASSERT_TRUE(fld.hasLaunched());

    fld.reset();
    TEST_ASSERT_FALSE(fld.hasLaunched());
    TEST_ASSERT_EQUAL_UINT32(0, fld.getLaunchedTime());
}

/**
 * Test that switching from below threshold to above threshold in subsequent updates
 * leads to launch detection.
 */
void test_acceleration_edge_case(void) {
    FastLaunchDetector fld(10.0);
    // First update with value just below threshold.
    DataPoint dp1(1000, 9.9);
    DataPoint dp2(1000, 0.0);     
    AccelerationTriplet accel1 = { dp1, dp2, dp2 };
    fld.update(accel1); // Total MAG is only 9.9^2 = 98.01 which is less than 100
    TEST_ASSERT_FALSE(fld.hasLaunched());
    
    // Then update with value just above threshold.
    DataPoint dp3(1000, 10.1);     
    AccelerationTriplet accel2 = { dp3, dp2, dp2 }; 
    fld.update(accel2);// Total MAG is 10.1^2 = 102.01 which is more than 100
    TEST_ASSERT_TRUE(fld.hasLaunched());
}

int main(){
    UNITY_BEGIN();
    RUN_TEST(test_already_launched);
    RUN_TEST(test_reset);
    RUN_TEST(test_acceleration_edge_case);
}
