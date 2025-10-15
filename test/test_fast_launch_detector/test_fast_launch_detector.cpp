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
    int ret = fld.update(accel1);
    TEST_ASSERT_EQUAL_INT(ALREADY_LAUNCHED, ret);
}

int main(){
    UNITY_BEGIN();
    RUN_TEST(test_already_launched);
}
