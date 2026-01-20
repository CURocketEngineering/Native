#include "unity.h"
#include "SimpleSimulation.h"   
#include "state_estimation/StateMachine.h"
#include "data_handling/DataPoint.h"
#include "DataSaver_mock.h"
#include "state_estimation/States.h"
#include "ArduinoHAL.h"  // if required for your platform

// Forward declaration of CSV test
void test_state_machine_with_real_data(void);

#include <fstream>
#include <iostream>  // for error messages (if needed)
#include <random>

// Use a mock Serial for debug prints in tests
MockSerial Serial;

DataSaverMock dataSaver;
IDataSaver* dataSaverPtr = &dataSaver;

//
// setUp() and tearDown()
//
void setUp(void) {
    Serial.clear();
}

void tearDown(void) {
    Serial.clear();
}

void test_init(){
    LaunchDetector lp(30, 1000, 40);
    ApogeeDetector ad;
    VerticalVelocityEstimator vve;
    StateMachine sm(dataSaverPtr, &lp, &ad, &vve);

    TEST_ASSERT_EQUAL(STATE_ARMED, sm.getState());
}

void test_launch(){
    LaunchDetector lp(30, 1000, 40);
    ApogeeDetector ad;
    VerticalVelocityEstimator vve;
    StateMachine sm(dataSaverPtr, &lp, &ad, &vve);

    // Start sim
    SimpleSimulator sim(10000, 70, 3000, 10);

    while (sim.getApogeeTimestamp() == 0) {
        sim.tick();
        DataPoint aclX(sim.getCurrentTime(), 0);
        DataPoint aclY(sim.getCurrentTime(), 0);

        // Adding 9.8 because the launchDetector expects measured acceleration
        // I.e. 0m/s^2 stationary on ground is measured as 9.8m/s^2 by the accelerometer
        DataPoint aclZ(sim.getCurrentTime(), sim.getIntertialVerticalAcl() + 9.8);
        DataPoint alt(sim.getCurrentTime(), sim.getAltitude());
        // std::cout << aclZ.data << "  mag: " << lp.getMedianAccelerationSquared() << " Ts: " << aclZ.timestamp_ms << std::endl;
        AccelerationTriplet accel = {aclX, aclY, aclZ};
        sm.update(accel, alt);
    }

    // Check that the launchDetector detected the launch
    TEST_ASSERT_TRUE(lp.isLaunched());

    // Check that the lp was within 1s of the actual launch time
    TEST_ASSERT_UINT32_WITHIN(500, sim.getLaunchTimestamp(), lp.getLaunchedTime());

    // Check that the state machine state is greater than ARMED
    TEST_ASSERT_GREATER_THAN(STATE_ARMED, sm.getState());
}

void test_apogee_detection(){
    LaunchDetector lp(30, 1000, 40);
    ApogeeDetector ad;
    VerticalVelocityEstimator vve;
    StateMachine sm(dataSaverPtr, &lp, &ad, &vve);

    // Start sim
    SimpleSimulator sim(3000, 70, 2000, 5);

    // init noise for acl and alt
    std::mt19937 gen(42); 
    std::normal_distribution<float> aclNoise(0, 5);
    std::normal_distribution<float> altNoise(0, 5);

    while (!sim.getHasLanded()) {
        sim.tick();
        DataPoint aclX(sim.getCurrentTime(), 0);
        DataPoint aclY(sim.getCurrentTime(), 0);

        // Subtracting 9.8 because the launchDetector expects measured acceleration
        // I.e. 0m/s^2 stationary on ground is measured as -9.8m/s^2 by the accelerometer
        DataPoint aclZ(sim.getCurrentTime(), sim.getIntertialVerticalAcl() + 9.8 + aclNoise(gen));
        DataPoint alt(sim.getCurrentTime(), sim.getAltitude() + altNoise(gen));
        AccelerationTriplet accel = {aclX, aclY, aclZ};
        sm.update(accel, alt);
    }

    // Check that the apogeeDetector detected the apogee
    TEST_ASSERT_TRUE(ad.isApogeeDetected());

    // Check that the state machine state is greater than ASCENT b/c apoge was detected and now descending
    TEST_ASSERT_GREATER_THAN(STATE_ASCENT, sm.getState());

    // Check that the ad was within .5s of the actual apogee time
    std::cout << "Sim apogee: " << sim.getApogeeTimestamp() << "  AD apogee: " << ad.getApogee().timestamp_ms << std::endl;
    TEST_ASSERT_UINT32_WITHIN(500, sim.getApogeeTimestamp(), ad.getApogee().timestamp_ms);
}

void test_apogee_detection_noise(){
    LaunchDetector lp(30, 1000, 40);
    ApogeeDetector ad;
    VerticalVelocityEstimator vve;
    StateMachine sm(dataSaverPtr, &lp, &ad, &vve);

    // Start sim
    SimpleSimulator sim(10000, 70, 3000, 10);

    while (!sim.getHasLanded()) {
        sim.tick();
        DataPoint aclX(sim.getCurrentTime(), 0);
        DataPoint aclY(sim.getCurrentTime(), 0);

        // Subtracting 9.8 because the launchDetector expects measured acceleration
        // I.e. 0m/s^2 stationary on ground is measured as -9.8m/s^2 by the accelerometer
        DataPoint aclZ(sim.getCurrentTime(), sim.getIntertialVerticalAcl() + 9.8);
        DataPoint alt(sim.getCurrentTime(), sim.getAltitude());
        // std::cout << aclZ.data << "  mag: " << lp.getMedianAccelerationSquared() << " Ts: " << aclZ.timestamp_ms << std::endl;
        AccelerationTriplet accel = {aclX, aclY, aclZ};
        sm.update(accel, alt);
    }

    // Check that the apogeeDetector detected the apogee
    TEST_ASSERT_TRUE(ad.isApogeeDetected());

    // Check that the state machine state is greater than ASCENT b/c apoge was detected and now descending
    TEST_ASSERT_GREATER_THAN(STATE_ASCENT, sm.getState());

    // Check that the ad was within 1s of the actual apogee time
    std::cout << "Sim apogee: " << sim.getApogeeTimestamp() << "  AD apogee: " << ad.getApogee().timestamp_ms << std::endl;
    TEST_ASSERT_UINT32_WITHIN(0, sim.getApogeeTimestamp(), ad.getApogee().timestamp_ms);
}

//
// Main: Run all tests
//
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_init);
    RUN_TEST(test_launch);
    RUN_TEST(test_apogee_detection);
    RUN_TEST(test_state_machine_with_real_data);
    return UNITY_END();
}
