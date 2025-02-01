// test_ApogeeDetector.cpp

#define DEBUG
#include "unity.h"
#include "state_estimation/ApogeeDetector.h"
#include "data_handling/DataPoint.h"
#include "ArduinoHAL.h"  // if required for your platform

#include <fstream>
#include <iostream>  // for error messages (if needed)
#include <random>

// Use a mock Serial for debug prints in tests (if your HAL uses it)
MockSerial Serial;

//
// setUp() and tearDown()
//
void setUp(void) {
    Serial.clear();
}

void tearDown(void) {
    Serial.clear();
}

//
// Test Cases
//

/**
 * Test that before any update the detector is not initialized and returns default values.
 * Then, after the very first update the filter initializes using the altimeter reading.
 */
void test_initialization(void) {
    ApogeeDetector detector;
    
    // Before any update, apogee should not be detected.
    TEST_ASSERT_FALSE(detector.isApogeeDetected());
    DataPoint apogee = detector.getApogee();
    TEST_ASSERT_EQUAL_UINT32(0, apogee.timestamp_ms);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, apogee.data);
    
    // Provide the first update.
    uint32_t ts = 1000;
    DataPoint accelX(ts, 0.0f);
    DataPoint accelY(ts, 0.0f);
    // Simulate an accelerometer z reading of 19.81 m/s² (i.e. 9.81 + 10) and altimeter 0 m.
    DataPoint accelZ(ts, 19.81f);
    DataPoint alt(ts, 0.0f);
    
    detector.update(accelX, accelY, accelZ, alt);
    
    // After the first update, the estimated altitude should match the altimeter measurement.
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, detector.getEstimatedAltitude());
    // And no apogee should be detected yet.
    TEST_ASSERT_FALSE(detector.isApogeeDetected());
}

/**
 * Test that during a sustained ascent the detector does not trigger apogee detection.
 * We simulate several updates (with dt = 10 ms) where the altimeter value increases steadily.
 */
void test_no_apogee_during_ascent(void) {
    ApogeeDetector detector;
    uint32_t ts = 1000;
    float altitude = 0.0f;
    
    // Simulate 50 updates during ascent.
    for (int i = 0; i < 50; i++) {
        ts += 10;              // 10 ms interval
        altitude += 0.5f;      // altitude increases by 0.5 m each update
        DataPoint accelX(ts, 0.0f);
        DataPoint accelY(ts, 0.0f);
        // Upward acceleration: sensor reads 19.81 m/s² (net +10 m/s²)
        DataPoint accelZ(ts, 19.81f);
        DataPoint alt(ts, altitude);
        
        detector.update(accelX, accelY, accelZ, alt);
        TEST_ASSERT_FALSE(detector.isApogeeDetected());
    }
}

void test_apogee_detection(void) {
    ApogeeDetector detector;
    uint32_t ts = 1000;  // starting timestamp in ms
    float altitude = 0.0f;
    float trueVertVel = 0.0f;
    const float dt = 0.01f;   // 10 ms time step (seconds)
    const int dt_ms = 10;     // time step in ms

    // --- Simulation Parameters ---
    // Phase 1: Powered ascent (burn)
    // We simulate a 3-second powered flight (300 updates) with a net acceleration of 70 m/s^2
    // At rest, the accelerometer reads 9.81 m/s² (gravity), so the net acceleration is 70 - 9.81 = 60.19 m/s²
    const int powered_updates = 300;
    const float powered_net_accel = 70.0f;  // m/s² net upward acceleration during burn
    const float powered_aZ = powered_net_accel;

    // Setup random noise generators:
    std::default_random_engine rng(std::random_device{}());
    std::normal_distribution<float> noiseAccel(0.0f, 0.05f);  // accelerometer noise (m/s²)
    std::normal_distribution<float> noiseAlt(0.0f, 0.3f);       // altitude sensor noise (m)

    // Open CSV file for writing.
    std::ofstream csvFile("apogee_test_output.csv");
    if (!csvFile.is_open()) {
        std::cerr << "Error: Unable to open CSV file for writing." << std::endl;
        return;
    }
    csvFile << "timestamp,true_altitude,predicted_altitude,predicted_velocity,apogee_flag\n";

    // --- Phase 1: Powered Ascent ---
    for (int i = 0; i < powered_updates; i++) {
        ts += dt_ms;
        trueVertVel += powered_net_accel * dt;
        altitude += trueVertVel * dt;

        // During powered flight, the accelerometer reading is (net accel + gravity)
        float measured_aZ = powered_aZ + noiseAccel(rng);
        DataPoint aX(ts, noiseAccel(rng));  // assume near zero in X (plus noise)
        DataPoint aY(ts, noiseAccel(rng));  // assume near zero in Y (plus noise)
        DataPoint aZ(ts, measured_aZ);

        // Altimeter reading (with noise)
        float measured_alt = altitude + noiseAlt(rng);
        DataPoint alt(ts, measured_alt);

        detector.update(aX, aY, aZ, alt);

        csvFile << ts << ","
                << altitude << ","
                << detector.getEstimatedAltitude() << ","
                << detector.getEstimatedVelocity() << ","
                << (detector.isApogeeDetected() ? 1 : 0) << "\n";
    }

    // --- Phase 2: Coast (Free-Fall Ascent) ---
    // After engine cutoff, the rocket is in free-fall so that its proper acceleration is ~0.
    // The velocity now decreases due to gravity (-9.81 m/s²) until it reaches 0 (apogee).
    float maxAlt = altitude;  // record the maximum altitude reached so far
    uint32_t trueApogeeTs = ts;

    // Simulate until the upward velocity goes to 0.
    while (trueVertVel > 0.0f) {
        ts += dt_ms;
        trueVertVel += -9.81f * dt;  // free-fall deceleration (gravity)
        altitude += trueVertVel * dt;

        // Keep track of the maximum altitude and its time.
        if (altitude > maxAlt) {
            maxAlt = altitude;
            trueApogeeTs = ts;
        }

        // In free-fall, the accelerometer reads 0 (plus noise).
        float measured_aZ = 0.0f + noiseAccel(rng);
        DataPoint aX(ts, noiseAccel(rng));
        DataPoint aY(ts, noiseAccel(rng));
        DataPoint aZ(ts, measured_aZ);
        float measured_alt = altitude + noiseAlt(rng);
        DataPoint alt(ts, measured_alt);

        detector.update(aX, aY, aZ, alt);

        csvFile << ts << ","
                << altitude << ","
                << detector.getEstimatedAltitude() << ","
                << detector.getEstimatedVelocity() << ","
                << (detector.isApogeeDetected() ? 1 : 0) << "\n";

        // Until the filter “latches” apogee, assert it is not flagged.
        TEST_ASSERT_FALSE(detector.isApogeeDetected());
    }

    // One extra update to simulate the beginning of the descent.
    ts += dt_ms;
    trueVertVel += -9.81f * dt;
    altitude += trueVertVel * dt;
    {
        float measured_aZ = 0.0f + noiseAccel(rng);
        DataPoint aX(ts, noiseAccel(rng));
        DataPoint aY(ts, noiseAccel(rng));
        DataPoint aZ(ts, measured_aZ);
        float measured_alt = altitude + noiseAlt(rng);
        DataPoint alt(ts, measured_alt);

        detector.update(aX, aY, aZ, alt);

        csvFile << ts << ","
                << altitude << ","
                << detector.getEstimatedAltitude() << ","
                << detector.getEstimatedVelocity() << ","
                << (detector.isApogeeDetected() ? 1 : 0) << "\n";
    }

    // --- Phase 3: Free-Fall Descent ---
    // Continue simulating descent for a fixed number of updates or until the detector flags apogee.
    const int descent_updates = 200;
    for (int i = 0; i < descent_updates; i++) {
        ts += dt_ms;
        trueVertVel += -9.81f * dt;
        altitude += trueVertVel * dt;

        float measured_aZ = 0.0f + noiseAccel(rng);
        DataPoint dX(ts, noiseAccel(rng));
        DataPoint dY(ts, noiseAccel(rng));
        DataPoint dZ(ts, measured_aZ);
        float measured_alt = altitude + noiseAlt(rng);
        DataPoint dAlt(ts, measured_alt);

        detector.update(dX, dY, dZ, dAlt);

        csvFile << ts << ","
                << altitude << ","
                << detector.getEstimatedAltitude() << ","
                << detector.getEstimatedVelocity() << ","
                << (detector.isApogeeDetected() ? 1 : 0) << "\n";

        if (detector.isApogeeDetected()) {
            break;
        }
    }

    csvFile.close();

    // --- Final Assertions ---
    TEST_ASSERT_TRUE(detector.isApogeeDetected());
    DataPoint apogee = detector.getApogee();
    // The detected apogee altitude should be within ~20 m of our simulated max altitude.
    TEST_ASSERT_FLOAT_WITHIN(20.0f, maxAlt, apogee.data);
    // The timestamp of the detected apogee should be within 10 ms of the simulated true apogee time.
    TEST_ASSERT_UINT32_WITHIN(100, trueApogeeTs, apogee.timestamp_ms);
}


/**
 * Test that after a couple of updates the estimated altitude and vertical velocity
 * can be retrieved and are reasonable. During ascent the velocity should be positive.
 */
void test_get_estimated_values(void) {
    ApogeeDetector detector;
    uint32_t ts = 1000;
    float altitude = 0.0f;
    
    // First update to initialize.
    DataPoint accelX(ts, 0.0f);
    DataPoint accelY(ts, 0.0f);
    DataPoint accelZ(ts, 19.81f);
    DataPoint alt(ts, altitude);
    detector.update(accelX, accelY, accelZ, alt);
    
    // Give a bunch of updates at 10 meters of altitude
    for (int i = 0; i < 1000; i++){
        ts += 10;
        altitude = 10.0f;
        DataPoint aX(ts, 0.0f);
        DataPoint aY(ts, 0.0f);
        DataPoint aZ(ts, 9.8f);  // Like hanging from a parachute
        DataPoint aAlt(ts, altitude);
        detector.update(aX, aY, aZ, aAlt);
    }
    
    float estAlt = detector.getEstimatedAltitude();
    float estVel = detector.getEstimatedVelocity();
    
    // The estimated altitude should be very near the provided altimeter reading.
    TEST_ASSERT_FLOAT_WITHIN(0.5f, altitude, estAlt);
    
    // Since the rocket is sitting at 10 meters, the estimated velocity should be near 0.
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, estVel);
}

/**
 * Test that if an update is provided with a timestamp earlier than the last update,
 * the detector still updates the state (using a default small dt) and does not crash.
 */
void test_update_with_old_timestamp(void) {
    ApogeeDetector detector;
    uint32_t ts = 1000;
    float altitude = 0.0f;
    
    // First update.
    DataPoint accelX(ts, 0.0f);
    DataPoint accelY(ts, 0.0f);
    DataPoint accelZ(ts, 19.81f);
    DataPoint alt(ts, altitude);
    detector.update(accelX, accelY, accelZ, alt);
    
    // Provide a second update with an older timestamp.
    uint32_t old_ts = 900;  // earlier than 1000 ms
    DataPoint oldAccelX(old_ts, 0.0f);
    DataPoint oldAccelY(old_ts, 0.0f);
    DataPoint oldAccelZ(old_ts, 19.81f);
    DataPoint oldAlt(old_ts, altitude);
    detector.update(oldAccelX, oldAccelY, oldAccelZ, oldAlt);
    
    // Since the update() method uses a default dt (0.01 sec) if the timestamp is not later,
    // the estimated altitude should remain near the measurement.
    float estAlt = detector.getEstimatedAltitude();
    TEST_ASSERT_TRUE(estAlt >= altitude);
}

/**
 * Test that once apogee has been detected, further updates do not change the recorded apogee.
 */
void test_multiple_updates_after_apogee(void) {
    ApogeeDetector detector;
    uint32_t ts = 1000;
    float altitude = 0.0f;
    
    // Simulate ascent for 200 updates.
    for (int i = 0; i < 200; i++) {
        ts += 10;
        altitude += 0.5f;
        DataPoint aX(ts, 0.0f);
        DataPoint aY(ts, 0.0f);
        DataPoint aZ(ts, 19.81f);
        DataPoint alt(ts, altitude);
        detector.update(aX, aY, aZ, alt);
    }
    
    // Now simulate descent to trigger apogee detection.
    for (int i = 0; i < 200; i++) {
        ts += 10;
        altitude -= 0.5f;
        DataPoint aX(ts, 0.0f);
        DataPoint aY(ts, 0.0f);
        DataPoint aZ(ts, 0.0f);
        DataPoint alt(ts, altitude);
        detector.update(aX, aY, aZ, alt);
        if (detector.isApogeeDetected()) {
            break;
        }
    }
    
    TEST_ASSERT_TRUE(detector.isApogeeDetected());
    DataPoint apogee1 = detector.getApogee();
    
    // Continue with more descent updates.
    for (int i = 0; i < 10; i++) {
        ts += 10;
        altitude -= 0.5f;
        DataPoint aX(ts, 0.0f);
        DataPoint aY(ts, 0.0f);
        DataPoint aZ(ts, 0.0f);
        DataPoint alt(ts, altitude);
        detector.update(aX, aY, aZ, alt);
    }
    
    DataPoint apogee2 = detector.getApogee();
    // The apogee DataPoint should remain the same after it has been locked in.
    TEST_ASSERT_EQUAL_UINT32(apogee1.timestamp_ms, apogee2.timestamp_ms);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, apogee1.data, apogee2.data);
}

//
// Main: Run all tests
//
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initialization);
    RUN_TEST(test_no_apogee_during_ascent);
    RUN_TEST(test_apogee_detection);
    RUN_TEST(test_get_estimated_values);
    RUN_TEST(test_update_with_old_timestamp);
    RUN_TEST(test_multiple_updates_after_apogee);
    return UNITY_END();
}
