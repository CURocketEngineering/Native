#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>
#include "unity.h"
#include "state_estimation/BurnoutStateMachine.h"
#include "state_estimation/LaunchPredictor.h"
#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/VerticalVelocityEstimator.h"
#include "data_handling/DataPoint.h"
#include "DataSaver_mock.h"
#include "../CSVMockData.h"

// launch @ 3675568
// burnout @ 3677343 | 1436761
// apogee @ - | 1444561
// - 3674269.348155

/**
 * Test the BurnoutStateMachine using real flight data from CSV.
 * This test reads sensor data from a CSV file, feeds it into the state machine,
 * and logs the results including launch and apogee predictions.
 */
void test_burnout_state_machine_with_real_data(void) {
    // Create CSV provider to read test data with 25Hz sample rate (40ms interval)
    CSVDataProvider provider("data/MARTHA_3-8_1.3_B2_SingleID_transformed.csv", 25.0f);
    
    // Initialize components
    LaunchPredictor lp(30, 1000, 40);  // threshold, window, min samples
    ApogeeDetector ad;
    VerticalVelocityEstimator vve;
    DataSaverMock dataSaver;
    BurnoutStateMachine sm(&dataSaver, &lp, &ad, &vve);
    
    bool hasData = false;
    float maxAltitude = -1000.0f;
    uint32_t maxAltitudeTime = 0;
    uint32_t launchTime = 0;
    
    // Process each data point at 25Hz
    while (provider.hasNextDataPoint()) {
        hasData = true;
        SensorData data = provider.getNextDataPoint();
        
        // Create DataPoints for the state machine
        DataPoint accelX(data.time, data.accelx);
        DataPoint accelY(data.time, data.accely);
        DataPoint accelZ(data.time, data.accelz);
        DataPoint alt(data.time, data.altitude);
        
        // Update state machine with sensor data
        sm.update(accelX, accelY, accelZ, alt);
        
        // Track maximum altitude for validation
        if (data.altitude > maxAltitude) {
            maxAltitude = data.altitude;
            maxAltitudeTime = data.time;
        }
        
        // Track launch time
        if (lp.isLaunched() && launchTime == 0) {
            launchTime = lp.getLaunchedTime();
        }
        
        // Write results to CSV
        // Only output apogee predictions after launch is detected
        float estAlt = 0.0f;
        float estVel = 0.0f;
        if (sm.getState() >= STATE_POWERED_ASCENT) {
            estAlt = vve.getEstimatedAltitude();
            estVel = vve.getEstimatedVelocity();
        }
    }
    
    // Verify test data was processed
    TEST_ASSERT_TRUE(hasData);
    
    // Verify launch was detected
    TEST_ASSERT_TRUE(lp.isLaunched());
    
    // Verify apogee was detected
    TEST_ASSERT_TRUE(ad.isApogeeDetected());
    
    // Verify state progression
    TEST_ASSERT_GREATER_THAN(STATE_ARMED, sm.getState());
    TEST_ASSERT_GREATER_THAN(STATE_POWERED_ASCENT, sm.getState());
    TEST_ASSERT_GREATER_THAN(STATE_COAST_ASCENT, sm.getState()); // ends with descent
}
