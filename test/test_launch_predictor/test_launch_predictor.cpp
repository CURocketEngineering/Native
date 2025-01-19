#define DEBUG 
#include "unity.h"
#include "data_handling/LaunchPredictor.h"
#include "data_handling/CircularArray.h"
#include "data_handling/DataPoint.h"
#include "ArduinoHAL.h" 

// Use a mock Serial for debug prints in tests
MockSerial Serial;

void setUp(void) {
    Serial.clear();
}

void tearDown(void) {
    Serial.clear();
}

// Convenience: Retrieve window’s max size from the predictor’s circular array.
static inline uint16_t getWindowMaxSize(LaunchPredictor &lp) {
    return lp.getWindowPtr()->getMaxSize();
}

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * Fills the LaunchPredictor window with updates having a constant 
 * time interval (`delta_t_ms`) between data points and constant acceleration
 * components. The timestamp of the first update is given by initialTime.
 *
 * Each update will be called in sequence.
 */
void fillWindowWithInterval(LaunchPredictor &lp, uint32_t initialTime, uint16_t delta_t_ms,
                            float x_val, float y_val, float z_val)
{
    uint16_t maxSize = getWindowMaxSize(lp);
    // For the first update, we assume the predictor’s window might be empty.
    // We perform maxSize updates to fill the window.
    for (uint16_t i = 0; i < maxSize; i++) {
        uint32_t ts = initialTime + i * delta_t_ms;
        DataPoint dp(ts, x_val);
        DataPoint dp_y(ts, y_val);
        DataPoint dp_z(ts, z_val);
        // Note: During the filling phase, we expect LP_INITIAL_POPULATION
        int ret = lp.update(dp, dp_y, dp_z);
        // If the predictor has not yet been fully populated, it should return LP_INITIAL_POPULATION.
        if (!lp.getWindowPtr()->isFull()) {
            TEST_ASSERT_EQUAL_INT(LP_INITIAL_POPULATION, ret);
        }
    }
}

/**
 * A helper which fills the window with valid updates at the default windowInterval.
 * Uses the default delta_t = windowInterval_ms.
 */
void fillWindow(LaunchPredictor &lp, float x_val, float y_val, float z_val)
{
    uint16_t delta = lp.getWindowInterval();
    // Choose an arbitrary starting timestamp.
    uint32_t start = lp.getWindowPtr()->getFromHead(0).timestamp_ms + delta;
    fillWindowWithInterval(lp, start, delta, x_val, y_val, z_val);
}

// =============================================================================
// Test Cases
// =============================================================================

/**
 * Test that while the window is not full, update() returns LP_INITIAL_POPULATION
 * and once the window is full it starts checking for launch condition.
 */
void test_initial_population(void) {
    LaunchPredictor lp(10.0, 100, 5);
    uint16_t maxSize = getWindowMaxSize(lp);
    uint32_t start = 1000;
    for (uint16_t i = 0; i < maxSize; i++) {
        uint32_t ts = start + i * lp.getWindowInterval();
        DataPoint dp(ts, 1.0);
        DataPoint dp_y(ts, 1.0);
        DataPoint dp_z(ts, 1.0);
        int ret = lp.update(dp, dp_y, dp_z);
        if (i < maxSize - 1) {
            TEST_ASSERT_EQUAL_INT(LP_INITIAL_POPULATION, ret);
            TEST_ASSERT_FALSE(lp.isLaunched());
        }
    }
    // One more update once window is full will get past the population stage.
    uint32_t ts = start + maxSize * lp.getWindowInterval();
    DataPoint dp(ts, 1.0);
    DataPoint dp_y(ts, 1.0);
    DataPoint dp_z(ts, 1.0);
    int ret = lp.update(dp, dp_y, dp_z);
    // Since the median value (acc^2 = 3) is likely below the threshold (10^2 = 100),
    // expect LP_ACL_TOO_LOW.
    TEST_ASSERT_EQUAL_INT(LP_ACL_TOO_LOW, ret);
    TEST_ASSERT_FALSE(lp.isLaunched());
}

/**
 * Test that once a launch is detected the predictor ignores further updates.
 */
void test_already_launched(void) {
    LaunchPredictor lp(30.0, 100, 5);
    TEST_ASSERT_FALSE(lp.isLaunched());
    
    // Fill the window to get past the initial population stage.
    fillWindow(lp, 10.0, 0, 0);

    uint32_t newestTime = lp.getWindowPtr()->getFromHead(0).timestamp_ms + lp.getWindowInterval();

    // fill half the window with values above the threshold
    for (uint16_t i = 0; i < getWindowMaxSize(lp) / 2; i++) {
        uint32_t ts = newestTime + i * lp.getWindowInterval();
        DataPoint dp(ts, 100.0);

        int ret = lp.update(dp, dp, dp);
        // Expect LP_ACL_TOO_LOW until the median value is above the threshold.
        if (i < getWindowMaxSize(lp) / 2 - 1) {
            TEST_ASSERT_EQUAL_INT(LP_ACL_TOO_LOW, ret);
            TEST_ASSERT_FALSE(lp.isLaunched());
        }
    }

    TEST_ASSERT_TRUE(lp.isLaunched());
    uint32_t newTime = 10000;
    DataPoint dp(newTime, 20.0);
    DataPoint dp_y(newTime, 20.0);
    DataPoint dp_z(newTime, 20.0);
    int ret = lp.update(dp, dp_y, dp_z);
    TEST_ASSERT_EQUAL_INT(LP_ALREADY_LAUNCHED, ret);
}

/**
 * Test update() returns LP_YOUNGER_TIMESTAMP when provided a timestamp
 * that is older than the head of the current window.
 */
void test_update_with_early_timestamp(void) {
    LaunchPredictor lp(10.0, 100, 5);
    // Do one update to populate at least one data point.
    DataPoint dp1(1000, 1.0);
    lp.update(dp1, dp1, dp1);
    // Provide an update with an earlier timestamp.
    DataPoint dp2(900, 1.0);
    int ret = lp.update(dp2, dp2, dp2);
    TEST_ASSERT_EQUAL_INT(LP_YOUNGER_TIMESTAMP, ret);
}

/**
 * Test that an update coming in too soon (faster than the minimum allowed interval)
 * is rejected with LP_DATA_TOO_FAST.
 *
 * To simulate this, we first fill the window (so that the full branch of update() is taken),
 * then supply a new update with a timestamp delta smaller than (windowInterval - 20%).
 */
void test_update_too_fast(void) {
    LaunchPredictor lp(10.0, 100, 5);
    // Fill the window with valid data at the nominal interval.
    fillWindow(lp, 1.0, 1.0, 1.0);
    // Get the current most-recent timestamp (head of the window)
    uint32_t headTime = lp.getWindowPtr()->getFromHead(0).timestamp_ms;
    // For windowInterval = 5, twenty percent is 1 (5*0.2).
    // Data arriving with a difference less than 5 - 1 = 4 should trigger LP_DATA_TOO_FAST.
    uint32_t tooFastTime = headTime + 3; // 3 < 4
    DataPoint dp(tooFastTime, 1.0);
    DataPoint dp_y(tooFastTime, 1.0);
    DataPoint dp_z(tooFastTime, 1.0);
    int ret = lp.update(dp, dp_y, dp_z);
    TEST_ASSERT_EQUAL_INT(LP_DATA_TOO_FAST, ret);
}

/**
 * Test that an update that comes too late (beyond the allowed maximum gap)
 * causes the window to clear and returns LP_WINDOW_DATA_STALE.
 */
void test_update_window_data_stale(void) {
    LaunchPredictor lp(10.0, 100, 5);
    // Fill the window first.
    fillWindow(lp, 1.0, 1.0, 1.0);
    // Get current head timestamp.
    uint32_t headTime = lp.getWindowPtr()->getFromHead(0).timestamp_ms;
    // With windowInterval = 5 and twenty percent = 1,
    // updates delayed by more than 5+1 = 6 ms are too late.
    uint32_t staleTime = headTime + 7;  // 7 > 6, should be too late.
    DataPoint dp(staleTime, 10.0);
    DataPoint dp_y(staleTime, 10.0);
    DataPoint dp_z(staleTime, 10.0);
    int ret = lp.update(dp, dp_y, dp_z);
    TEST_ASSERT_EQUAL_INT(LP_WINDOW_DATA_STALE, ret);
    
    // After a stale update the window should have been cleared.
    // A subsequent update will be treated as initial population.
    DataPoint dp2(staleTime + 5, 10.0);
    DataPoint dp2_y(staleTime + 5, 10.0);
    DataPoint dp2_z(staleTime + 5, 10.0);
    int ret2 = lp.update(dp2, dp2_y, dp2_z);
    TEST_ASSERT_EQUAL_INT(LP_INITIAL_POPULATION, ret2);
}

/**
 * Test the condition when the overall time range of the window is too small.
 *
 * We simulate this by filling the window with data at a slightly faster rate than expected.
 * For example, for windowInterval=5 ms with 20 slots (100/5 = 20), the ideal range is 95 ms.
 * If we use a delta less than 5 (but still acceptable by the DATA_TOO_FAST check), the overall
 * time range will be below the minimum (which is windowSize_ms - 10%).
 *
 * For example: For windowSize_ms = 100, min_window_size_ms = 90.
 * If we use a delta of 4 ms, then total time range = 4*(maxSize-1).
 */
void test_window_time_range_too_small(void) {
    LaunchPredictor lp(10.0, 100, 5);
    // Use a delta that is exactly at the lower bound allowed.
    // For windowInterval 5, twenty percent is 1 => minimum allowed delta = 5 - 1 = 4.
    // With maxSize = 100/5 = 20, time range = 4*(20-1) = 76 which is below 90.
    uint32_t start = 1000;
    fillWindowWithInterval(lp, start, 4, 10.0, 10.0, 10.0);
    
    // After the window is full, the update that pushed the last value should have
    // returned LP_WINDOW_TIME_RANGE_TOO_SMALL.
    // (Since our helper does not capture the return of the final call,
    // we can do an extra update with an allowed delta and check the return.)
    // Use a valid delta so that DATA_TOO_FAST is not triggered.
    uint32_t headTime = lp.getWindowPtr()->getFromHead(0).timestamp_ms;
    uint32_t validTime = headTime + 4;  // exactly at lower bound.
    DataPoint dp(validTime, 10.0);
    DataPoint dp_y(validTime, 10.0);
    DataPoint dp_z(validTime, 10.0);
    int ret = lp.update(dp, dp_y, dp_z);
    TEST_ASSERT_EQUAL_INT(LP_WINDOW_TIME_RANGE_TOO_SMALL, ret);
    TEST_ASSERT_FALSE(lp.isLaunched());
}

/**
 * Test that when the median of the window is below the threshold the predictor
 * does not trigger a launch.
 */
void test_median_acceleration_below_threshold(void) {
    // Use a threshold of 10 m/s^2; squared threshold = 100.
    LaunchPredictor lp(10.0, 100, 5);
    // Fill the window with low acceleration values.
    fillWindow(lp, 1.0, 1.0, 1.0);
    // The median of [1,1,1] per axis results in acceleration squared = 1^2+1^2+1^2 = 3
    TEST_ASSERT_FALSE(lp.isLaunched());
}

/**
 * Test that when the median of the window is above the threshold the launch is detected.
 */
void test_median_acceleration_above_threshold(void) {
    LaunchPredictor lp(10.0, 100, 5);
    // Fill the window with high acceleration values.
    fillWindow(lp, 10.0, 10.0, 10.0);

    // Add one more update with high acceleration.
    uint32_t newTime = lp.getWindowPtr()->getFromHead(0).timestamp_ms + lp.getWindowInterval();
    DataPoint dp(newTime, 100.0);
    int result = lp.update(dp, dp, dp);
    TEST_ASSERT_EQUAL_INT(LP_LAUNCH_DETECTED, result);

    // The median acceleration squared is much higher than 10^2.
    TEST_ASSERT_TRUE(lp.isLaunched());
    TEST_ASSERT_NOT_EQUAL(0, lp.getLaunchedTime());
}

/**
 * Test that switching from below threshold to above threshold in subsequent updates
 * leads to launch detection.
 */
void test_median_acceleration_edge_case(void) {
    LaunchPredictor lp(10.0, 100, 5);
    // First fill with values just below threshold.
    fillWindow(lp, 9.9, 0, 0); // Total MAG is only 9.9^2 = 98.01 which is less than 100
    TEST_ASSERT_FALSE(lp.isLaunched());
    
    // Then fill with values just above threshold.
    // Note: This may require more than one update because the window median takes time to shift.
    fillWindow(lp, 10.1, 0, 0); // Total MAG is 10.1^2 = 102.01 which is more than 100
    TEST_ASSERT_TRUE(lp.isLaunched());
}

/**
 * Test that if the window is not full, the median is not checked and the predictor
 * does not trigger a launch.
 */
void test_window_not_full(void) {
    LaunchPredictor lp(10.0, 100, 5);
    // Do a single update.
    DataPoint dp(1000, 1.0);
    TEST_ASSERT_EQUAL_INT(LP_INITIAL_POPULATION, lp.update(dp, dp, dp));
    TEST_ASSERT_FALSE(lp.isLaunched());
}

/**
 * Test that resetting the predictor clears the launch flag and the window.
 */
void test_reset(void) {
    LaunchPredictor lp(10.0, 100, 5);
    // Fill the window to trigger launch.
    fillWindow(lp, 10.0, 0.0, 0.0);
    fillWindow(lp, 20.0, 0.0, 0.0);
    TEST_ASSERT_TRUE(lp.isLaunched());
    
    // Reset the predictor.
    lp.reset();
    TEST_ASSERT_FALSE(lp.isLaunched());
    TEST_ASSERT_EQUAL_UINT32(0, lp.getLaunchedTime());
    
    // After reset, a new update should be treated as initial population.
    DataPoint dp(5000, 10.0);
    TEST_ASSERT_EQUAL_INT(LP_INITIAL_POPULATION, lp.update(dp, dp, dp));
}

// =============================================================================
// Main
// =============================================================================

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_population);
    RUN_TEST(test_already_launched);
    RUN_TEST(test_update_with_early_timestamp);
    RUN_TEST(test_update_too_fast);
    RUN_TEST(test_update_window_data_stale);
    RUN_TEST(test_window_time_range_too_small);
    RUN_TEST(test_median_acceleration_below_threshold);
    RUN_TEST(test_median_acceleration_above_threshold);
    RUN_TEST(test_median_acceleration_edge_case);
    RUN_TEST(test_window_not_full);
    RUN_TEST(test_reset);
    return UNITY_END();
}
