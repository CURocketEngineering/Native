#define DEBUG 
#include "gtest/gtest.h"
#include "data_handling/LaunchPredictor.h"

#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <iostream>

void fillWindow(LaunchPredictor &lp, float xac, float yac, float zac){
    CircularArray<DataPoint>* window = lp.getWindowPtr();
    uint32_t oldestTimestamp = window->getFromHead(0).timestamp_ms;

    for (int i = 0; i < window->getMaxSize(); i++){
        uint32_t timestamp = (i+1) * lp.getWindowInterval() + oldestTimestamp;
        DataPoint dx(timestamp, xac);
        DataPoint dy(timestamp, yac);
        DataPoint dz(timestamp, zac);
        lp.update(dx, dy, dz);
    }
}

TEST(LaunchPredictorTest, test_launch){
    LaunchPredictor lp(10, 100, 5);

    EXPECT_FALSE(lp.isLaunched());
    EXPECT_EQ(lp.getLaunchedTime(), 0);

    // Fill with low acceleration first
    fillWindow(lp, 1, 1, 1);

    EXPECT_FALSE(lp.isLaunched());

    // Fill with high acceleration
    fillWindow(lp, 10.1, 0, 0);

    EXPECT_TRUE(lp.isLaunched());
    // 150 b/c it is half way through the second time we fill the window
    EXPECT_EQ(lp.getLaunchedTime(), 150);

    // Reset
    lp.reset();
    // Check
    EXPECT_FALSE(lp.isLaunched());
    EXPECT_EQ(lp.getLaunchedTime(), 0);
    
    // Fill with components that don't add up to enough
    fillWindow(lp, 3, 4, 5); // 9 + 16 + 25 = 50 
    EXPECT_FALSE(lp.isLaunched()); // 50 < (10 * 10)

    // Fill with components that add up to enough
    fillWindow(lp, 8, 4, 5); // 64 + 16 + 25 = 105 > (10 * 10)
    EXPECT_TRUE(lp.isLaunched());
    EXPECT_EQ(lp.getLaunchedTime(), 150);

}

TEST(LaunchPredictorTest, test_update_rejection){
    // 100 ms window size, with 10 ms interval
    LaunchPredictor lp(10, 100, 10);

    EXPECT_FALSE(lp.isLaunched());
    EXPECT_EQ(lp.getLaunchedTime(), 0);

    

    EXPECT_FALSE(lp.update(DataPoint(0, 1), DataPoint(0, 1), DataPoint(0, 1)));
    
    uint32_t timestamp = lp.getWindowPtr()->getFromHead(0).timestamp_ms + lp.getWindowInterval();
    EXPECT_TRUE(lp.update(DataPoint(timestamp, 1), DataPoint(timestamp, 1), DataPoint(timestamp, 1)));

    // Try 2ms off which is 1/5 or 20% off
    timestamp = lp.getWindowPtr()->getFromHead(0).timestamp_ms + lp.getWindowInterval() + 2;
    EXPECT_FALSE(lp.update(DataPoint(timestamp, 1), DataPoint(timestamp, 1), DataPoint(timestamp, 1)));

    // Try being only 1 off which is 1/10 or 10% off
    timestamp = lp.getWindowPtr()->getFromHead(0).timestamp_ms + lp.getWindowInterval() + 1;
    EXPECT_TRUE(lp.update(DataPoint(timestamp, 1), DataPoint(timestamp, 1), DataPoint(timestamp, 1)));

    // Try being 1 off on the other side
    timestamp = lp.getWindowPtr()->getFromHead(0).timestamp_ms + lp.getWindowInterval() - 1;
    EXPECT_TRUE(lp.update(DataPoint(timestamp, 1), DataPoint(timestamp, 1), DataPoint(timestamp, 1)));
}
