#include "gtest/gtest.h"
#include "data_handling/DataSaverSPI.h"

#include <iostream>

// Setup for the test that creates a DataSaverSPI object
class DataSaverSPITest : public ::testing::Test {
    protected:
        DataSaverSPI* dss;
        void SetUp() override {
            dss = new DataSaverSPI(100, 1, 2, 3, 4);
        }

        void TearDown() override {
            delete dss;
        }
};

TEST_F(DataSaverSPITest, test_data_saver_spi){
    DataPoint dp(500, 1.0);
    dss->saveDataPoint(dp, 1);
    EXPECT_EQ(dss->getLastTimestamp(), 500);
    EXPECT_EQ(dss->getLastDataPoint().timestamp_ms, 500);
    EXPECT_EQ(dss->getLastDataPoint().data, 1.0);

    DataPoint dp2(550, 2.0); // Not enough time has passed
    dss->saveDataPoint(dp2, 1);

    // The timestamp should not have changed
    EXPECT_EQ(dss->getLastTimestamp(), 500);

    // The data point should have changed (they all get written when passed to the data saver)
    EXPECT_EQ(dss->getLastDataPoint().timestamp_ms, 550);
    EXPECT_EQ(dss->getLastDataPoint().data, 2.0);

    // Test the saveDataPoint overload
    dss->saveDataPoint(3.0, 600, 1);
    EXPECT_EQ(dss->getLastTimestamp(), 600);
    EXPECT_EQ(dss->getLastDataPoint().timestamp_ms, 600);
    EXPECT_EQ(dss->getLastDataPoint().data, 3.0);
}

TEST_F(DataSaverSPITest, test_past_points){
    DataPoint dp(500, 1.0);
    dss->saveDataPoint(dp, 1);
    EXPECT_EQ(dss->getLastTimestamp(), 500);
    EXPECT_EQ(dss->getLastDataPoint().timestamp_ms, 500);
    EXPECT_EQ(dss->getLastDataPoint().data, 1.0);

    DataPoint dp2(499, 2.0); // 1 ms younger than the last point
    dss->saveDataPoint(dp2, 1);
    EXPECT_EQ(dss->getLastTimestamp(), 500); // The timestamp should not have changed
    EXPECT_EQ(dss->getLastDataPoint().timestamp_ms, 499); // The data point should have changed
    EXPECT_EQ(dss->getLastDataPoint().data, 2.0);
}