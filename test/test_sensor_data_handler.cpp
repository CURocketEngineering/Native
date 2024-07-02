#include "gtest/gtest.h"
#include "data_handling/SensorDataHandler.h"
#include "DataSaver_mock.h"

using namespace ::testing;

TEST(SensorDataHandler, test_data_point){
    DataPoint dp(100, 1.0);
    ASSERT_EQ(dp.timestamp_ms, 100);
    ASSERT_EQ(dp.data, 1.0);

    DataPoint dp2(200, 2.0);
    ASSERT_EQ(dp2.timestamp_ms, 200);
    ASSERT_EQ(dp2.data, 2.0);

    ASSERT_TRUE(dp < dp2);
    ASSERT_TRUE(dp <= dp2);
    ASSERT_FALSE(dp > dp2);
    ASSERT_FALSE(dp >= dp2);
}

TEST(SensorDataHandler, test_saving_data){
    DataSaverMock dsm;
    SensorDataHandler sdh ("test", &dsm);

    EXPECT_CALL(dsm, saveDataPoint(_, _)).Times(1);
    DataPoint dp(100, 1.0);
    ASSERT_EQ(sdh.addData(dp), 0);

    EXPECT_CALL(dsm, saveDataPoint(_, _)).Times(1);
    DataPoint dp2(200, 2.0);
    ASSERT_EQ(sdh.addData(dp2), 0);

    // Restricting save speed
    sdh.restrictSaveSpeed(100);
    EXPECT_CALL(dsm, saveDataPoint(_, _)).Times(0);
    DataPoint dp3(250, 3.0);
    ASSERT_EQ(sdh.addData(dp3), 0);

    EXPECT_CALL(dsm, saveDataPoint(_, _)).Times(0);
    DataPoint dp4(300, 3.0);
    ASSERT_EQ(sdh.addData(dp4), 0);

    EXPECT_CALL(dsm, saveDataPoint(_, _)).Times(1);
    DataPoint dp5(301, 3.0);
    ASSERT_EQ(sdh.addData(dp5), 0);

}














