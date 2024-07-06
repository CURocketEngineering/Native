#include "gtest/gtest.h"
#include "data_handling/CircularArray.h"
#include "data_handling/SensorDataHandler.h"

#include <gtest/gtest.h>

#include <iostream>

TEST(DPCircularArrayTest, test_median){
    CircularArray<DataPoint> *circularArray = new CircularArray<DataPoint>(5);
    EXPECT_EQ(circularArray->getMaxSize(), 5);
    EXPECT_FALSE(circularArray->isFull());
    EXPECT_EQ(circularArray->getHead(), 0);

    // **Timestamp, Data**
    circularArray->push(DataPoint(1, 1.0));

    // Check the median when not full, 0 b/c default value
    EXPECT_EQ(circularArray->getMedian().data, 0);

    circularArray->push(DataPoint(2, 2.0));
    circularArray->push(DataPoint(3, 3.0));
    circularArray->push(DataPoint(4, 4.0));
    circularArray->push(DataPoint(5, 5.0));

    EXPECT_EQ(circularArray->getMedian().data, 3.0);

    // Making sure that is collecting median based on the data, not the timestamp
    circularArray->push(DataPoint(6, 0.0));

    // The median is 3 because the data is 0, 1, 2, 3, 4
    EXPECT_EQ(circularArray->getMedian().data, 3.0);

    // Other misc checks
    EXPECT_EQ(circularArray->getFromHead(0).data, 0);
    EXPECT_EQ(circularArray->getFromHead(1).data, 5);
    EXPECT_EQ(circularArray->getFromHead(2).data, 4);
    EXPECT_EQ(circularArray->getFromHead(3).data, 3);
    EXPECT_EQ(circularArray->getFromHead(4).data, 2);
    EXPECT_TRUE(circularArray->isFull());
}