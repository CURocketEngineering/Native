#include "gtest/gtest.h"
#include "data_handling/CircularArray.h"
#include "data_handling/SensorDataHandler.h"

#include <gtest/gtest.h>

#include <iostream>


// Test the CircularArray class
TEST(CircularArrayTest, test_push) {
    CircularArray<int> *circularArray = new CircularArray<int>(5);
    EXPECT_EQ(circularArray->getMaxSize(), 5);
    EXPECT_FALSE(circularArray->isFull());
    EXPECT_EQ(circularArray->getHead(), 0);
    circularArray->push(1);
    EXPECT_FALSE(circularArray->isFull());

    // Head should still be 0 after one push
    EXPECT_EQ(circularArray->getHead(), 0);

    circularArray->push(2);
    // Head should be 1 after the second push
    // because the head is always where the last value was pushed to
    EXPECT_EQ(circularArray->getHead(), 1);

    circularArray->push(3);
    circularArray->push(4);
    circularArray->push(5);
    EXPECT_EQ(circularArray->getFromHead(0), 5);
    EXPECT_EQ(circularArray->getFromHead(1), 4);
    EXPECT_EQ(circularArray->getFromHead(2), 3);
    EXPECT_EQ(circularArray->getFromHead(3), 2);
    EXPECT_EQ(circularArray->getFromHead(4), 1);
    EXPECT_TRUE(circularArray->isFull());
}

// Fill the circular array with a lot of values and make sure it all looks good
TEST(CircularArrayTest, test_fill) {
    CircularArray<int> *circularArray = new CircularArray<int>(5);
    EXPECT_FALSE(circularArray->isFull());
    for (int i = 0; i < 100; i++) {
        circularArray->push(i);
        if (i == 0){
            EXPECT_EQ(circularArray->getHead(), 0);
        } else{
            EXPECT_EQ(circularArray->getHead(), i % circularArray->getMaxSize());
        }
    }
    EXPECT_EQ(circularArray->getFromHead(0), 99);
    EXPECT_EQ(circularArray->getFromHead(1), 98);
    EXPECT_EQ(circularArray->getFromHead(2), 97);
    EXPECT_EQ(circularArray->getFromHead(3), 96);
    EXPECT_EQ(circularArray->getFromHead(4), 95);
    EXPECT_TRUE(circularArray->isFull());
}

// Make a circular array as big as possible 
TEST(CircularArrayTest, test_max_size) {
    CircularArray<int> *circularArray = new CircularArray<int>(255);
    EXPECT_EQ(circularArray->getMaxSize(), 255);
    EXPECT_FALSE(circularArray->isFull());
    for (int i = 0; i < 1000; i++) {
        circularArray->push(i);
    }
    EXPECT_EQ(circularArray->getFromHead(0), 999);
    EXPECT_EQ(circularArray->getFromHead(1), 998);
    EXPECT_EQ(circularArray->getFromHead(2), 997);
    EXPECT_EQ(circularArray->getFromHead(3), 996);
    EXPECT_EQ(circularArray->getFromHead(4), 995);
    EXPECT_TRUE(circularArray->isFull());
}

// Test for wrapping behavior and data integrity
TEST(CircularArrayTest, test_wrapping_and_data_integrity) {
    CircularArray<int> *circularArray = new CircularArray<int>(10); // Small size for easy testing
    // Fill the array to its capacity
    for (int i = 0; i < 10; i++) {
        circularArray->push(i);
    }
    // Verify initial full state
    EXPECT_TRUE(circularArray->isFull());
    for (int i = 0; i < 10; i++) {
        EXPECT_EQ(circularArray->getFromHead(i), 9 - i); // Check order is correct
    }
    // Push more elements to force wrapping
    for (int i = 10; i < 20; i++) {
        circularArray->push(i);
    }
    // Verify elements after wrapping
    for (int i = 0; i < 10; i++) {
        EXPECT_EQ(circularArray->getFromHead(i), 19 - i); // New order after wrapping
    }
    EXPECT_TRUE(circularArray->isFull()); // Still full
    delete circularArray;
}

// Test getMedian
TEST(CircularArrayTest, test_get_median_odd) {
    CircularArray<int> *circularArray = new CircularArray<int>(5);
    circularArray->push(1);
    circularArray->push(2);
    circularArray->push(3);
    circularArray->push(4);
    circularArray->push(5);
    EXPECT_EQ(circularArray->getMedian(), 3);
    circularArray->push(6);
    EXPECT_EQ(circularArray->getMedian(), 4);
    circularArray->push(7);
    EXPECT_EQ(circularArray->getMedian(), 5);
    circularArray->push(8);
    EXPECT_EQ(circularArray->getMedian(), 6);
    circularArray->push(9);
    EXPECT_EQ(circularArray->getMedian(), 7);
    delete circularArray;
}

// Test getMedian
TEST(CircularArrayTest, test_get_median_even) {
    CircularArray<int> *circularArray = new CircularArray<int>(6);
    circularArray->push(1);
    circularArray->push(2);
    circularArray->push(3);
    circularArray->push(4);
    circularArray->push(5);
    circularArray->push(6);
    EXPECT_EQ(circularArray->getMedian(), 4);
    circularArray->push(7);
    EXPECT_EQ(circularArray->getMedian(), 5);
    circularArray->push(8);
    EXPECT_EQ(circularArray->getMedian(), 6);
    circularArray->push(9);
    EXPECT_EQ(circularArray->getMedian(), 7);
    circularArray->push(10);
    EXPECT_EQ(circularArray->getMedian(), 8);
    delete circularArray;
}

// Test clear
TEST(CircularArrayTest, test_clear) {
    CircularArray<int> *circularArray = new CircularArray<int>(5);
    circularArray->push(1);
    circularArray->push(2);
    circularArray->push(3);
    circularArray->push(4);
    circularArray->push(5);
    EXPECT_TRUE(circularArray->isFull());
    circularArray->clear();
    EXPECT_FALSE(circularArray->isFull());
    EXPECT_EQ(circularArray->getHead(), 0);
    delete circularArray;
}