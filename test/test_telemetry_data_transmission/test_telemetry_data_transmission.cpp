#include "unity.h"
#include "data_handling/Telemetry.h"
#include "data_handling/DataPoint.h"
#include "data_handling/DataSaver.h"

MockSerial Serial;

void setUp(void) {
    Serial.clear();
}

void tearDown(void) {
    Serial.clear();
}

// ---------------------------------------------------------------------
// Mock IDataSaver Implementation - copied from test_sensor_data_handler.cpp
// ---------------------------------------------------------------------
class MockDataSaver : public IDataSaver {
public:
    struct SavedRecord {
        DataPoint data;
        uint8_t sensorName;
    };

    std::vector<SavedRecord> savedRecords;

    // This method will be called by SensorDataHandler.
    virtual int saveDataPoint(const DataPoint& data, uint8_t sensorName) override {
        SavedRecord record = { data, sensorName };
        savedRecords.push_back(record);
        return 0;
    }

    // Resets the record for reuse between tests.
    void reset() {
        savedRecords.clear();
    }
};

void test_initialization(void) {
    MockDataSaver mockXAcl, mockYAcl, mockZAcl, altitude, packetCounter;
    uint8_t xAclName = 1;
    uint8_t yAclName = 2;
    uint8_t zAclName = 3;
    uint8_t altName = 4;
    uint8_t packetCounterName = 5;
    SensorDataHandler xAclData(xAclName, &mockXAcl);
    SensorDataHandler yAclData(yAclName, &mockYAcl);
    SensorDataHandler zAclData(zAclName, &mockZAcl);
    SensorDataHandler altitudeData(altName, &altitude);
    SensorDataHandler numberSentPackets(packetCounterName, &packetCounter);

    SendableSensorData* ssds[] {
        new SendableSensorData(&numberSentPackets, nullptr, 0, 0, 2),
        new SendableSensorData(nullptr, (SensorDataHandler*[]) {&xAclData, &yAclData, &zAclData}, 3, 102, 2),
        new SendableSensorData(&altitudeData, nullptr, 0, 0, 1),
    };
    Stream mockRfdSerial;
    Telemetry telemetry(ssds, 3, mockRfdSerial);
}

//This does a lot of tests
void test_a_full_second_of_ticks(void) {
    MockDataSaver mockXAcl, mockYAcl, mockZAcl, altitude, packetCounter;
    uint8_t xAclName = 1;
    uint8_t yAclName = 2;
    uint8_t zAclName = 3;
    uint8_t altName = 8;
    uint8_t packetCounterName = 5;
    SensorDataHandler xAclData(xAclName, &mockXAcl);
    SensorDataHandler yAclData(yAclName, &mockYAcl);
    SensorDataHandler zAclData(zAclName, &mockZAcl);
    SensorDataHandler altitudeData(altName, &altitude);
    SensorDataHandler numberSentPackets(packetCounterName, &packetCounter);
    xAclData.addData(DataPoint(1, 6.767676f)); //01000000 11011000 10010000 11001101
    yAclData.addData(DataPoint(1, 6.969696f)); //01000000 11011111 00000111 11000000
    zAclData.addData(DataPoint(1, 1.234567f)); //00111111 10011110 00000110 01001011
    altitudeData.addData(DataPoint(1, 10000.0f)); //01000110 00011100 01000000 00000000
    numberSentPackets.addData(DataPoint(1, 1));
    uint8_t expectedSentBytes[61] = {0, 0, 0, 51, 0, 0, 0, 0, 5, 0, 0, 0, 0, 102, 64, 216, 144, 205, 64, 223, 7, 192, 63, 158, 6, 75, 0, 0, 0, 52, 0, 0, 0, 51, 0, 0, 0, 0, 5, 0, 0, 0, 0, 102, 64, 216, 144, 205, 64, 223, 7, 192, 63, 158, 6, 75, 8, 70, 28, 64, 0};
                                   //  START    | TIMESTAMP |NUM| sent pkts| ACCL|     FLOATX       |      FLOATY    |    FLOATZ     |    END     |   START    | TIMESTAMP |NUM|sent pkts| ACCL|     FLOATX       |      FLOATY    |    FLOATZ    |ALT| FLOAT_ALT   |

    SendableSensorData* ssds[] {
        new SendableSensorData(&numberSentPackets, nullptr, 0, 0, 2),
        new SendableSensorData(nullptr, (SensorDataHandler*[]) {&xAclData, &yAclData, &zAclData}, 3, 102, 2),
        new SendableSensorData(&altitudeData, nullptr, 0, 0, 1)
    };
    Stream mockRfdSerial;
    Telemetry telemetry(ssds, 3, mockRfdSerial);

    TEST_ASSERT_EQUAL(telemetry.tick((uint32_t)500), true);
    TEST_ASSERT_EQUAL(telemetry.tick((uint32_t)1000), true);

    printf("\n");
    for (int byte : mockRfdSerial.writeCalls) {
        printf(" %03d", byte);
    }
    printf("\n");
    printf("\n");
    for (int byte : expectedSentBytes) {
        printf(" %03d", byte);
    }
    printf("\n");
    TEST_ASSERT_EQUAL(expectedSentBytes[3], mockRfdSerial.writeCalls.at(3)); //START
    TEST_ASSERT_EQUAL(expectedSentBytes[8+14], mockRfdSerial.writeCalls.at(8+14)); //ACCL
    TEST_ASSERT_EQUAL(expectedSentBytes[9+14], mockRfdSerial.writeCalls.at(9+14)); //X byte 1
    TEST_ASSERT_EQUAL(expectedSentBytes[10+14], mockRfdSerial.writeCalls.at(10+14)); //X byte 2
    TEST_ASSERT_EQUAL(expectedSentBytes[11+14], mockRfdSerial.writeCalls.at(11+14)); //X byte 3
    TEST_ASSERT_EQUAL(expectedSentBytes[12+14], mockRfdSerial.writeCalls.at(12+14)); //X byte 4

    TEST_ASSERT_EQUAL(expectedSentBytes[24+14], mockRfdSerial.writeCalls.at(24+14)); //START 1hz
    TEST_ASSERT_EQUAL(expectedSentBytes[41+14], mockRfdSerial.writeCalls.at(41+14)); //ALT byte 1
    TEST_ASSERT_EQUAL(expectedSentBytes[42+14], mockRfdSerial.writeCalls.at(42+14)); //ALT byte 2
    TEST_ASSERT_EQUAL(expectedSentBytes[43+14], mockRfdSerial.writeCalls.at(43+14)); //ALT byte 3
    TEST_ASSERT_EQUAL(expectedSentBytes[44+14], mockRfdSerial.writeCalls.at(44+14)); //ALT byte 4

    printf("2 SECONDS:\n");
    mockRfdSerial.clearWriteCalls();
    TEST_ASSERT_EQUAL(telemetry.tick((uint32_t)1500), true);
    TEST_ASSERT_EQUAL(telemetry.tick((uint32_t)2000), true);

    printf("\n");
    for (int byte : mockRfdSerial.writeCalls) {
        printf(" %03d", byte);
    }
    printf("\n");
    printf("\n");
    for (int byte : expectedSentBytes) {
        printf(" %03d", byte);
    }
    printf("\n");
    TEST_ASSERT_EQUAL(expectedSentBytes[3], mockRfdSerial.writeCalls.at(3)); //START
    TEST_ASSERT_EQUAL(expectedSentBytes[8+14], mockRfdSerial.writeCalls.at(8+14)); //ACCL
    TEST_ASSERT_EQUAL(expectedSentBytes[9+14], mockRfdSerial.writeCalls.at(9+14)); //X byte 1
    TEST_ASSERT_EQUAL(expectedSentBytes[10+14], mockRfdSerial.writeCalls.at(10+14)); //X byte 2
    TEST_ASSERT_EQUAL(expectedSentBytes[11+14], mockRfdSerial.writeCalls.at(11+14)); //X byte 3
    TEST_ASSERT_EQUAL(expectedSentBytes[12+14], mockRfdSerial.writeCalls.at(12+14)); //X byte 4

    TEST_ASSERT_EQUAL(expectedSentBytes[24+14], mockRfdSerial.writeCalls.at(24+14)); //START 1hz
    TEST_ASSERT_EQUAL(expectedSentBytes[41+14], mockRfdSerial.writeCalls.at(41+14)); //ALT byte 1
    TEST_ASSERT_EQUAL(expectedSentBytes[42+14], mockRfdSerial.writeCalls.at(42+14)); //ALT byte 2
    TEST_ASSERT_EQUAL(expectedSentBytes[43+14], mockRfdSerial.writeCalls.at(43+14)); //ALT byte 3
    TEST_ASSERT_EQUAL(expectedSentBytes[44+14], mockRfdSerial.writeCalls.at(44+14)); //ALT byte 4
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initialization);
    RUN_TEST(test_a_full_second_of_ticks);
    return UNITY_END();
}