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
    MockDataSaver mockXAcl, mockYAcl, mockZAcl, altitude;
    uint8_t xAclName = 1;
    uint8_t yAclName = 2;
    uint8_t zAclName = 3;
    uint8_t altName = 4;
    SensorDataHandler xAclData(xAclName, &mockXAcl);
    SensorDataHandler yAclData(yAclName, &mockYAcl);
    SensorDataHandler zAclData(zAclName, &mockZAcl);
    SensorDataHandler altitudeData(altName, &altitude);

    SendableSensorData* ssds[] {
        new SendableSensorData(nullptr, (SensorDataHandler*[]) {&xAclData, &yAclData, &zAclData}, 3, 102, 2),
        new SendableSensorData(&altitudeData, nullptr, 0, 0, 1),
    };
    HardwareSerial mockRfdSerial;
    Telemetry telemetry(ssds, 2, mockRfdSerial);
}

//This does a lot of 
void test_a_full_second_of_ticks(void) {
    MockDataSaver mockXAcl, mockYAcl, mockZAcl, altitude;
    uint8_t xAclName = 1;
    uint8_t yAclName = 2;
    uint8_t zAclName = 3;
    uint8_t altName = 8;
    SensorDataHandler xAclData(xAclName, &mockXAcl);
    SensorDataHandler yAclData(yAclName, &mockYAcl);
    SensorDataHandler zAclData(zAclName, &mockZAcl);
    SensorDataHandler altitudeData(altName, &altitude);
    xAclData.addData(DataPoint(1, 6.767676f)); //01000000 11011000 10010000 11001101
    yAclData.addData(DataPoint(1, 6.969696f)); //01000000 11011111 00000111 11000000
    zAclData.addData(DataPoint(1, 1.234567f)); //00111111 10011110 00000110 01001011
    altitudeData.addData(DataPoint(1, 10000.0f)); //01000110 00011100 01000000 00000000
    uint8_t expectedSentBytes[34] = {0, 0, 0, 51, 0, 0, 0, 0, 102, 64, 216, 144, 205, 64, 223, 7, 192, 63, 158, 6, 75, 0, 0, 0, 51, 0, 0, 0, 0, 8, 70, 28, 64, 0};

    SendableSensorData* ssds[] {
        new SendableSensorData(nullptr, (SensorDataHandler*[]) {&xAclData, &yAclData, &zAclData}, 3, 102, 2),
        new SendableSensorData(&altitudeData, nullptr, 0, 0, 1)
    };
    HardwareSerial mockRfdSerial;
    Telemetry telemetry(ssds, 2, mockRfdSerial);

    int numPacketsSent = 0;
    for (int i = 0; i < 100; i++) {
        uint32_t timestamp = millis();
        bool didSendPacket = telemetry.tick();
        if (didSendPacket) {
            bool didSendPacketAtRightTime = ((timestamp >= 475 && timestamp <= 525) || numPacketsSent == 0) || ((timestamp >= 975 && timestamp <= 1025) || numPacketsSent == 1);
            TEST_ASSERT_EQUAL(didSendPacketAtRightTime, true);
            numPacketsSent++;
        }
        int too_fast = millis() - timestamp;
        if (too_fast < 10) {
            delay(10 - too_fast);
        }
        if (millis() >= 1100) break;
    }
    printf("\n");
    for (int byte : mockRfdSerial.writeCalls) {
        printf(" %03d", byte);
    }
    printf("\n");
    for (int byte : expectedSentBytes) {
        printf(" %03d", byte);
    }
    printf("\n");
    TEST_ASSERT_EQUAL(expectedSentBytes[3], mockRfdSerial.writeCalls.at(3)); //START 2hz
    TEST_ASSERT_EQUAL(expectedSentBytes[8], mockRfdSerial.writeCalls.at(8)); //ACCL
    TEST_ASSERT_EQUAL(expectedSentBytes[9], mockRfdSerial.writeCalls.at(9)); //X byte 1
    TEST_ASSERT_EQUAL(expectedSentBytes[10], mockRfdSerial.writeCalls.at(10)); //X byte 2
    TEST_ASSERT_EQUAL(expectedSentBytes[11], mockRfdSerial.writeCalls.at(11)); //X byte 3
    TEST_ASSERT_EQUAL(expectedSentBytes[12], mockRfdSerial.writeCalls.at(12)); //X byte 4

    TEST_ASSERT_EQUAL(expectedSentBytes[24], mockRfdSerial.writeCalls.at(24)); //START 1hz
    TEST_ASSERT_EQUAL(expectedSentBytes[29], mockRfdSerial.writeCalls.at(29)); //ALT

}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initialization);
    RUN_TEST(test_a_full_second_of_ticks);
    return UNITY_END();
}