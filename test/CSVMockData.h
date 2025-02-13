#ifndef CSV_MOCK_DATA_H
#define CSV_MOCK_DATA_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdlib>

// Structure to hold sensor data parsed from the CSV
struct SensorData {
    long time;
    float accelx;
    float accely;
    float accelz;
    float gyrox;
    float gyroy;
    float gyroz;
    float magx;
    float magy;
    float magz;
    float altitude;
    float pressure;
    float temp;
};

// CSVDataProvider class reads and parses the CSV file.
class CSVDataProvider {
private:
    std::ifstream file;
    std::string currentLine;
public:
    CSVDataProvider(const std::string& filename) {
        file.open(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open CSV file: " << filename << std::endl;
            exit(EXIT_FAILURE);
        }
        // Skip header line
        getline(file, currentLine);
    }

    // Returns true if a new line is available.
    bool hasNext() {
        return static_cast<bool>(getline(file, currentLine));
    }

    // Splits the current line by commas and returns a vector of strings.
    std::vector<std::string> getRow() {
        std::vector<std::string> row;
        std::stringstream ss(currentLine);
        std::string cell;
        while (getline(ss, cell, ',')) {
            row.push_back(cell);
        }
        return row;
    }
};

// SensorMockWrapper converts a CSV row into SensorData structure.
class SensorMockWrapper {
public:
    static SensorData parseRow(const std::vector<std::string>& row) {
        if (row.size() < 13) {
            std::cerr << "Invalid row data: expected 13 columns, got " << row.size() << std::endl;
            exit(EXIT_FAILURE);
        }
        SensorData data;
        data.time    = stoll(row[0]);
        data.accelx  = stof(row[1]);
        data.accely  = stof(row[2]);
        data.accelz  = stof(row[3]);
        data.gyrox   = stof(row[4]);
        data.gyroy   = stof(row[5]);
        data.gyroz   = stof(row[6]);
        data.magx    = stof(row[7]);
        data.magy    = stof(row[8]);
        data.magz    = stof(row[9]);
        data.altitude= stof(row[10]);
        data.pressure= stof(row[11]);
        data.temp    = stof(row[12]);
        return data;
    }
};

#endif // CSV_MOCK_DATA_H
