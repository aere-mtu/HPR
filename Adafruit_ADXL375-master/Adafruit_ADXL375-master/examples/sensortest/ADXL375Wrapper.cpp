#include "ADXL375Wrapper.h"
#include <Arduino.h>
#include <Wire.h>  // Needed for I2C

// Constructor: default I2C, unique ID 12345
ADXL375Wrapper::ADXL375Wrapper() : accel(12345) {
}

bool ADXL375Wrapper::begin() {
    if (!accel.begin()) {
        Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
        return false;
    }
    return true;
}

void ADXL375Wrapper::getAcceleration(float &x, float &y, float &z) {
    sensors_event_t event;
    accel.getEvent(&event);
    x = event.acceleration.x;
    y = event.acceleration.y;
    z = event.acceleration.z;
}

void ADXL375Wrapper::displayDataRate() {
    Serial.print("Data Rate: ");
    switch (accel.getDataRate()) {
        case ADXL343_DATARATE_3200_HZ: Serial.print("3200 "); break;
        case ADXL343_DATARATE_1600_HZ: Serial.print("1600 "); break;
        case ADXL343_DATARATE_800_HZ:  Serial.print("800 "); break;
        case ADXL343_DATARATE_400_HZ:  Serial.print("400 "); break;
        case ADXL343_DATARATE_200_HZ:  Serial.print("200 "); break;
        case ADXL343_DATARATE_100_HZ:  Serial.print("100 "); break;
        case ADXL343_DATARATE_50_HZ:   Serial.print("50 "); break;
        case ADXL343_DATARATE_25_HZ:   Serial.print("25 "); break;
        case ADXL343_DATARATE_12_5_HZ: Serial.print("12.5 "); break;
        case ADXL343_DATARATE_6_25HZ:  Serial.print("6.25 "); break;
        case ADXL343_DATARATE_3_13_HZ: Serial.print("3.13 "); break;
        case ADXL343_DATARATE_1_56_HZ: Serial.print("1.56 "); break;
        case ADXL343_DATARATE_0_78_HZ: Serial.print("0.78 "); break;
        case ADXL343_DATARATE_0_39_HZ: Serial.print("0.39 "); break;
        case ADXL343_DATARATE_0_20_HZ: Serial.print("0.20 "); break;
        case ADXL343_DATARATE_0_10_HZ: Serial.print("0.10 "); break;
        default: Serial.print("???? "); break;
    }
    Serial.println(" Hz");
}

void ADXL375Wrapper::printSensorDetails() {
    accel.printSensorDetails();
}
