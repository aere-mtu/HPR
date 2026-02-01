#ifndef ADXL375WRAPPER_H
#define ADXL375WRAPPER_H

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>

// Class to handle ADXL375 accelerometer
class ADXL375Wrapper {
public:
    ADXL375Wrapper();                                   // Constructor
    bool begin();                                       // Initialize sensor
    void getAcceleration(float &x, float &y, float &z); // Get latest acceleration
    void displayDataRate();                             // Print data rate to Serial
    void printSensorDetails();                          // Print sensor info

private:
    Adafruit_ADXL375 accel;                             // ADXL375 object
};

#endif
