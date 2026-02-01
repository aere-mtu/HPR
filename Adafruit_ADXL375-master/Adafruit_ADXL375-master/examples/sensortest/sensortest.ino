#include <Arduino.h>
#include "ADXL375Wrapper.h"

ADXL375Wrapper myAccel;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("ADXL375 Accelerometer Test");

    if (!myAccel.begin()) {
        while (1); // Halt if sensor not found
    }

    myAccel.printSensorDetails();
    myAccel.displayDataRate();
}

void loop() {
    float x, y, z;
    myAccel.getAcceleration(x, y, z);

    Serial.print("X: "); Serial.print(x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(y); Serial.print("  ");
    Serial.print("Z: "); Serial.println(z);
    
    delay(500);
}
