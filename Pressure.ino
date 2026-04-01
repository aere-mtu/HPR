//#include <Adafruit_LSM6DSOX.h>  //imports the library needed for the sensor

#include <Wire.h>


#define PRESSURE_PIN 18
const float DIVIDER_RATIO = 1.5; 
const float V_REF = 3.3; 
const int ADC_RES = 1023; 
float currentPressure = 0.0;
unsigned long launchTime; //measured in millisecs
unsigned long curTime; //measured in millisecs



// Forward declarations
void stop();

void readPressure();
void rocketStatus();


/*
This function runs ONCE at startup to initialize everything.
*/
void setup(void){

  // --- PRESSURE SENSOR SETUP ---
  pinMode(PRESSURE_PIN, INPUT);

  //allows us to use the serial moniotor for debugging
  Serial.begin(9600);

  //allows us to output data on pins 24 & 25
  Serial6.begin(9600);

  Serial.println("Yo wassup world");
  Wire.begin();

} 
void loop() {
  
  // --- READ PRESSURE ---
  readPressure();


                
  // --- PRINT PRESSURE ---
  Serial.printf("Pressure: %.2f\n", currentPressure); 

}
// --- PRESSURE SENSOR READ & CONVERT FUNCTION ---
void readPressure() {
  int rawADC = analogRead(PRESSURE_PIN);
  
  // Convert raw 10-bit analog reading (0-1023) to the voltage at the pin
  float pinVoltage = ((float)rawADC*V_REF)/ADC_RES;
  
  // Multiply by the divider ratio to find the actual voltage output of the sensor (V)
  float sensorVoltage = pinVoltage;
  
  // Apply the specific transfer function to convert voltage to actual pressure
  currentPressure = (sensorVoltage - 0.453) / 0.0231; 
}