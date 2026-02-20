// --- Standalone Pressure Transducer Test for Teensy 4.1 ---

#define PRESSURE_PIN 19 // Analog Pin A5

// Voltage divider configuration: 10k resistor from sensor, 20k resistor to ground
// This divides the voltage by 0.666 (Multiplier = 1.5 to get original voltage back)
const float DIVIDER_RATIO = 1.5; 
const float V_REF = 3.3; // Teensy logic level
const int ADC_RES = 1023; // Default 10-bit resolution

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10); 
  Serial.println("Starting 100 PSI Pressure Sensor Test...");
  pinMode(PRESSURE_PIN, INPUT);
}

void loop() {
  int rawADC = analogRead(PRESSURE_PIN);
  
  // 1. Convert raw ADC value to the actual voltage at the pin
  float pinVoltage = (rawADC / (float)ADC_RES) * V_REF;
  
  // 2. Multiply by the divider ratio to find what the sensor is actually outputting
  float sensorVoltage = pinVoltage * DIVIDER_RATIO;
  
  // 3. Convert voltage to PSI
  // The sensor outputs 0.5V at 0 PSI and 4.5V at 100 PSI (4.0V range)
  // 100 PSI / 4.0V = 25 PSI per volt
  float pressurePSI = (sensorVoltage - 0.5) * 25.0;
  
  // Filter out tiny negative numbers caused by electrical noise at 0 PSI
  if (pressurePSI < 0) pressurePSI = 0;

  Serial.printf("Raw ADC: %d | Pin: %.2fV | Sensor: %.2fV | Pressure: %.2f PSI\n", 
                rawADC, pinVoltage, sensorVoltage, pressurePSI);
                
  delay(250); // Read 4 times a second
}
