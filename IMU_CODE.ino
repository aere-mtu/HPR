#include <Wire.h>   // Enables I²C communication

// I²C address of the MPU‑92xx IMU
#define MPU_ADDR 0x68

// Important IMU register addresses
#define REG_PWR_MGMT_1   0x6B      // Power management register
#define REG_ACCEL_XOUT_H 0x3B      // Starting register for accelerometer data
#define REG_GYRO_XOUT_H  0x43      // Starting register for gyroscope data

// Structure to store raw accelerometer and gyroscope readings
struct IMUData {
  int16_t ax, ay, az;   // Accelerometer X, Y, Z
  int16_t gx, gy, gz;   // Gyroscope X, Y, Z
};

IMUData imu;  // Create an IMU data object

// ------------------------------------------------------------
// Writes a single byte to a specific IMU register
// ------------------------------------------------------------
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);  // Begin communication with IMU
  Wire.write(reg);                   // Select register
  Wire.write(value);                 // Write value to register
  Wire.endTransmission();            // End transmission
}

// ------------------------------------------------------------
// Reads multiple bytes starting from a specific register
// ------------------------------------------------------------
void readRegisters(uint8_t startReg, uint8_t count, uint8_t* dest) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);              // Tell IMU where to start reading
  Wire.endTransmission(false);       // Restart condition for read mode
  Wire.requestFrom(MPU_ADDR, count); // Request 'count' bytes

  for (uint8_t i = 0; i < count; i++) {
    dest[i] = Wire.read();           // Store each received byte
  }
}

// ------------------------------------------------------------
// Initializes the IMU by waking it from sleep mode
// ------------------------------------------------------------
void initIMU() {
  writeRegister(REG_PWR_MGMT_1, 0x00);  // Clear sleep bit → IMU wakes up
  delay(100);                           // Allow IMU to stabilize
}

// ------------------------------------------------------------
// Reads raw accelerometer and gyroscope values from the IMU
// ------------------------------------------------------------
void readIMU(IMUData &d) {
  uint8_t buffer[14];                   // IMU outputs 14 bytes of data
  readRegisters(REG_ACCEL_XOUT_H, 14, buffer);

  // Combine high and low bytes for each axis
  d.ax = (buffer[0] << 8) | buffer[1];
  d.ay = (buffer[2] << 8) | buffer[3];
  d.az = (buffer[4] << 8) | buffer[5];

  d.gx = (buffer[8] << 8) | buffer[9];
  d.gy = (buffer[10] << 8) | buffer[11];
  d.gz = (buffer[12] << 8) | buffer[13];
}

// ------------------------------------------------------------
// Arduino setup: initializes serial and I²C, then wakes IMU
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);   // Start serial monitor for debugging
  Wire.begin();           // Start I²C communication
  initIMU();              // Wake and initialize IMU
  Serial.println("IMU initialized.");
}

// ------------------------------------------------------------
// Main loop: reads IMU data and prints raw values
// ------------------------------------------------------------
void loop() {
  readIMU(imu);           // Get latest IMU readings

  // Print accelerometer values
  Serial.print("Accel: ");
  Serial.print(imu.ax); Serial.print(", ");
  Serial.print(imu.ay); Serial.print(", ");
  Serial.print(imu.az);

  // Print gyroscope values
  Serial.print(" | Gyro: ");
  Serial.print(imu.gx); Serial.print(", ");
  Serial.print(imu.gy); Serial.print(", ");
  Serial.println(imu.gz);

  delay(100);             // Slow down output for readability
}
