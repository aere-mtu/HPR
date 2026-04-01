//#include <Adafruit_LSM6DSOX.h>  //imports the library needed for the sensor

#include <Wire.h>
#include <Adafruit_ADXL375.h>

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// --- ADXL375 ---
//Adafruit_ADXL375 adxl(12345); // default I2C ID
Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345);

// --- ICM-20948 ---
Adafruit_ICM20948 icm; 
sensors_event_t icm_accel, icm_gyro, icm_temp, icm_mag;

/****************************
         THRESHOLDS
*****************************/
const int takeoffThreshold = 20; //measured m/s/s

//assuming y is up
const int xDegThreshold = 45;
const int zDegThreshold = 45;

const unsigned long appogeTime = 2400000; // takes 24.1 secs to reach appoge
/****************************
         VALUES
*****************************/
/****************************
         PRESSURE SENSOR
*****************************/
#define PRESSURE_PIN 19
const float DIVIDER_RATIO = 1.5; 
const float V_REF = 3.3; 
const int ADC_RES = 1023; 
float currentPressure = 0.0;
unsigned long launchTime; //measured in millisecs
unsigned long curTime; //measured in millisecs

//sets up variables for the sensor
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp; //unused might be able to remove

double vertAccel;
bool hasLaunched = false; 

/****************************
         OTHER
*****************************/

//Defines pins for the LSM6DSOX sensor
#define LSM_CS    10    //cs pin
#define LSM_SCK   18    //scl pin
#define LSM_MISO  12    //do pin
#define LSM_MOSI  17    //sda pin

//Adafruit_LSM6DSOX sox; //sets up the sensor so we can get readings from it

// ---- ACCEL STUCK DETECTION ----
float lastAx = 0, lastAy = 0, lastAz = 0;
unsigned long accelLastChangeTime = 0;

const float accelMinDelta = 0.5;        // m/s^2 minimum change considered real
const unsigned long accelTimeout = 300; // ms before declaring stuck

bool accelInitialized = false;

// ---- GYRO STUCK DETECTION ----
float lastGyroX = 0;
unsigned long gyroLastChangeTime = 0;

const float gyroMinDelta = 0.5;      // deg/sec
const unsigned long gyroTimeout = 250; // ms

bool gyroInitialized = false;

// Forward declarations
void stop();
bool initADXL375();
bool initICM20948();
void readADXL375(float &x, float &y, float &z);
void readICM20948();
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

  // Init both ADXL375 and ICM20948, if one of them does not work program loops
  bool adxlOK = initADXL375();
  bool icmOK  = initICM20948();

  /*
  if(!adxlOK || !icmOK){
    Serial.println("Sensor initialization failed!");
    while(1){} // Stop program if sensors fail
  }
  */

  Serial.println("All sensors initialized. Waiting for launch...");
}

/*
This function runs CONTINUOUSLY. 
It replaces the infinite while() loops you had in setup().
*/
void loop() {
  
  // 1. ALWAYS read and print sensors (Pad or In-Flight)
  rocketStatus();

  // 2. CHECK FOR LAUNCH (Only if we haven't launched yet)
  /*
  if (!hasLaunched) {
    adxl.getEvent(&accel);
    vertAccel = accel.acceleration.x;

    Serial6.printf("x: %.2lf \n", vertAccel);

    if(vertAccel >= takeoffThreshold){
      hasLaunched = true;
      launchTime = millis(); // Begin counting millisecs AFTER launch
      Serial6.println("LAUNCHED");
      Serial.println(">>> ROCKET HAS LAUNCHED! <<<");
    } else {
      Serial.println("NOT LAUNCHED...");
    }
  }
  */

  // Delay to prevent spamming the Serial Monitor too fast
  // 5 seconds might be too slow to catch apogee exactly, consider lowering this to 500ms or 1000ms later!
  delay(1000); 
}


/*
Checks to make sure the rocket flight is nominal and gets current data
*/
void rocketStatus() {
  
  /*
  // Check for apogee ONLY if the rocket has actually launched
  if(hasLaunched && (millis() - launchTime >= appogeTime)){ 
    stop();
  }
  */
  
  /*
  float ax375, ay375, az375;
  readADXL375(ax375, ay375, az375);
  readICM20948(); 
  */

  // --- READ PRESSURE ---
  readPressure();

  /*
  Serial.printf("ADXL375: X=%.2f, Y=%.2f, Z=%.2f\n", ax375, ay375, az375);
  Serial.printf("ICM20948: AX=%.2f, AY=%.2f, AZ=%.2f | GX=%.2f, GY=%.2f, GZ=%.2f\n",
                icm_accel.acceleration.x, icm_accel.acceleration.y, icm_accel.acceleration.z, 
                icm_gyro.gyro.x, icm_gyro.gyro.y, icm_gyro.gyro.z);

  // --- PRINT PRESSURE ---
  Serial.printf("Pressure: %.2f\n", currentPressure); 

  // TEST CASE 1: ADXL out of range
  if ( ax375 < -200 || ax375 > 200 || az375 < -200 || az375 > 200 || ay375 < -200 || ay375 > 200){
           Serial.printf("ADXL bounds out of range!\n");
  }

  // TEST CASE 2: ICM out of range
  if(abs(icm_gyro.gyro.x) > 1000){ 
      Serial.printf("ICM is experiencing extreme spin\n"); 
  }

  // TEST CASE 3: ADXL stuck detection
  if(!accelInitialized){
      lastAx = ax375;
      lastAy = ay375;
      lastAz = az375;
      accelLastChangeTime = millis();
      accelInitialized = true;
  }

  bool accelChanged =
      (abs(ax375 - lastAx) > accelMinDelta) ||
      (abs(ay375 - lastAy) > accelMinDelta) ||
      (abs(az375 - lastAz) > accelMinDelta);

  if(accelChanged){
      accelLastChangeTime = millis();
  }

  lastAx = ax375;
  lastAy = ay375;
  lastAz = az375;

  // Only check for stuck during powered ascent
  if(hasLaunched && millis() - launchTime < appogeTime){
      if(millis() - accelLastChangeTime > accelTimeout){
          Serial.println("ACCELEROMETER STUCK DETECTED!");
      }
  }

  // TEST CASE 4: Gyro stuck detection
  if(!gyroInitialized){
      lastGyroX = icm_gyro.gyro.x; 
      gyroLastChangeTime = millis();
      gyroInitialized = true;
  }

  if(abs(icm_gyro.gyro.x - lastGyroX) > gyroMinDelta){ 
      gyroLastChangeTime = millis();
  }

  lastGyroX = icm_gyro.gyro.x; 

  if(hasLaunched && millis() - launchTime < appogeTime){
      if(millis() - gyroLastChangeTime > gyroTimeout){
          Serial.println("GYROSCOPE STUCK DETECTED!");
          stop();
      }
  }
  */
}

/*
Outputs an erorr mesage and enters an endless loop
*/
void stop(){
  Serial6.println("\nROCKET PARAMETERS OUT OF BOUNDS, SPIN TERMINATED");
  Serial.println("\nROCKET PARAMETERS OUT OF BOUNDS, SPIN TERMINATED");
  while(true){}
}

bool initADXL375() {
  if (!adxl.begin()) {
    Serial.println("ADXL375 not found!");
    return false;
  }
  Serial.println("ADXL found!");
  return true;
}

void readADXL375(float &x, float &y, float &z) {
  sensors_event_t event;
  adxl.getEvent(&event);
  x = event.acceleration.x;
  y = event.acceleration.y;
  z = event.acceleration.z;
}

bool initICM20948() {
   Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20948 test!");

  // Try to initialize!
  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("ICM20948 not found!");
    return false;
  }
  Serial.println("ICM20948 Found!");
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
  case ICM20948_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.println("OK");

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }
  Serial.println();
  return true;
}

void readICM20948() {
   //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  Serial.print("\t\tMag X: ");
  Serial.print(mag.magnetic.x);
  Serial.print(" \tY: ");
  Serial.print(mag.magnetic.y);
  Serial.print(" \tZ: ");
  Serial.print(mag.magnetic.z);
  Serial.println(" uT");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();
}

// --- PRESSURE SENSOR READ & CONVERT FUNCTION ---
void readPressure() {
  int rawADC = analogRead(PRESSURE_PIN);
  
  // Convert raw 10-bit analog reading (0-1023) to the voltage at the pin
  float pinVoltage = ((float)rawADC / ADC_RES) * V_REF;
  
  // Multiply by the divider ratio to find the actual voltage output of the sensor (V)
  float sensorVoltage = pinVoltage * DIVIDER_RATIO;
  
  // Apply the specific transfer function to convert voltage to actual pressure
  currentPressure = (sensorVoltage - 0.453) / 0.0231; 
}
