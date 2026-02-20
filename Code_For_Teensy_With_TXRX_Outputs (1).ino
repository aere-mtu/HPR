#include <Adafruit_LSM6DSOX.h>  //imports the library needed for the sensor
//#include <SoftwareSerial.h>     //imports the library needed for communication with 

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>

// --- ADXL375 ---
Adafruit_ADXL375 adxl(12345); // default I2C ID

// --- ICM-20948 ---
#define ICM_ADDR 0x68
struct ICMData { int16_t ax, ay, az; int16_t gx, gy, gz; };
ICMData icm;

/****************************
         THRESHOLDS
*****************************/
const int takeoffThreshold = 20; //measured m/s/s

//assuming y is up
const int xDegThreshold = 45;
const int zDegThreshold = 45;

const int appogeTime = 2400000; // takes 24.1 secs to reach appoge
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
long launchTime;//measured in millisecs
long curTime; //measured in millisecs

//sets up variables for the sensor
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp; //unused might be able to remove

double vertAccel;
bool hasLaunched = false; 
/****************************
         OTHER
*****************************/

#define nozzels_1_3 18 //sets nozzels 1 & 3 to be controlled by pin 18
#define nozzels_2_4 16 //sets nozzels 2 & 4 to be controlled by pin 16


//Defines pins for the LSM6DSOX sensor
#define LSM_CS    34    //cs pin
#define LSM_SCK   40    //scl pin
#define LSM_MISO  36    //do pin
#define LSM_MOSI  38    //sda pin

Adafruit_LSM6DSOX sox; //sets up the sensor so we can get readings from it

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


/*
this funcion will run once and is the main body of code we will be using
*/
void setup(void){

Wire.begin();

// Initialize ADXL375
if(!initADXL375()) while(1);

// Initialize ICM-20948
initICM20948();

Serial.println("All sensors initialized.");

         
  //sets the pins for the nozzels to be outputs
  pinMode(nozzels_1_3,OUTPUT); 
  pinMode(nozzels_2_4,OUTPUT);
  
  //provides power to the sensor
  pinMode(14,OUTPUT);
  digitalWrite(14,HIGH);

  //allows us to use the serial moniotor for debugging
  Serial.begin(9600);

  //allows us to output data on pins 24 & 25
  Serial6.begin(9600);

  //sets up the sensor
  sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI);
  sox.setAccelDataRate(LSM6DS_RATE_208_HZ);



  //enters loop until the takeoff threshold is achived
  while(!hasLaunched && sox.getEvent(&accel, &gyro, &temp)){
    vertAccel = accel.acceleration.x;

    Serial6.printf("x: %.2lf \n", vertAccel);

    if(vertAccel >= takeoffThreshold){
      hasLaunched = true;
    }else{
      Serial.println("NOT LAUNCHED...");
      delay(200);
    }
  }
  Serial6.println("LAUNCHED");
  Serial.println("LAUNCHED");

   launchTime = millis(); //begins counting millisecs after launch

  spin(0);//stops the rocket
  delay(3000);//waits a second
  spin(5);
  delay(3000);//waits a second
  spin(-5);
  //@TODO add more 
}


/*
this function takes in a double target velocity and will open and close the nozzels to make the rocket match the given rotational velocity
*/
void spin(double targetVelocity) {
  
  while(gyro.gyro.x!=targetVelocity){ //while the spin isnt equal to target spin
    Serial6.printf("Cur Vel: %f \t Tar Vel: %lf\n",gyro.gyro.x,targetVelocity);

    rocketStatus(); //make sure the rocket is safe & get updated data
        if(gyro.gyro.x>targetVelocity){//if x is rotating faster than our target velocity
      digitalWrite(nozzels_2_4, HIGH);//open 2&4
    }else{
      digitalWrite(nozzels_2_4, LOW);//close 2&4
    }
    if(gyro.gyro.x<targetVelocity){//if x is rotating less than our target velocity
      digitalWrite(nozzels_1_3, HIGH);//open 1&3
    }else{
      digitalWrite(nozzels_1_3, LOW);//close 1&3
    }
  }
  //value is matched close all
  Serial6.printf("Target Rotation of %lf Reached \n",targetVelocity);
  digitalWrite(nozzels_1_3, LOW);//close 1&3
  digitalWrite(nozzels_2_4, LOW);//close 2&4
}
/*
Checks to make sure the rocket flight is nominal and gets curent data
*/
void rocketStatus() {
  sox.getEvent(&accel, &gyro, &temp);
  if(millis()-launchTime>=appogeTime){ //if time after launch is greater than appo time
    stop();
  }
float ax375, ay375, az375;
readADXL375(ax375, ay375, az375);
readICM20948(icm);

Serial.printf("ADXL375: X=%.2f, Y=%.2f, Z=%.2f\n", ax375, ay375, az375);
Serial.printf("ICM20948: AX=%d, AY=%d, AZ=%d | GX=%d, GY=%d, GZ=%d\n",
              icm.ax, icm.ay, icm.az, icm.gx, icm.gy, icm.gz);

  //@TODO: ADD MORE TEST CASES STILL

// TEST CASE 1: ADXL out of range
if ( ax375 < -200 || ax375 > 200 || az375 < -200 || az375 > 200 || ay375 < -200 || ay375 > 200){
         Serial.printf("ADXL bounds out of range!");
}

// TEST CASE 2: ICM out of range

if(abs(icm.gx) > 1000){
    Serial.printf("ADXL is experiencing extreme spin");
}

// TEST CASE  3: ADXL stuck detection

// First time initialization
if(!accelInitialized){
    lastAx = ax375;
    lastAy = ay375;
    lastAz = az375;
    accelLastChangeTime = millis();
    accelInitialized = true;
}

// Check if acceleration changed meaningfully
bool accelChanged =
    (abs(ax375 - lastAx) > accelMinDelta) ||
    (abs(ay375 - lastAy) > accelMinDelta) ||
    (abs(az375 - lastAz) > accelMinDelta);

// If changed, reset timer
if(accelChanged){
    accelLastChangeTime = millis();
}

// Update stored values
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
    lastGyroX = gyro.gyro.x;
    gyroLastChangeTime = millis();
    gyroInitialized = true;
}

if(abs(gyro.gyro.x - lastGyroX) > gyroMinDelta){
    gyroLastChangeTime = millis();
}

lastGyroX = gyro.gyro.x;

if(hasLaunched && millis() - launchTime < appogeTime){

    if(millis() - gyroLastChangeTime > gyroTimeout){
        Serial.println("GYROSCOPE STUCK DETECTED!");
        stop();
    }
}



}

/*
Outputs an erorr mesage and enters an endless loop
*/
void stop(){
  Serial6.println("\nROCKET PARAMETERS OUT OF BOUNDS, SPIN TERMINATED");
  while(true){}
}

bool initADXL375() {
  if (!adxl.begin()) {
    Serial.println("ADXL375 not found!");
    return false;
  }
  adxl.printSensorDetails();
  return true;
}

void readADXL375(float &x, float &y, float &z) {
  sensors_event_t event;
  adxl.getEvent(&event);
  x = event.acceleration.x;
  y = event.acceleration.y;
  z = event.acceleration.z;
}

void writeICMRegister(uint8_t reg, uint8_t value){
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readICMRegisters(uint8_t reg, uint8_t count, uint8_t* dest){
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, count);
  for(uint8_t i=0;i<count;i++){
    dest[i] = Wire.read();
  }
}

void initICM20948() {
  writeICMRegister(0x06, 0x01); // wake sensor (PWR_MGMT_1)
  delay(100);
}

void readICM20948(ICMData &d){
  uint8_t buf[14];
  readICMRegisters(0x2D,14,buf); // ACCEL_XOUT_H
  d.ax = (buf[0]<<8)|buf[1];
  d.ay = (buf[2]<<8)|buf[3];
  d.az = (buf[4]<<8)|buf[5];
  d.gx = (buf[8]<<8)|buf[9];
  d.gy = (buf[10]<<8)|buf[11];
  d.gz = (buf[12]<<8)|buf[13];
}


/*
only have this function because it is required to exist to compile
*/
void loop(){}
