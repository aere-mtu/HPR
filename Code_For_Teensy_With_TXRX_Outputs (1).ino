#include <Adafruit_LSM6DSOX.h>  //imports the library needed for the sensor
//#include <SoftwareSerial.h>     //imports the library needed for communication with 

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


/*
this funcion will run once and is the main body of code we will be using
*/
void setup(void){
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
  //@TODO: ADD MORE TEST CASES STILL
}

/*
Outputs an erorr mesage and enters an endless loop
*/
void stop(){
  Serial6.println("\nROCKET PARAMETERS OUT OF BOUNDS, SPIN TERMINATED");
  while(true){}
}



/*
only have this function because it is required to exist to compile
*/
void loop(){}
