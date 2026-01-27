#include <SoftwareSerial.h>
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS   16 //RFM chip select pin
#define RFM95_RST  17 //RFM reset pin
#define RFM95_INT  21 //

//defines the frequency as 915MHz
#define RF95_FREQ 915.0

SoftwareSerial mySerial (0, 1);
RH_RF95 rf95(RFM95_CS, RFM95_INT);
  int buffSize = 9;
  int buffCount = 0;
  char buff[9];

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial) delay(1);
  delay(100);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  rf95.init();

  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);

}


void loop() {
   byte n = Serial1 .available();
  if (n != 0){
    char ch = Serial1 .read();
   // Serial.print(ch);
    buff[buffCount] = ch;
    buffCount++;
    if(buffCount >= buffSize){
      rf95.send((uint8_t*)buff, sizeof(buffCount));
      buffCount = 0;
    }
    delay(1);
  }
}