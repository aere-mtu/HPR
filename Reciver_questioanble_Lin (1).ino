#include <SPI.h>
#include <RH_RF95.h>


#define RFM95_CS   16 //RFM chip select pin
#define RFM95_RST  17 //RFM reset pin
#define RFM95_INT  21 //

//defines the frequency as 915MHz
#define RF95_FREQ 915.0

//creates radio "object" 
RH_RF95 rf95(RFM95_CS, RFM95_INT);


void setup() {
 pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
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
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf95.recv(buf, &len)) {
      Serial.print((char*)buf);
  }
}
