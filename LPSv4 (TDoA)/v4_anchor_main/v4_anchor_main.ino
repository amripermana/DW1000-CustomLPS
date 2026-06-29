#include "LPS.h"

LPS lps;

uint8_t device_address = 0x01;
unsigned long lastTimeGlobal = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(led, OUTPUT);

  lps.begin(device_address);
  lps.printModulInfo();
  lastTimeGlobal = millis();
  lps.restartTRX();
  //delay(2000);

  //DEBUG
  // lps.sendBroadcast();
  // digitalWrite(led, digitalRead(led) ^ 1);
  // delay(100);
  // lps.sendBroadcast();
  // digitalWrite(led, digitalRead(led) ^ 1);
  // delay(100);

}

void loop() {
  if(millis()-lastTimeGlobal > 100){
    lps.sendBroadcast();
    lastTimeGlobal = millis();
    digitalWrite(led, digitalRead(led) ^ 1);
  }
}
