#include "LPS.h"

LPS lps;
uint8_t device_address = 0x02;
uint32_t lastTimeGlobal = 0;
uint32_t lastTimeLocal = 0;
uint8_t currentTag = 0x00;
STATE state = ST_IDLE;
double distance;

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(led, OUTPUT);

  lps.begin(device_address);
  lps.printModulInfo();

  lastTimeGlobal = millis();
  delay(1000);
  lps.restartTRX();
  lps.startReceive();
}

void loop() {
  if(sentAck){
      sentAck = false;
      if(lps.getLastCmdSent() == POLL_ACK){
        lps.setTimePollAckSent();
      }
      lps.startReceive();
  }
  if(receiveAck){
    receiveAck = false;
    lps.commitData();
    currentTag = lps.getLastSender();
    //Serial.println(currentTag);
    if(lps.getLastCmd() == POLL){
      lps.setTimePollReceived();
      lps.sendCmd(currentTag, POLL_ACK);
    }
    else if(lps.getLastCmd() == RANGE){
      distance = lps.calculateDistance();
      lps.sendDistance(currentTag, distance);
    }
  }
}
