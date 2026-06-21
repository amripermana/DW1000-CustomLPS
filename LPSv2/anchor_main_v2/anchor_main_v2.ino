#include "LPS.h"

LPS lps;
uint8_t device_address = 0x01;
uint32_t lastTimeGlobal = 0;
uint32_t lastTimeLocal = 0;
uint8_t currentTag = 0x05;
uint8_t totalTag = 1;
STATE state = ST_BROADCAST;
double distance;
const int timeout = 300;


void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(led, OUTPUT);

  lps.begin(device_address);
  lps.printModulInfo();

  lps.restartTRX();
  lastTimeGlobal = millis();
  delay(1000);
}

void loop() {
  if(state == ST_BROADCAST){
    if(millis()-lastTimeLocal > 30){
      lps.sendCmd(currentTag, START_TWR);
      lastTimeLocal = millis();
    }
    if(sentAck){
      sentAck = false;
      //reset parameters
      lastTimeGlobal = millis();
      lastTimeLocal = 0;
      // lps.restartTRX();
      lps.startReceive();
      state = ST_IDLE;
      /////////DEBUG/////////
      // Serial.print("CMD SENT TO TAG : ");
      // Serial.println(currentTag);
      digitalWrite(led, digitalRead(led) ^ 1);
      ///////////////////////
    }
  }
  else if(state == ST_IDLE){
    /////////DEBUG/////////
    //Serial.println("MASUK IDLE");
    // delay(1000);
    ///////////////////////
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
      if(lps.getLastCmd() == POLL){
        lps.setTimePollReceived();
        lps.sendCmd(currentTag, POLL_ACK);
      }
      else if(lps.getLastCmd() == RANGE){
        distance = lps.calculateDistance();
        lps.sendDistance(currentTag, distance);
        state = ST_WAIT;
        lastTimeGlobal = millis();
      }
    }
    if(millis()-lastTimeGlobal > timeout){
      state = ST_BROADCAST;
      lps.restartTRX();
      delay(10);
      currentTag++;
      if(currentTag-4>totalTag){
        currentTag = 0x05;
      }
    }
  }
  else if(state == ST_WAIT){
    if(lps.isTagFinish(currentTag)){
      currentTag++;
      if(currentTag-4>totalTag){
        currentTag = 0x05;
      }
      delay(10);
      Serial.println("Back to Broadcast");
      lps.restartTRX();
      state = ST_BROADCAST;
    }
    /*
      TODO :
      - add timeout for not respond tag
    */
    if(millis()-lastTimeGlobal > timeout){
      state = ST_BROADCAST;
      currentTag++;
      lps.restartTRX();
      delay(10);
      if(currentTag-4>totalTag){
        currentTag = 0x05;
      }
      Serial.println("Back to Broadcast");
    }
  }
  

}
