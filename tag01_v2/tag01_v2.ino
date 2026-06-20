#include "LPS.h"
#include "BLE.h"

BLE ble;
LPS lps;
uint8_t device_address = 0x05;
uint32_t lastTimeGlobal = 0;
uint32_t lastTimeLocal = 0;
uint8_t currentAnchor = 0x01;
uint8_t totalAnchor = 3;
STATE state = ST_WAIT;
double distance[3];
Position position_self = {0,0};
Position position_B = {1.5,0};
Position position_C = {1.5,2.5};
unsigned long lastTimeBle = 0;
int timeout = 1000;

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(led, OUTPUT);

  ble.begin();

  while(!ble.connectionStatus()){
    delay(1);
  }

  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
  digitalWrite(led, HIGH);

  lps.begin(device_address);
  lps.printModulInfo();

  lps.restartTRX();
  lps.startReceive();
  lastTimeGlobal = millis();
}

void loop() {
  if(state == ST_WAIT){
    if(lps.isMyTurn()){
      //reset parameters
      currentAnchor = 0x01;
      lastTimeGlobal = millis();
      lastTimeLocal = millis();
      //lps.restartTRX();
      state = ST_IDLE;
      //Serial.println("DAPAT GILIRAN");
      //delay(10);
      lps.sendCmd(currentAnchor, POLL);
    }
  }
  else if(state == ST_IDLE){
    /////////DEBUG/////////
    // Serial.println("MASUK IDLE");
    // delay(1000);
    ///////////////////////
    //send poll
    if(sentAck){
        sentAck = false;
        lps.startReceive();
    }
    if(receiveAck){
      receiveAck = false;
      lps.commitData();
      if(lps.getLastCmd() == POLL_ACK){
        lps.sendRange(currentAnchor);
      }
      else if(lps.getLastCmd() == RANGE_REPORT){
        distance[currentAnchor-1] = lps.getDistance();
        //Serial.print("Distance From Anchor ");
        //Serial.print(currentAnchor);
        //Serial.print(" : ");
        //Serial.println(distance);
        currentAnchor++;
        state = ST_COMPLETE;
      }
    }

    if(millis()-lastTimeGlobal > timeout){
      /*
        TODO :
        - TIMEOUT
        - back to ST_WAIT 
        - report to ANCHOR MAIN TAG_FINISH
      */
      Serial.print("TIMEOUT FOR ANCHOR ");
      Serial.println(currentAnchor);
      state = ST_WAIT;
      lps.sendCmd(ANCHOR_MAIN, TAG_FINISH);
      delay(100);
      lps.restartTRX();
      lps.startReceive();
    }
    
  }
  else if(state == ST_COMPLETE){
    if(currentAnchor > totalAnchor){
      //TODO : Calculate Coordinate
      double x = 0.0;
      double y = 0.0;
      lps.calculatePosition(x, y);
      // Serial.print("X : ");
      // Serial.print(x,3);
      // Serial.print("  |  ");
      // Serial.println(y, 3);

      //Send coordinate to BLE Periodically
      if(millis() - lastTimeBle > 100){
        String x_str = String(x, 2);
        String y_str = String(y, 2);
        String ble_send = x_str+","+y_str;
        ble.send(ble_send);
        lastTimeBle = millis();
        // Serial.print("X : ");
        // Serial.print(x,3);
        // Serial.print("  |  y : ");
        // Serial.println(y, 3);
      }

      if(sentAck){
        sentAck = false;
        currentAnchor = 0x01;
        lps.restartTRX();
        lps.startReceive();
        state = ST_WAIT;
        //Serial.println("Return to WAIT");
        //delay(100); //for debug only
      }
      if(millis()-lastTimeLocal > 30){
        lps.sendCmd(ANCHOR_MAIN, TAG_FINISH);
        lastTimeLocal = millis();
      }
    }
    else{
      state = ST_IDLE;
      lps.sendCmd(currentAnchor, POLL);
      //lastTimeGlobal = millis();
    }
  }
 


}
