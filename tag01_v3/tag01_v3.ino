#include "LPS.h"
#include "BLE.h"

BLE ble;
LPS lps;
uint8_t device_address = 0x05;
unsigned long lastTimeGlobal = 0;
double x = 0.0;
double y = 0.0;
const int timeout = 100;
State state = IDLE;

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(led, OUTPUT);

  // ble.begin();
  // while(!ble.connectionStatus()){
  //   delay(1);
  // }
  // digitalWrite(led, HIGH);
  // delay(500);
  // digitalWrite(led, LOW);
  // delay(500);
  // digitalWrite(led, HIGH);

  lps.beginTag(device_address);
  lps.printModulInfo();
  lps.startReceive();
  
}

void loop() {
  if(state == IDLE){
    if(DW1000NgRTLS::receiveFrame()){
      lps.commitData();
      if(lps.getLastCmd() == START_TWR && lps.getLastSender() == ANCHOR_MAIN){
        state = ACTIVE;
        lastTimeGlobal = millis();
      }
      else if(lps.getLastCmd() == XY_REPORT && lps.getLastSender() == ANCHOR_MAIN){
        /*
        TODO :
          - parse coordinate report
          - send ble
        */
        digitalWrite(led, digitalRead(led) ^ 1);
        lps.getXY(x, y);
        String x_str = String(x, 2);
        String y_str = String(y, 2);
        String data = x_str+","+y_str;
        //ble.send(data);
        delay(100); //delay 500 ok dengan ble
        ///////////DEBUG///////////
        String positioning = "TAG01 POSITION >> x: ";
        positioning += x; positioning +=" y: ";
        positioning += y;
        Serial.println(positioning);
        ///////////////////////////
      }
    }
  }else{
    if(lps.tagStartRTLS()){
      state = IDLE;
      lps.startReceive();
      //Serial.println("TWR SUKSES");
    }
  }

  if(millis() - lastTimeGlobal > timeout && state == ACTIVE){
    state = IDLE;
    lps.startReceive();
  }

}
