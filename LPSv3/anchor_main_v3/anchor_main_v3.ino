#include "LPS.h"

LPS lps;
uint8_t device_address = 0x01;
unsigned long lastTimeGlobal = 0;
uint8_t currentTag = 0x05;
uint8_t totalTag = 1;
double x = 0.0;
double y = 0.0;
const int timeout = 100;

State state = IDLE;

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(led, OUTPUT);

  lps.begin(device_address);
  lps.printModulInfo();
}

void loop() {
  if(state == IDLE){
    //Serial.println("CMD SENT");
    lps.sendCmd(currentTag, START_TWR);
    lastTimeGlobal = millis();
    state = ACTIVE;
  }else{
    if(lps.startRTLS(currentTag, x, y)){
      ///////////DEBUG///////////
      // String positioning = "Found position >> x: ";
      // positioning += x; positioning +=" y: ";
      // positioning += y;
      // Serial.println(positioning);
      ///////////////////////////
      /*
        TODO : 
        - Send Position to Tag
      */   
      lps.sendXY(currentTag, x, y);   
    }
  }



  if(millis() - lastTimeGlobal > timeout && state == ACTIVE){
    state = IDLE;
    currentTag++;
    if(currentTag-4>totalTag) currentTag = 0x05;
  }

}
