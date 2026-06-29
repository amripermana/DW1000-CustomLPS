#include "LPS.h"

uint8_t device_address = 0x03;
uint64_t waktu_A1;
double jarak_A1_A2 = 5.0; //dalam meter, sesuaikan dengan jarak antar anchor sebenarnya
uint64_t tof_A1_A2_uwb;
double kecepatan_cahaya = 299792458.0;
double dw1000_tick_duration = 1.5658430119766968e-11;
uint64_t timeReceive;

LPS lps;

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(led, OUTPUT);

  double ticks_desimal = jarak_A1_A2 / (kecepatan_cahaya * dw1000_tick_duration);
  tof_A1_A2_uwb = (uint64_t)ticks_desimal;

  Serial.print("ToF Saat Ini : ");
  Serial.println(tof_A1_A2_uwb);

  lps.begin(device_address);
  lps.printModulInfo();

  lps.setReplyDelay(20000);//satuan us, delay anchorb untuk kirim setelah anchormain kirim

  lps.restartTRX();
  lps.startReceive();

}

void loop() {
  if(sentAck){
    sentAck = false;
    lps.startReceive();
  }
  if(receiveAck){
    receiveAck = false;
    //digitalWrite(led, digitalRead(led) ^ 1);
    lps.commitData();
    if(lps.getLastSender()==ANCHOR_MAIN){
      lps.slaveAnchorBroadcast();
      digitalWrite(led, digitalRead(led) ^ 1);
      //delay(3);
      //DEBUG
      // lps.startReceive();
    }
    else{
      lps.startReceive();
    }
  }

}
