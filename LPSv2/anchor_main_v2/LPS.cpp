#include "LPS.h"

const uint8_t led = 8;
const uint8_t PIN_RST = 10; 
const uint8_t PIN_IRQ = 3; 
const uint8_t PIN_SS = 7; 
char EUI[23] = "DE:13:06:26:00:00:00:";
uint16_t PAN_ID = 0xDEDE;
uint8_t self_address;
uint32_t manualPower = 0xBFBFBFBF; 
volatile boolean sentAck = false;
volatile boolean receiveAck = false;
uint64_t timePollSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
uint64_t timeRangeReceived;
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint16_t replyDelayTimeUS = 3000;


byte tx_packet[25] = {0x41, 0x88, 0x01, 0xDE, 0xDE,
                        0x00, 0x00, 0x01, 0x00, 0x00, 
                        0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00};
byte rx_buff[25];

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

frame_filtering_configuration_t FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    false
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
    true,
    true,
    true,
    false,
    true
};

LPS::LPS(){}

void LPS::txHandler(){
  sentAck = true;
}

void LPS::rxHandler(){
  receiveAck = true;
}

void LPS::begin(uint8_t &device_address){
  int len = strlen(EUI);
  sprintf(&EUI[len], "%02X", device_address);
  self_address = device_address;
  DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::enableFrameFiltering(FRAME_FILTER_CONFIG);
  DW1000Ng::setEUI(EUI);
  DW1000Ng::setNetworkId(PAN_ID);
  DW1000Ng::setDeviceAddress(device_address);
  DW1000Ng::setAntennaDelay(16493);
  DW1000Ng::setPreambleDetectionTimeout(15); //15
  DW1000Ng::setSfdDetectionTimeout(0); //273
  DW1000Ng::setReceiveFrameWaitTimeoutPeriod(0); //2000
  DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);
  DW1000Ng::attachSentHandler(txHandler);
  DW1000Ng::attachReceivedHandler(rxHandler);
  DW1000Ng::setTXPower(manualPower);
}

void LPS::printModulInfo(){
  char msg[128];
  Serial.println("===== Modul Info =====");
  DW1000Ng::getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);  
  Serial.println("======================");   
}

void LPS::sendCmd(uint8_t target, uint8_t cmd){
  tx_packet[9] = cmd;
  tx_packet[5] = target;
  tx_packet[7] = self_address;
  DW1000Ng::setTransmitData(tx_packet, LEN_DATA);
  DW1000Ng::startTransmit();
  //Serial.println("CMD SEND");
}

void LPS::startReceive(){
  DW1000Ng::startReceive();
}

void LPS::commitData(){
  DW1000Ng::getReceivedData(rx_buff, LEN_DATA);
}

uint8_t LPS::getLastCmd(){
  return rx_buff[9];
}

uint8_t LPS::getLastSender(){
  return rx_buff[7];
}

void LPS::sendRange(uint8_t target){
  timePollSent = DW1000Ng::getTransmitTimestamp();
  timePollAckReceived = DW1000Ng::getReceiveTimestamp();

  tx_packet[9] = RANGE;
  tx_packet[5] = target;
  byte futureTimeBytes[LENGTH_TIMESTAMP];
  
  timeRangeSent = DW1000Ng::getSystemTimestamp();
  timeRangeSent += DW1000NgTime::microsecondsToUWBTime(replyDelayTimeUS);
  DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
  DW1000Ng::setDelayedTRX(futureTimeBytes);
  timeRangeSent += DW1000Ng::getTxAntennaDelay();

  DW1000NgUtils::writeValueToBytes(tx_packet + 10, timePollSent, LENGTH_TIMESTAMP);
  DW1000NgUtils::writeValueToBytes(tx_packet + 15, timePollAckReceived, LENGTH_TIMESTAMP);
  DW1000NgUtils::writeValueToBytes(tx_packet + 20, timeRangeSent, LENGTH_TIMESTAMP);
  DW1000Ng::setTransmitData(tx_packet, LEN_DATA);
  DW1000Ng::startTransmit(TransmitMode::DELAYED);
}

void LPS::setTimePollReceived(){
  timePollReceived = DW1000Ng::getReceiveTimestamp();
}

void LPS::setTimePollAckSent(){
  timePollAckSent = DW1000Ng::getTransmitTimestamp();
  tx_packet[9] = 0xFF;
}

double LPS::calculateDistance(){
  timeRangeReceived = DW1000Ng::getReceiveTimestamp();
  timePollSent = DW1000NgUtils::bytesAsValue(rx_buff + 10, LENGTH_TIMESTAMP);
  timePollAckReceived = DW1000NgUtils::bytesAsValue(rx_buff + 15, LENGTH_TIMESTAMP);
  timeRangeSent = DW1000NgUtils::bytesAsValue(rx_buff + 20, LENGTH_TIMESTAMP);

  double distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
                                                        timePollReceived, 
                                                        timePollAckSent, 
                                                        timePollAckReceived, 
                                                        timeRangeSent, 
                                                        timeRangeReceived);

  return distance;
}

uint8_t LPS::getLastCmdSent(){
  return tx_packet[9];
}

STATE LPS::waitLastAnchor(){
  STATE output = ST_IDLE;
  if(sentAck){
    sentAck = false;
    LPS::startReceive();
  }
  if(receiveAck){
    receiveAck = false;
    LPS::commitData();
    //!!!URGENT!!! : change ANCHOR_B if ANCHOR_C is Ready
    if(rx_buff[7] == ANCHOR_B && rx_buff[9] == 0xDE){
      output = ST_WAIT_POLLACK;
      
    }
  }
  return output;
}

void LPS::restartTRX(){
  DW1000Ng::forceTRxOff();
}

void LPS::sendDistance(uint8_t target, double jarak){
  tx_packet[5] = target;
  tx_packet[7] = self_address;
  tx_packet[9] = RANGE_REPORT;
  memcpy(&tx_packet[10], &jarak, sizeof(double));
  DW1000Ng::setTransmitData(tx_packet, LEN_DATA);
  DW1000Ng::startTransmit();
}

bool LPS::isTagFinish(uint8_t target){
  bool output = false;
  if(sentAck){
    sentAck = false;
    LPS::startReceive();
  }
  if(receiveAck){
    receiveAck = false;
    LPS::commitData();
    if(rx_buff[7] == target && rx_buff[9] == TAG_FINISH){
      output = true;
    }
  }
  return output;
}



