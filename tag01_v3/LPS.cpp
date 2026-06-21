#include "LPS.h"

const uint8_t led = 8;
const uint8_t PIN_RST = 10; 
const uint8_t PIN_IRQ = 3; 
const uint8_t PIN_SS = 7; 

typedef struct Position {
    double x;
    double y;
} Position;

char EUI[] = "DE:21:06:26:00:00:00:";
uint16_t PAN_ID = 0xDEDE;
uint8_t self_address = 0x00;
Position position_self = {0,0};
Position position_B = {3,0};
Position position_C = {3,2.5};
double range_self;
double range_B;
double range_C;
boolean received_B = false;
byte target_eui[8];
byte tag_shortAddress[] = {0x05, 0x00};
byte anchor_b[] = {0x02, 0x00};
uint16_t next_anchor = 2;
byte anchor_c[] = {0x03, 0x00};

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

frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    true /* This allows blink frames */
};

frame_filtering_configuration_t TAG_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    false
};

byte tx_packet[26] = {0x41, 0x88, 0x01, 0xDE, 0xDE,
                        0x00, 0x00, 0x01, 0x00, 0x00, 
                        0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte rx_buff[26];

LPS::LPS(){}


void LPS::begin(uint8_t device_address){
  int len = strlen(EUI);
  sprintf(&EUI[len], "%02X", device_address);
  self_address = device_address;
  DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);
  DW1000Ng::setEUI(EUI);
  DW1000Ng::setPreambleDetectionTimeout(15);
  DW1000Ng::setSfdDetectionTimeout(273);
  DW1000Ng::setReceiveFrameWaitTimeoutPeriod(2000);
  DW1000Ng::setNetworkId(PAN_ID);
  DW1000Ng::setDeviceAddress(device_address);
  DW1000Ng::setAntennaDelay(16436);
}

void LPS::beginTag(uint8_t device_address){
  int len = strlen(EUI);
  sprintf(&EUI[len], "%02X", device_address);
  self_address = device_address;
  DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::enableFrameFiltering(TAG_FRAME_FILTER_CONFIG);
  DW1000Ng::setEUI(EUI);
  DW1000Ng::setNetworkId(PAN_ID);
  DW1000Ng::setAntennaDelay(16436);
  DW1000Ng::setPreambleDetectionTimeout(15);
  DW1000Ng::setSfdDetectionTimeout(273);
  DW1000Ng::setReceiveFrameWaitTimeoutPeriod(2000);
  DW1000Ng::setDeviceAddress(device_address);
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

void LPS::calculatePosition(double &x, double &y) {

    /* This gives for granted that the z plane is the same for anchor and tags */
    double A = ( (-2*position_self.x) + (2*position_B.x) );
    double B = ( (-2*position_self.y) + (2*position_B.y) );
    double C = (range_self*range_self) - (range_B*range_B) - (position_self.x*position_self.x) + (position_B.x*position_B.x) - (position_self.y*position_self.y) + (position_B.y*position_B.y);
    double D = ( (-2*position_B.x) + (2*position_C.x) );
    double E = ( (-2*position_B.y) + (2*position_C.y) );
    double F = (range_B*range_B) - (range_C*range_C) - (position_B.x*position_B.x) + (position_C.x*position_C.x) - (position_B.y*position_B.y) + (position_C.y*position_C.y);

    x = (C*E-F*B) / (E*A-B*D);
    y = (C*D-A*F) / (B*D-A*E);
}

bool LPS::startRTLS(uint8_t target, double &outX, double &outY){
  bool output = false;
  if(DW1000NgRTLS::receiveFrame()){
    size_t recv_len = DW1000Ng::getReceivedDataLength();
    byte recv_data[recv_len];
    DW1000Ng::getReceivedData(recv_data, recv_len);

    if(recv_data[0] == BLINK) {
        DW1000NgRTLS::transmitRangingInitiation(&recv_data[2], tag_shortAddress);
        DW1000NgRTLS::waitForTransmission();

        RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::RANGING_CONFIRM, next_anchor);
        if(!result.success) return false;
        range_self = result.range;

        // String rangeString = "Range: "; rangeString += range_self; rangeString += " m";
        // rangeString += "\t RX power: "; rangeString += DW1000Ng::getReceivePower(); rangeString += " dBm";
        // Serial.println(rangeString);

    } else if(recv_data[9] == 0x60) {
        double range = static_cast<double>(DW1000NgUtils::bytesAsValue(&recv_data[10],2) / 1000.0);
        // String rangeReportString = "Range from: "; rangeReportString += recv_data[7];
        // rangeReportString += " = "; rangeReportString += range;
        // Serial.println(rangeReportString);
        if(received_B == false && recv_data[7] == anchor_b[0] && recv_data[8] == anchor_b[1]) {
            range_B = range;
            received_B = true;
        } else if(received_B == true && recv_data[7] == anchor_c[0] && recv_data[8] == anchor_c[1]){
            range_C = range;
            // double x,y;
            calculatePosition(outX, outY);
            // String positioning = "Found position - x: ";
            // positioning += x; positioning +=" y: ";
            // positioning += y;
            // Serial.println(positioning);
            output = true;
            received_B = false;
        } else {
            received_B = false;
        }
    }
  }
  return output;
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

uint8_t LPS::getLastCmd(){
  return rx_buff[9];
}

uint8_t LPS::getLastSender(){
  return rx_buff[7];
}

void LPS::commitData(){
  DW1000Ng::getReceivedData(rx_buff, LEN_DATA);
}

bool LPS::tagStartRTLS(){
  RangeInfrastructureResult res = DW1000NgRTLS::tagTwrLocalize(1500);
  if(res.success){
    //Serial.println("SUKSES");
    return true;
  }
  return false;
}

void LPS::getXY(double &x, double &y){
  memcpy(&x, &rx_buff[10], sizeof(double)); 
  memcpy(&y, &rx_buff[18], sizeof(double));
}







