
#ifndef LPS_H
#define LPS_H

#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

#define LEN_DATA 25
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
#define ANCHOR_B 0x02
#define ANCHOR_MAIN 0x01
#define ANCHOR_C 0x03
#define START_TWR 4
#define TAG_FINISH 5
#define CALIBRATE 6
#define BROADCAST 0xFFFF
extern const uint8_t led;
extern const uint8_t PIN_RST;
extern const uint8_t PIN_IRQ;
extern const uint8_t PIN_SS;
extern char EUI[];
extern uint16_t PAN_ID;
extern uint8_t self_address;
extern uint32_t manualPower;
extern byte tx_packet[];
extern byte rx_buff[];
extern volatile boolean sentAck;
extern volatile boolean receiveAck;
extern uint64_t tof_A1_A2_uwb;

extern device_configuration_t DEFAULT_CONFIG;
extern frame_filtering_configuration_t FRAME_FILTER_CONFIG;
extern interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG;


enum STATE{
  ST_WAIT_POLLACK,
  ST_COMPLETE,
  ST_FAIL,
  ST_IDLE,
  ST_BROADCAST,
  ST_WAIT,
  ST_NEXT 
};
extern STATE state;

class LPS {
public:
    LPS();
    static void txHandler();
    static void rxHandler();
    void begin( uint8_t &device_address);
    void printModulInfo();
    void sendCmd(uint8_t target, uint8_t cmd);
    void startReceive();
    void commitData();
    void sendRange(uint8_t target);
    uint8_t getLastCmd();
    uint8_t getLastSender();
    void setTimePollReceived();
    void setTimePollAckSent();
    uint8_t getLastCmdSent();
    double calculateDistance();
    STATE waitLastAnchor();
    void restartTRX();
    void sendDistance(uint8_t target, double jarak);
    bool isTagFinish(uint8_t target);
    double getTransmitTimeStamp();
    void sendBroadcast();
    void setReplyDelay(uint16_t delay);
    void slaveAnchorBroadcast();
};

#endif