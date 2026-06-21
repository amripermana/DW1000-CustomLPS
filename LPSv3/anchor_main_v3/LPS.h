#ifndef LPS_H
#define LPS_H

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

#define LEN_DATA 26
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define START_TWR 4
#define RANGE_FAILED 255
#define XY_REPORT 5
extern const uint8_t led;

enum State{
  IDLE,
  ACTIVE
};


class LPS{
  public:
    LPS();
    void begin(uint8_t device_address);
    void printModulInfo();
    void calculatePosition(double &x, double &y);
    bool startRTLS(uint8_t target, double &outX, double &outY);
    void sendCmd(uint8_t target, uint8_t cmd);
    void startReceive();
    void sendXY(uint8_t target, double x, double y);
    
};


#endif