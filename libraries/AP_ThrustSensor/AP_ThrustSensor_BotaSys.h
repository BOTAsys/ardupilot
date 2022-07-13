#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_ThrustSensor.h"
#include "AP_ThrustSensor_Backend.h"

class BotaForceTorqueSensorComm
{
private:
  bool _synced;
  uint32_t _crc_err_count;
  union DataStatus
  {
      struct __attribute__((__packed__))  
      { 
          uint16_t app_took_too_long:1; 
          uint16_t overrange:1; 
          uint16_t invalid_measurements:1; 
          uint16_t raw_measurements:1; 
          uint16_t:12; //reserved 
      }; 
      uint16_t val;
      uint8_t bytes[1];
  };
  union AppOutput
  {
      struct __attribute__((__packed__))   
      { 
          DataStatus status; 
          float forces[6]; 
          uint32_t timestamp; 
          float temperature; 
      }; 
      uint8_t bytes[1];
  };
  union RxFrame 
  {
      struct __attribute__((__packed__))  
      { 
          uint8_t header; 
          AppOutput data; 
          uint16_t crc; 
      };
      uint8_t bytes[1];
  };
  
public:
  enum ReadFrameRes {NO_FRAME, VALID_FRAME, NOT_VALID_FRAME, NOT_ALLIGNED_FRAME};
  BotaForceTorqueSensorComm();
  static uint16_t crc16_x25(uint8_t* data, size_t len);
  static uint16_t crc16_ccitt_false(uint8_t* data, size_t len);
  static uint16_t crc16_mcrf4xx(uint8_t *data, size_t len);
  bool isCrcOk();
  bool checkSync();
  bool isSynced() {return _synced;}
  uint32_t get_crc_count() {return _crc_err_count;}
  virtual int serialAvailable() = 0;
  virtual int serialReadBytes(uint8_t* data, size_t len) = 0;
  ReadFrameRes readFrame();
  RxFrame frame;
};


class AP_BotaForceTorqueSensorComm : public BotaForceTorqueSensorComm
{
  public:
  AP_HAL::UARTDriver *serial;
  int serialReadBytes(uint8_t* data, size_t len) override {return serial->read(data, len);}
  int serialAvailable() override {return serial->available();}
};

class AP_ThrustSensor_BotaSys : public AP_ThrustSensor_Backend
{

public:

    using AP_ThrustSensor_Backend::AP_ThrustSensor_Backend;

protected:

    bool get_signal_quality_pct(uint8_t &quality_pct) const override {
        quality_pct = no_signal ? 0 : 100;
        return true;
    }

private:
    AP_BotaForceTorqueSensorComm sensorComm;
    // get a reading
    bool get_reading(float &reading_m) override;
    //bool is_lost_signal_distance(int16_t distance_cm, int16_t distance_cm_max);

    char linebuf[10];           // legacy protocol buffer
    uint8_t linebuf_len;        // legacy protocol buffer length
    uint32_t last_init_ms;      // init time used to switch lw20 to serial mode
    uint8_t high_byte;          // binary protocol high byte
    bool high_byte_received;    // true if high byte has been received

    // automatic protocol decision variables
    enum class ProtocolState {
        UNKNOWN,    // the protocol used is not yet known
        LEGACY,     // legacy protocol, distances are sent as strings
        BINARY      // binary protocol, distances are sent using two bytes
    } protocol_state;
    uint8_t legacy_valid_count;
    uint8_t binary_valid_count;

    bool no_signal = false;
};
