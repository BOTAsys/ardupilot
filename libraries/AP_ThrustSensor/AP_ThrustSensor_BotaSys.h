#pragma once

#include "AP_ThrustSensor.h"
#include "AP_ThrustSensor_Backend.h"

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
