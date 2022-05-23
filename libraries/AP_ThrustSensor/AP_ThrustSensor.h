/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_MSP/msp.h>
#include "AP_ThrustSensor_Params.h"

#ifndef AP_THRUSTSENSOR_ENABLED
#define AP_THRUSTSENSOR_ENABLED 1
#endif

// Maximum number of range finder instances available on this platform
#ifndef THRUSTSENSOR_MAX_INSTANCES 
  #if AP_THRUSTSENSOR_ENABLED
  #define THRUSTSENSOR_MAX_INSTANCES 8
  #else
  #define THRUSTSENSOR_MAX_INSTANCES 1
  #endif
#endif

/*
#define THRUSTSENSOR_GROUND_CLEARANCE_CM_DEFAULT 10
#define THRUSTSENSOR_PREARM_ALT_MAX_CM           200
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define THRUSTSENSOR_PREARM_REQUIRED_CHANGE_CM   0
#else
#define THRUSTSENSOR_PREARM_REQUIRED_CHANGE_CM   50
#endif

#ifndef HAL_MSP_THRUSTSENSOR_ENABLED
#define HAL_MSP_THRUSTSENSOR_ENABLED HAL_MSP_ENABLED && !HAL_MINIMIZE_FEATURES
#endif
*/
class AP_ThrustSensor_Backend;

class ThrustSensor
{
    friend class AP_ThrustSensor_Backend;

public:
    ThrustSensor();

    /* Do not allow copies */
    ThrustSensor(const ThrustSensor &other) = delete;
    ThrustSensor &operator=(const ThrustSensor&) = delete;

    enum class Type {
        NONE   = 0,
        BOTA   = 1,
    };

    enum class Status {
        NotConnected = 0,
        NoData,
        NoSync,
        Good
    };

    // The ThrustSensor_State structure is filled in by the backend driver
    struct ThrustSensor_State {
        float force_n;               // force in newtons
        uint16_t voltage_mv;            // voltage in millivolts, if applicable, otherwise 0
        enum ThrustSensor::Status status; // sensor status
        uint8_t  thrust_valid_count;     // number of consecutive valid readings (maxes out at 10)
        uint32_t last_reading_ms;       // system time of last successful update from sensor

        const struct AP_Param::GroupInfo *var_info;
    };

    static const struct AP_Param::GroupInfo *backend_var_info[THRUSTSENSOR_MAX_INSTANCES];

    // parameters for each instance
    static const struct AP_Param::GroupInfo var_info[];

    //void set_log_rfnd_bit(uint32_t log_rfnd_bit) { _log_rfnd_bit = log_rfnd_bit; }

    /*
      Return the number of range finder instances. Note that if users
      sets up rangefinders with a gap in the types then this is the
      index of the maximum sensor ID plus one, so this gives the value
      that should be used when iterating over all sensors
    */
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // prearm checks
    bool prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const;

    // detect and initialise any available rangefinders
    void init(void);

    // update state of all rangefinders. Should be called at around
    // 10Hz from main loop
    void update(void);

    AP_ThrustSensor_Backend *get_backend(uint8_t id) const;

    // get rangefinder type for an ID
    Type get_type(uint8_t id) const {
        return id >= THRUSTSENSOR_MAX_INSTANCES? Type::NONE : Type(params[id].type.get());
    }

    static ThrustSensor *get_singleton(void) { return _singleton; }

protected:
    AP_ThrustSensor_Params params[THRUSTSENSOR_MAX_INSTANCES];

private:
    static ThrustSensor *_singleton;

    ThrustSensor_State state[THRUSTSENSOR_MAX_INSTANCES];
    AP_ThrustSensor_Backend *drivers[THRUSTSENSOR_MAX_INSTANCES];
    uint8_t num_instances;
    bool init_done;
    HAL_Semaphore detect_sem;

    void detect_instance(uint8_t instance, uint8_t& serial_instance);

    bool _add_backend(AP_ThrustSensor_Backend *driver, uint8_t instance, uint8_t serial_instance=0);

    //uint32_t _log_rfnd_bit = -1;
    //void Log_RFND() const;
};

namespace AP {
    ThrustSensor *thrustsensor();
};
