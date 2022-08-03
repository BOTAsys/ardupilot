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
#include "AP_ThrustSensor.h"
#include <Filter/LowPassFilter.h>
#include <Filter/LowPassFilter2p.h>
#include <Filter/AverageFilter.h>
#include <Filter/NotchFilter.h>
#include <Filter/HarmonicNotchFilter.h>

class AP_ThrustSensor_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_ThrustSensor_Backend(ThrustSensor::ThrustSensor_State &_state, AP_ThrustSensor_Params &_params);

    // we declare a virtual destructor so that ThrustSensor drivers can
    // override with a custom destructor if need be
    virtual ~AP_ThrustSensor_Backend(void) {}

    // update the state structure
    //virtual void update() = 0;
    void update();
    void init_serial(uint8_t serial_instance);
    //virtual void init_serial(uint8_t serial_instance) {};
    static bool detect(uint8_t serial_instance);

    float force_n() const { return state.force_n; }
    float force_filt_n() const { return state.force_filt_n;}
    float offset_n() const { return state.offset_n; }
    float temp_c() const { return state.temp_c;}
    ThrustSensor::Status status() const;
    ThrustSensor::Type type() const { return (ThrustSensor::Type)params.type.get(); }

    // true if sensor is returning data
    bool has_data() const;

    // returns count of consecutive good readings
    uint8_t thrust_valid_count() const { return state.thrust_valid_count; }

    // return system time of last successful read from the sensor
    uint32_t last_reading_ms() const { return state.last_reading_ms; }

    // 0 is no return value, 100 is perfect.  false means signal
    // quality is not available
    virtual bool get_signal_quality_pct(uint8_t &quality_pct) const { return false; }
    LowPassFilterFloat lpf{400.0, 10.0};
    LowPassFilter2pFloat lpf2{400.0, 10.0};
    AverageFilterFloat_Size5 avg{};
    NotchFilterFloat notch;    

protected:

    // update status based on distance measurement
    void update_status();

    // set status and update valid_count
    void set_status(ThrustSensor::Status status);

    ThrustSensor::ThrustSensor_State &state;
    AP_ThrustSensor_Params &params;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

    //Type Backend initialised with
    ThrustSensor::Type _backend_type;
;
    // baudrate used during object construction:
    virtual uint32_t initial_baudrate(uint8_t serial_instance) const;

    // the value 0 is special to the UARTDriver - it's "use default"
    virtual uint16_t rx_bufsize() const { return 0; }
    virtual uint16_t tx_bufsize() const { return 0; }

    AP_HAL::UARTDriver *uart = nullptr;
    uint8_t instance;

    // update state; not all backends call this!
    //virtual void update(void) override;

    // it is essential that anyone relying on the base-class update to
    // implement this:
    virtual bool get_reading(float &reading_m) = 0;

    // maximum time between readings before we change state to NoData:
    virtual uint16_t read_timeout_ms() const { return 200; }
};
