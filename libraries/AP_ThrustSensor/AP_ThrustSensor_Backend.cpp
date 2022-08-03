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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_ThrustSensor.h"
#include "AP_ThrustSensor_Backend.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_ThrustSensor_Backend::AP_ThrustSensor_Backend(ThrustSensor::ThrustSensor_State &_state, AP_ThrustSensor_Params &_params) :
        state(_state),
		params(_params) 
{
    _backend_type = type();
}

ThrustSensor::Status AP_ThrustSensor_Backend::status() const {
    if (type() == ThrustSensor::Type::NONE) {
        // turned off at runtime?
        return ThrustSensor::Status::NotConnected;
    }
    return state.status;
}

// true if sensor is returning data
bool AP_ThrustSensor_Backend::has_data() const {
    return ((state.status != ThrustSensor::Status::NotConnected) &&
            (state.status != ThrustSensor::Status::NoData));
}

// update status based on distance measurement
void AP_ThrustSensor_Backend::update_status()
{
    if (!state.offset_flag) {
        set_status(ThrustSensor::Status::NoOffset);
    }
    else {
        set_status(ThrustSensor::Status::Good);
    }
    //out of range code should be implemented here
}

// set status and update valid count
void AP_ThrustSensor_Backend::set_status(ThrustSensor::Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == ThrustSensor::Status::Good) {
        if (state.thrust_valid_count < 10) {
            state.thrust_valid_count++;
        }
    } else {
        state.thrust_valid_count = 0;
    }
}

void AP_ThrustSensor_Backend::init_serial(uint8_t serial_instance)
{
    instance = serial_instance;
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ThrustSensor, serial_instance);
    if (uart != nullptr) {
        uart->begin(initial_baudrate(serial_instance), rx_bufsize(), tx_bufsize());
    }
}

uint32_t AP_ThrustSensor_Backend::initial_baudrate(const uint8_t serial_instance) const
{
    return AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_ThrustSensor, serial_instance);
}

/*
   detect if a Serial thrustsensor is connected. We'll detect by simply
   checking for SerialManager configuration
*/
bool AP_ThrustSensor_Backend::detect(uint8_t serial_instance)
{
    return AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_ThrustSensor, serial_instance);
}


/*
   update the state of the sensor
*/
void AP_ThrustSensor_Backend::update(void)
{
    if (get_reading(state.force_n)) {
        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > read_timeout_ms()) {
        set_status(ThrustSensor::Status::NoData);
    }
}
