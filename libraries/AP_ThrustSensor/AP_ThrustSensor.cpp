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

#include "AP_ThrustSensor.h"
#include "AP_ThrustSensor_BotaSys.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo ThrustSensor::var_info[] = {

	// @Group: 1_
	// @Path: AP_ThrustSensor_Params.cpp
	AP_SUBGROUPINFO(params[0], "1_", 25, ThrustSensor, AP_ThrustSensor_Params),

    // @Group: 1_
    // @Path: AP_ThrustSensor_Wasp.cpp,AP_ThrustSensor_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  57, ThrustSensor, backend_var_info[0]),

#if THRUSTSENSOR_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_ThrustSensor_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, ThrustSensor, AP_ThrustSensor_Params),

    // @Group: 2_
    // @Path: AP_ThrustSensor_Wasp.cpp,AP_ThrustSensor_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  58, ThrustSensor, backend_var_info[1]),
#endif

#if THRUSTSENSOR_MAX_INSTANCES > 2
    // @Group: 3_
    // @Path: AP_ThrustSensor_Params.cpp
    AP_SUBGROUPINFO(params[2], "3_", 29, ThrustSensor, AP_ThrustSensor_Params),

    // @Group: 3_
    // @Path: AP_ThrustSensor_Wasp.cpp,AP_ThrustSensor_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_",  59, ThrustSensor, backend_var_info[2]),
#endif

#if THRUSTSENSOR_MAX_INSTANCES > 3
    // @Group: 4_
    // @Path: AP_ThrustSensor_Params.cpp
    AP_SUBGROUPINFO(params[3], "4_", 31, ThrustSensor, AP_ThrustSensor_Params),

    // @Group: 4_
    // @Path: AP_ThrustSensor_Wasp.cpp,AP_ThrustSensor_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[3], "4_",  60, ThrustSensor, backend_var_info[3]),
#endif
    
    AP_GROUPEND
};

const AP_Param::GroupInfo *ThrustSensor::backend_var_info[THRUSTSENSOR_MAX_INSTANCES];

ThrustSensor::ThrustSensor()
{
    AP_Param::setup_object_defaults(this, var_info);

    _singleton = this;
}


/*
  initialise the ThrustSensor class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  rangefinders.
 */
void ThrustSensor::init(void)
{
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "Init called");
    if (init_done) {
        // init called a 2nd time?
        return;
    }
    init_done = true;

    for (uint8_t i=0, serial_instance = 0; i<THRUSTSENSOR_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        WITH_SEMAPHORE(detect_sem);
        detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy). We use MAX()
            // here as a UAVCAN rangefinder may already have been
            // found
            num_instances = i+1;
        }

        // initialise status
        state[i].status = Status::NotConnected;
        state[i].thrust_valid_count = 0;
    }
}

/*
  update ThrustSensor state for all instances. This should be called at
  around 10Hz by main loop
 */
void ThrustSensor::update(void)
{
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "num_instances %5.3f", (double)num_instances);
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "init_done %5.3f", (double)init_done);
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            //gcs().send_text(MAV_SEVERITY_CRITICAL, "Drivers available");
            /*if ((Type)params[i].type.get() == Type::NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].status = Status::NotConnected;
                state[i].thrust_valid_count = 0;
                continue;
            }*/
            drivers[i]->update();   
            static uint8_t counter = 0;
            counter++;
            if (counter > 50) {
                counter = 0;
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Thrust %5.3f", (double)state[i].force_n);
             }
        }
    }
    
    //AP::logger().Write("TSBS", "TimeUS", "Thrust", "Qf", AP_HAL::micros64(), (double)(state->force_n));
#if HAL_LOGGING_ENABLED
    //Log_RFND();
#endif
}

bool ThrustSensor::_add_backend(AP_ThrustSensor_Backend *backend, uint8_t instance, uint8_t serial_instance)
{
    if (!backend) {
        return false;
    }
    if (instance >= THRUSTSENSOR_MAX_INSTANCES) {
        AP_HAL::panic("Too many THRUSTSENSOR backends");
    }
    if (drivers[instance] != nullptr) {
        // we've allocated the same instance twice
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    backend->init_serial(serial_instance);
    drivers[instance] = backend;
    num_instances = MAX(num_instances, instance+1);

    return true;
}

/*
  detect if an instance of a rangefinder is connected. 
 */
void ThrustSensor::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
#if AP_THRUSTSENSOR_ENABLED
    //const Type _type = (Type)params[instance].type.get();
    const Type _type = Type::BOTA;
    switch (_type) {
    
        break;
    case Type::BOTA:
        if (AP_ThrustSensor_BotaSys::detect(serial_instance)) {
            _add_backend(new AP_ThrustSensor_BotaSys(state[instance], params[instance]), instance, serial_instance++);
        }
        break;
    case Type::NONE:
    default:
        break;

    }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);

        // param count could have changed
        AP_Param::invalidate_count();
    }
#endif //AP_THRUSTSENSOR_ENABLED
}

AP_ThrustSensor_Backend *ThrustSensor::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == Type::NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

/*
// Write an RFND (rangefinder) packet
void ThrustSensor::Log_RFND() const
{
    if (_log_rfnd_bit == uint32_t(-1)) {
        return;
    }

    AP_Logger &logger = AP::logger();
    if (!logger.should_log(_log_rfnd_bit)) {
        return;
    }

    for (uint8_t i=0; i<THRUSTSENSOR_MAX_INSTANCES; i++) {
        const AP_ThrustSensor_Backend *s = get_backend(i);
        if (s == nullptr) {
            continue;
        }

        const struct log_RFND pkt = {
                LOG_PACKET_HEADER_INIT(LOG_RFND_MSG),
                time_us      : AP_HAL::micros64(),
                instance     : i,
                dist         : s->distance_cm(),
                status       : (uint8_t)s->status(),
                orient       : s->orientation(),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}
*/
bool ThrustSensor::prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const
{
    for (uint8_t i = 0; i < THRUSTSENSOR_MAX_INSTANCES; i++) {
        if ((Type)params[i].type.get() == Type::NONE) {
            continue;
        }

        if (drivers[i] == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "Thrustsensor %X: Not Detected", i + 1);
            return false;
        }

        switch (drivers[i]->status()) {
        case Status::NoData:
            hal.util->snprintf(failure_msg, failure_msg_len, "Thrustsensor %X: No Data", i + 1);
            return false;
        case Status::NotConnected:
            hal.util->snprintf(failure_msg, failure_msg_len, "Thrustsensor %X: Not Connected", i + 1);
            return false;
        case Status::NoSync:
        //case Status::OutOfRangeHigh:
        case Status::Good:  
            break;
        }
    }

    return true;
}

ThrustSensor *ThrustSensor::_singleton;

namespace AP {

ThrustSensor *thrustsensor()
{
    return ThrustSensor::get_singleton();
}

}
