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
/*
#if THRUSTSENSOR_MAX_INSTANCES > 4
    // @Group: 5_
    // @Path: AP_ThrustSensor_Params.cpp
    AP_SUBGROUPINFO(params[4], "5_", 33, ThrustSensor, AP_ThrustSensor_Params),

    // @Group: 5_
    // @Path: AP_ThrustSensor_Wasp.cpp,AP_ThrustSensor_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[4], "5_",  61, ThrustSensor, backend_var_info[4]),
#endif

#if THRUSTSENSOR_MAX_INSTANCES > 5
    // @Group: 6_
    // @Path: AP_ThrustSensor_Params.cpp
    AP_SUBGROUPINFO(params[5], "6_", 35, ThrustSensor, AP_ThrustSensor_Params),

    // @Group: 6_
    // @Path: AP_ThrustSensor_Wasp.cpp,AP_ThrustSensor_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[5], "6_",  62, ThrustSensor, backend_var_info[5]),
#endif

#if THRUSTSENSOR_MAX_INSTANCES > 6
    // @Group: 7_
    // @Path: AP_ThrustSensor_Params.cpp
    AP_SUBGROUPINFO(params[6], "7_", 37, ThrustSensor, AP_ThrustSensor_Params),

    // @Group: 7_
    // @Path: AP_ThrustSensor_Wasp.cpp,AP_ThrustSensor_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[6], "7_",  63, ThrustSensor, backend_var_info[6]),
#endif

#if THRUSTSENSOR_MAX_INSTANCES > 7
    // @Group: 8_
    // @Path: AP_ThrustSensor_Params.cpp
    AP_SUBGROUPINFO(params[7], "8_", 39, ThrustSensor, AP_ThrustSensor_Params),

    // @Group: 8_
    // @Path: AP_ThrustSensor_Wasp.cpp,AP_ThrustSensor_Benewake_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[7], "8_",  64, ThrustSensor, backend_var_info[7]),
#endif*/
    

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
        state[i].offset_flag = false;
    }
    //offset();
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
            if ((Type)params[i].type.get() == Type::NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].status = Status::NotConnected;
                state[i].thrust_valid_count = 0;
                continue;
            }
            drivers[i]->update();
            state[i].force_norm = state[i].force_n/params[i].maxthrust;
            static uint8_t counter = 0;
            counter++;
            if (counter > 50) {
                counter = 0;
                //gcs().send_text(MAV_SEVERITY_INFO, "Thrust[%d]: %5.3f", i, (double)state[i].force_n);
                //gcs().send_text(MAV_SEVERITY_CRITICAL, "Offset[%d]: %5.3f", i, (double)state[i].offset_n);
             }
        }
    }
    
    //AP::logger().Write("TSBS", "TimeUS", "Thrust", "Qf", AP_HAL::micros64(), (double)(state->force_n));
#if HAL_LOGGING_ENABLED
    Log_THRS();
#endif
}

void ThrustSensor::offset(void)
{
    if (num_instances>0) {
        uint8_t N = 25;
        float sum[num_instances] = {0.0};
        for (uint8_t i=0; i<N; i++) {
            ThrustSensor::update();
            for (uint8_t j=0; j<num_instances; j++) {
                sum[j] += state[j].force_n;
                //gcs().send_text(MAV_SEVERITY_CRITICAL, "sum[%d] %d: %5.3f", j, i, (double)sum[j]);
            }
        }
        for (uint8_t k=0; k<num_instances; k++) {
            state[k].offset_n = sum[k]/N;
            state[k].offset_flag = true; 
        }
    }
}

float ThrustSensor::publish_thrust(uint8_t index) {
    //if (index > num_instances){
    //    return 0.0;
    //}
    for (uint8_t i=0; i < num_instances; i++) {
        //gcs().send_text(MAV_SEVERITY_CRITICAL, "params[%d]: %d", i, (uint8_t)params[i].motor);
        //gcs().send_text(MAV_SEVERITY_CRITICAL, "index: %d", index);
        if (index == (uint8_t)params[i].motor){
            return state[i].force_norm;
        }
    }
    /*
    switch (index)
    {
    case 0:
        //if (0 > num_instances){
        //return 0.0;
        //}
        return state[0].force_norm;
    case 1:
        if (1 > num_instances){
        return 0.0;
        }
        return state[1].force_norm;
    case 2:
        if (2 > num_instances){
        return 0.0;
        }
        return state[2].force_norm;
    case 3:
        if (3 > num_instances){
        return 0.0;
        }
        return state[3].force_norm;
    default:
        return 0.0;
    }
    */
   return 0.0;
}

bool ThrustSensor::publish_offset_flag(){
    for (uint8_t i=0; i < num_instances; i++){
        if (state[i].offset_flag == false){
            return false;
        }
    }
    return true;

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

void ThrustSensor::Log_THRS() const
{
    
    //if (_log_thrs_bit == uint32_t(-1)) {
    //    return;
    //}
//
    //AP_Logger &logger = AP::logger();
    //if (!logger.should_log(_log_thrs_bit)) {
    //    return;
    //}
    

    for (uint8_t i=0; i<THRUSTSENSOR_MAX_INSTANCES; i++) {
        const AP_ThrustSensor_Backend *s = get_backend(i);
        if (s == nullptr) {
            continue;
        }
        //gcs().send_text(MAV_SEVERITY_CRITICAL, "Trying Log");
        const struct log_THRS pkt = {
                LOG_PACKET_HEADER_INIT(LOG_THRS_MSG),
                time_us      : AP_HAL::micros64(),
                instance     : i,
                thrust       : s->force_n(),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}

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
        //case Status::NoOffset:
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
