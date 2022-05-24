#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_ThrustSensor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_ThrustSensor_Params(void);

    /* Do not allow copies */
    AP_ThrustSensor_Params(const AP_ThrustSensor_Params &other) = delete;
    AP_ThrustSensor_Params &operator=(const AP_ThrustSensor_Params&) = delete;

    AP_Int8  type;
    AP_Int16 sinc_length;
    AP_Int8 chop;
    AP_Int8 fast;
    AP_Int8 skip;
    AP_Float maxthrust;
};
