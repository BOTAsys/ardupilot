#include "AP_ThrustSensor_Params.h"
#include "AP_ThrustSensor.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_ThrustSensor_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Rangefinder type
    // @Description: Type of connected rangefinder
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLite-I2C,5:PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:USD1_Serial,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X or VL53L1X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:Benewake-Serial,21:LidarLightV3HP,22:PWM,23:BlueRoboticsPing,24:DroneCAN,25:BenewakeTFminiPlus-I2C,26:LanbaoPSK-CM8JL65-CC5,27:BenewakeTF03,28:VL53L1X-ShortRange,29:LeddarVu8-Serial,30:HC-SR04,31:GYUS42v2,32:MSP,33:USD1_CAN,34:Benewake_CAN,100:SITL
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_ThrustSensor_Params, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: SINCLENGTH
    // @DisplayName: Rangefinder pin
    // @Description: Analog or PWM input pin that rangefinder is connected to. Airspeed ports can be used for Analog input, AUXOUT can be used for PWM input. When using analog pin 103, the maximum value of the input in 3.3V. For PWM input, the pin must be configured as a digital GPIO, see the Wiki's "GPIOs" section for details.
    // @Values: -1:Not Used,11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6/Pixhawk2 ADC,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,103:Pixhawk SBUS
    // @User: Standard
    AP_GROUPINFO("SINCLENGTH",     2, AP_ThrustSensor_Params, sinc_length, 0),

    // @Param: SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts. For Maxbotix serial sonar this is unit conversion to meters.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("CHOP", 3, AP_ThrustSensor_Params, chop, 0),

    // @Param: OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM lidars
    // @Units: V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("FAST",  4, AP_ThrustSensor_Params, fast, 0),

    // @Param: FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Standard
    AP_GROUPINFO("SKIP", 5, AP_ThrustSensor_Params, skip, 0),

    // @Param: FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Standard
    AP_GROUPINFO("MAXTHRUST", 6, AP_ThrustSensor_Params, maxthrust, 0.0),



    AP_GROUPEND
};

AP_ThrustSensor_Params::AP_ThrustSensor_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
