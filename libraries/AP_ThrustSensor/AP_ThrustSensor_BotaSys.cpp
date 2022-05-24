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


#include "AP_ThrustSensor_BotaSys.h"

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define LIGHTWARE_DIST_MAX_CM           10000
#define LIGHTWARE_OUT_OF_RANGE_ADD_CM   100

//CRC16 checksum calculation 
#define lo8(x) ((x)&0xFF) 
#define hi8(x) (((x)>>8)&0xFF) 

uint16_t crc_ccitt_update (uint16_t crc, uint8_t data);

inline uint16_t calcCrc16X25(uint8_t *data, int len) 
{ 
    uint16_t crc = 0xFFFF; 
    while(len--) crc = crc_ccitt_update(crc, *data++); 
    return ~crc;
} 
    

 


// read - return last value measured by sensor
bool AP_ThrustSensor_BotaSys::get_reading(float &reading_m)
{
    
    // no readings so return false
    reading_m = 1.234;
    if (state.offset_flag) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "offset: %5.3f", (double)(state.offset_n));
        reading_m -= state.offset_n;
    }
    /*static uint8_t counter = 0;
    counter++;
    if (counter > 1) {
    counter = 0;
    gcs().send_text(MAV_SEVERITY_CRITICAL, "get_reading %5.3f", (double)(reading_m));
    }*/   
    return true;
}

/*
// check to see if distance returned by the LiDAR is a known lost-signal distance flag
bool AP_RangeFinder_LightWareSerial::is_lost_signal_distance(int16_t distance_cm, int16_t distance_cm_max)
{
    if (distance_cm < distance_cm_max + LIGHTWARE_OUT_OF_RANGE_ADD_CM) {
        // in-range
        return false;
    }
    const int16_t bad_distances[] { 13000, 16000, 23000, 25000 };
    for (const auto bad_distance_cm : bad_distances) {
        if (distance_cm == bad_distance_cm) {
            return true;
        }
    }
    return false;
}
*/