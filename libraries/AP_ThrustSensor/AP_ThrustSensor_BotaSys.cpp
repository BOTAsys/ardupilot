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

extern const AP_HAL::HAL& hal;

#define LIGHTWARE_DIST_MAX_CM           10000
#define LIGHTWARE_OUT_OF_RANGE_ADD_CM   100

// read - return last value measured by sensor
bool AP_ThrustSensor_BotaSys::get_reading(float &reading_m)
{
    //CRC16 checksum calculation 
    #define lo8(x) ((x)&0xFF) 
    #define hi8(x) (((x)>>8)&0xFF) 
    inline uint16_t calcCrc16_x25(uint8_t *data, int len) 
    { 
        uint16_t crc = 0xFFFF; 
        while(len--) crc = crc_ccitt_update(crc, *data++); 
        return ~crc; 
    } 
    
    uint16_t crc_ccitt_update (uint16_t crc, uint8_t data)

    struct DataStatus __attribute__((__packed__)) 
    { 
        uint16_t app_took_too_long:1; 
        uint16_t overrange:1; 
        uint16_t invalid_measurements:1; 
        uint16_t raw_measurements:1; 
        uint16_t:12; //reserved 
    }; 
        
    struct AppOutput __attribute__((__packed__)) 
    { 
        DataStatus status; 
        float forces[6]; 
        uint32_t timestamp; 
        float temperature; 
    }; 

    struct __attribute__((__packed__)) RxFrame 
    { 
        uint8_t header; 
        AppOutput data; 
        uint16_t crc16_ccitt; 
    }frame;

    if (uart == nullptr) {
        return false;
    }

    const char frameHeader = 0xAA;
    bool frameSync_ = false;

    // read any available lines from the sensor
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        while (!frameSync_)
        {
            char possible_header = uart->read(); // or read(uint8_t *buffer, uint16_t count);
            if (frameHeader == possible_header)
            {
                uart->read((char*)frame, sizeof(frame) - sizeof(frame.header));
                if (frame.crc16_ccitt == calcCrc16X25(frame.data.bytes, sizeof(frame.data))) 
                { 
                    frameSync_ = true; 
                } 
                else 
                { 
                /* if there is a frame that included the header 0xAA in 
                 * a fixed position. Could be the above checking mechanism 
                 * will get stuck because will find the wrong value as header
                 * then will remove from the buffer n bytes where n the size 
                 * of the frame and then will find again exactly the same
                 * situation the wrong header. So we read on extra byte to make 
                 * sure next time will start from the position that is size of frame 
                 * plus 1. It works 
                 */ 
                char dummy; 
                uart->read((char*)&dummy, sizeof(dummy)); 
                } 
            } 
        } 
        if (frameSync_) 
        {   
            /* Read the sensor measurements frame assuming that is alligned with the RX buffer */ 
            uart->read((char*)frame, sizeof(frame)); 
            /* Check if the frame is still alligned, otherwise exit */ 
            if (frame.header != frameHeader) 
            { 
                frameSync_ = false;
                break; 
            } 
            // Read and check CRC 16-bit
            uint16_t crc_received = frame.crc16_ccitt; 
            uint16_t crc_computed = calcCrc16X25(frame.data.bytes, sizeof(frame.data));
            if (crc_received != crc_computed) 
            { 
                break;
                //skip this measurements 
            }
            // Do something with the measurements
            state.force_n = frame.data.force[2];
            
        }

        

        // use binary protocol
        if (protocol_state == ProtocolState::UNKNOWN || protocol_state == ProtocolState::BINARY) {
            bool msb_set = BIT_IS_SET(c, 7);
            if (msb_set) {
                // received the high byte
                high_byte = c;
                high_byte_received = true;
            } else {
                // received the low byte which should be second
                if (high_byte_received) {
                    const int16_t dist = (high_byte & 0x7f) << 7 | (c & 0x7f);
                    if (dist >= 0 && !is_lost_signal_distance(dist, distance_cm_max)) {
                        sum += dist * 0.01f;
                        valid_count++;
                        // if still determining protocol update binary valid count
                        if (protocol_state == ProtocolState::UNKNOWN) {
                            binary_valid_count++;
                        }
                    } else {
                        invalid_count++;
                    }
                }
                high_byte_received = false;
            }
        }
        
    }

    uint32_t now = AP_HAL::millis();
    if (last_init_ms == 0 ||
        (now - last_init_ms > 1000 &&
         now - state.last_reading_ms > 1000)) {
        // send enough serial transitions to trigger LW20 into serial
        // mode. It starts in dual I2C/serial mode, and wants to see
        // enough transitions to switch into serial mode.
        uart->write("www\r\n");
        last_init_ms = now;
    } else {
        uart->write('d');
    }

    // return average of all valid readings
    if (valid_count > 0) {
        reading_m = sum / valid_count;
        no_signal = false;
        return true;
    }

    // all readings were invalid so return out-of-range-high value
    if (invalid_count > 0) {
        //reading_m = MIN(MAX(LIGHTWARE_DIST_MAX_CM, distance_cm_max + LIGHTWARE_OUT_OF_RANGE_ADD_CM), UINT16_MAX) * 0.01f;
        no_signal = true;
        return true;
    }
*/
    // no readings so return false
    reading_m = 1.2345;
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