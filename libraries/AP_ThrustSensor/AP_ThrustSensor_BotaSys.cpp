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
//

uint16_t AP_ThrustSensor_BotaSys::crc16_mcrf4xx(uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    if (!data || len == 0)
        return crc;

    while (len--) {
        crc ^= *data++;
        for (int i=0; i<8; i++) {
            if (crc & 1)  crc = (crc >> 1) ^ 0x8408;
            else          crc = (crc >> 1);
        }
    }
    return crc;
}

// read - return last value measured by sensor
bool AP_ThrustSensor_BotaSys::get_reading(float &reading_m)
{
    union DataStatus
    {
        struct __attribute__((__packed__))  
        { 
            uint16_t app_took_too_long:1; 
            uint16_t overrange:1; 
            uint16_t invalid_measurements:1; 
            uint16_t raw_measurements:1; 
            uint16_t:12; //reserved 
        }; 
        uint8_t bytes[1];
    };

    union AppOutput
    {
        struct __attribute__((__packed__))   
        { 
            DataStatus status; 
            float forces[6]; 
            uint32_t timestamp; 
            float temperature; 
        }; 
        uint8_t bytes[1];
    };

    union RxFrame 
    {
        struct __attribute__((__packed__))  
        { 
            uint8_t header; 
            AppOutput data; 
            uint16_t crc; 
        };
        uint8_t bytes[1];
    }frame;

    const char frameHeader = 0xAA;
//    static bool frameSync_ = false;

    // read any available lines from the sensor
    int16_t nbytes = uart->available();
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "nbytes: %d", nbytes);
//    if (nbytes)
//    {
//        char possible_header = uart->read(); // or read(uint8_t *buffer, uint16_t count);
//        gcs().send_text(MAV_SEVERITY_CRITICAL, "possible_header: %d", possible_header);
//    }
    if (nbytes>=sizeof(frame)) {
//        while (!frameSync_)
//        {
//            char possible_header = uart->read(); // or read(uint8_t *buffer, uint16_t count);
//            gcs().send_text(MAV_SEVERITY_CRITICAL, "possible_header: %d", possible_header);
//            if (frameHeader == possible_header)
//            {
//                uart->read((uint8_t*)&frame.data, sizeof(frame) - sizeof(frame.header));
//                gcs().send_text(MAV_SEVERITY_CRITICAL, "frame.crc: %d", frame.crc);
//                gcs().send_text(MAV_SEVERITY_CRITICAL, "calc_crc: %d", crc16_mcrf4xx(frame.data.bytes, sizeof(frame.data)));
//                if (frame.crc == crc16_mcrf4xx(frame.data.bytes, sizeof(frame.data))) 
//                { 
//                    frameSync_ = true; 
//                } 
//                else 
//                { 
//                /* if there is a frame that included the header 0xAA in 
//                 * a fixed position. Could be the above checking mechanism 
//                 * will get stuck because will find the wrong value as header
//                 * then will remove from the buffer n bytes where n the size 
//                 * of the frame and then will find again exactly the same
//                 * situation the wrong header. So we read on extra byte to make 
//                 * sure next time will start from the position that is size of frame 
//                 * plus 1. It works 
//                 */ 
//                char dummy; 
//                uart->read((uint8_t*)&dummy, sizeof(dummy)); 
//                } 
//            } 
//        } 
        if (1) 
        {   
            /* Read the sensor measurements frame assuming that is alligned with the RX buffer */ 
            uart->read((uint8_t*)&frame, sizeof(frame)); 
            //gcs().send_text(MAV_SEVERITY_CRITICAL, "frame.header: %d", frame.header);
            //gcs().send_text(MAV_SEVERITY_CRITICAL, "frame.crc: %x", frame.crc);
            //gcs().send_text(MAV_SEVERITY_CRITICAL, "calc crc: %x", crc16_mcrf4xx(frame.data.bytes, sizeof(frame.data)));
            /* Check if the frame is still alligned, otherwise exit */ 
            if (frame.header != frameHeader) 
            { 
                uart->read();
            //    frameSync_ = false;
                //break; 
            } 
            // Read and check CRC 16-bit
            if (frame.crc!=crc16_mcrf4xx(frame.data.bytes, sizeof(frame.data))) 
            { 
                //break;
                //skip this measurements 
            }
            // Do something with the measurements
            state.force_n = frame.data.forces[2];

            reading_m = state.force_n ;
            //reading_m = 1.234;
            if (state.offset_flag) {
                //gcs().send_text(MAV_SEVERITY_CRITICAL, "offset: %5.3f", (double)(state.offset_n));
                //reading_m -= state.offset_n;
            }
            //gcs().send_text(MAV_SEVERITY_CRITICAL, "reading_m: %5.3f", reading_m);
            /*static uint8_t counter = 0;
            counter++;
            if (counter > 1) {
            counter = 0;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "get_reading %5.3f", (double)(reading_m));
            }*/   
            return true;
            
        } 
    }
    return false;
}

 


//// read - return last value measured by sensor
//bool AP_ThrustSensor_BotaSys::get_reading(float &reading_m)
//{
//    
//    // no readings so return false
//    reading_m = 1.234;
//    if (state.offset_flag) {
//        gcs().send_text(MAV_SEVERITY_CRITICAL, "offset: %5.3f", (double)(state.offset_n));
//        reading_m -= state.offset_n;
//    }
//    /*static uint8_t counter = 0;
//    counter++;
//    if (counter > 1) {
//    counter = 0;
//    gcs().send_text(MAV_SEVERITY_CRITICAL, "get_reading %5.3f", (double)(reading_m));
//    }*/   
//    return true;
//}
