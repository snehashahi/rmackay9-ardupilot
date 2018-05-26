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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_Benewake.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

#define BENEWAKE_FRAME_HEADER 0x59
#define BENEWAKE_FRAME_LENGTH 9

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_Benewake::AP_RangeFinder_Benewake(RangeFinder::RangeFinder_State &_state,
                                                             AP_SerialManager &serial_manager,
                                                             uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
    }
}

/* 
   detect if a Benewake rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_Benewake::detect(AP_SerialManager &serial_manager, uint8_t serial_instance)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_Benewake::get_reading(uint16_t &reading_cm, bool &signal_ok)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_cm = 0;
    uint16_t count = 0;
    bool sig_ok = false;

    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        // if buffer is empty and this byte is 0x59, add to buffer, continue
        // if buffer has 1 element and this byte is 0x5, add it to buffer, if not continue (this leaves identical 0x59 at head of buffer)
        // add byte to buffer
        // if buffer now has 9 items try to decode it
        //     loop through and calculate checksum
        //     compare checksum
        //     extract distance and strength

        // if buffer is empty and this byte is 0x59, add to buffer
        if (linebuf_len == 0) {
            if (c == BENEWAKE_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            }
        } else if (linebuf_len == 1) {
            // if buffer has 1 element and this byte is 0x59, add it to buffer
            // if not clear the buffer
            if (c == BENEWAKE_FRAME_HEADER) {
                linebuf[linebuf_len++] = c;
            } else {
                linebuf_len = 0;
            }
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 9 items try to decode it
            if (linebuf_len == BENEWAKE_FRAME_LENGTH) {
                // calculate checksum
                uint8_t checksum = 0;
                for (uint8_t i=0; i<BENEWAKE_FRAME_LENGTH-1; i++) {
                    checksum += linebuf[i];
                }
                // if checksum matches extract contents
                if (checksum == linebuf[BENEWAKE_FRAME_LENGTH-1]) {
                    // distance
                    be16_t dist_val = ((uint16_t)linebuf[2] << 8) | linebuf[3];
                    uint16_t dist_cm = be16toh(dist_val);
                    // strength
                    be16_t strength_val = ((uint16_t)linebuf[4] << 8) | linebuf[5];
                    uint16_t strength = be16toh(strength_val);
                    // signal reliability
                    uint8_t sig = linebuf[6];
                    uint8_t time_val = linebuf[7];
                    // add distance to sum
                    sum_cm += dist_cm;
                    count++;
                    // a single reading of good strength results in confidence in the average distance
                    sig_ok = (sig >= 7);
                    // debug
                    ::printf("d:%u s:%u s:%u t:%d buf:%d %d %d %d %d %d %d %d %d\n",
                             (unsigned)dist_cm,
                             (unsigned)strength,
                             (unsigned)sig,
                             (unsigned)time_val,
                             (unsigned)linebuf[0],
                             (unsigned)linebuf[1],
                             (unsigned)linebuf[2],
                             (unsigned)linebuf[3],
                             (unsigned)linebuf[4],
                             (unsigned)linebuf[5],
                             (unsigned)linebuf[6],
                             (unsigned)linebuf[7],
                             (unsigned)linebuf[8],
                             (unsigned)linebuf[9]
                             );
                } else {
                    // debug
                    ::printf("CS exp:%u got:%u\n", (unsigned)checksum,(unsigned)linebuf[BENEWAKE_FRAME_LENGTH-1]);
                }
                // clear buffer
                linebuf_len = 0;
            }
        }
    }

    if (count == 0) {
        return false;
    }
    reading_cm = sum_cm / count;
    signal_ok = sig_ok;
    return true;
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_Benewake::update(void)
{
    bool signal_ok;
    if (get_reading(state.distance_cm, signal_ok)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        if (signal_ok) {
            update_status();
        } else {
            // if signal is weak set status to out-of-range
            set_status(RangeFinder::RangeFinder_OutOfRangeHigh);
        }
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
