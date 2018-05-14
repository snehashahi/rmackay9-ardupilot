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
#include "AP_RangeFinder_LightWareSerial.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_RangeFinder_NMEA.h"

extern const AP_HAL::HAL& hal;

// Convenience macros
#define DIGIT_TO_VAL(_x)        (_x - '0')
#define hexdigit(x) ((x)>9?'A'+((x)-10):'0'+(x))

// constructor initialises the rangefinder
// Note this is called after detect() returns true, so we
// already know that we should setup the rangefinder
AP_RangeFinder_NMEA::AP_RangeFinder_NMEA(RangeFinder::RangeFinder_State &_state,
                                         AP_SerialManager &serial_manager,
                                         uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
    }
}

// detect if a NMEA rangefinder by looking to see if the user has configured it
bool AP_RangeFinder_NMEA::detect(AP_SerialManager &serial_manager, uint8_t serial_instance)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

// update the state of the sensor
void AP_RangeFinder_NMEA::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        _last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - _last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

// return last value measured by sensor
bool AP_RangeFinder_NMEA::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the lidar
    float sum = 0;
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        if (decode(c)) {
            count++;
        }
    }

    // return false on failure
    if (count == 0) {
        return false;
    }

    // return average of all measurements
    reading_cm = 100.0f * sum / count;
    return true;
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded
bool AP_RangeFinder_NMEA::decode(char c)
{
    bool valid_sentence = false;

    switch (c) {
    case ',':
        // end of a term, add to checksum
        _checksum ^= c;
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*':
        if (_term_offset < sizeof(_term)) {
            _term[_term_offset] = 0;
            valid_sentence = decode_latest_term();
        }
        _term_number++;
        _term_offset = 0;
        _term_is_checksum = (c == '*');
        return valid_sentence;

    case '$': // sentence begin
        _sentence_ok = false;
        _sentence_type = SONAR_UNKNOWN;
        _term_number = 0;
        _term_offset = 0;
        _checksum = 0;
        _term_is_checksum = false;
        return valid_sentence;
    }

    // ordinary characters are added to term
    if (_term_offset < sizeof(_term) - 1)
        _term[_term_offset++] = c;
    if (!_term_is_checksum)
        _checksum ^= c;

    return valid_sentence;
}

// decode the most recently consumed term
// returns true if new sentence has just passed checksum test and is validated
bool AP_RangeFinder_NMEA::decode_latest_term()
{
    // handle the last term in a message
    if (_term_is_checksum) {
        uint8_t checksum = 16 * char_to_hex(_term[0]) + char_to_hex(_term[1]);
        if (checksum == _checksum) {
            if (_sentence_ok) {
                uint32_t now = AP_HAL::millis();
                switch (_sentence_type) {
                case SONAR_DBT:
                    _last_DBT_ms = now;
                    _depth = 0.0f;
                    break;
                case SONAR_DPT:
                    _last_DPT_ms = now;
                    _depth = 0.0f;
                    break;
                }
            }
            // see if we got a good message
            return have_new_message();
        }
        // we got a bad message, ignore it
        return false;
    }

    // the first term determines the sentence type
    if (_term_number == 0) {
        /*
          The first two letters of the NMEA term are the talker
          ID. The most common is 'GP' but there are a bunch of others
          that are valid. We accept any two characters here.
         */
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
            _sentence_type = SONAR_UNKNOWN;
            return false;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "DBT") == 0) {
            _sentence_type = SONAR_DBT;
        } else if (strcmp(term_type, "DPT") == 0) {
            _sentence_type = SONAR_DPT;
        } else {
            _sentence_type = SONAR_UNKNOWN;
        }
        return false;
    }

    // parse DBT messages
    if (_sentence_type == SONAR_DBT) {
        if (_term_number == 3) {
            _depth = atof(_term);
        }
    }

    // parse DPT messages
    if (_sentence_type == SONAR_DPT) {
        if (_term_number == 2) {
            _depth = atof(_term);
        }
    }

    return false;
}

// return true if we have a new set of NMEA message
bool AP_RangeFinder_NMEA::have_new_message()
{
    if (_last_DBT_ms == 0 ||
        _last_DPT_ms == 0) {
        return false;
    }
    uint32_t now = AP_HAL::millis();
    if (now - _last_DBT_ms > 150 ||
        now - _last_DPT_ms > 150) {
        return false;
    }
    _last_DBT_ms = 1;
    _last_DPT_ms = 1;
    return true;
}

// return the numeric value of an ascii hex character
int16_t AP_RangeFinder_NMEA::char_to_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}
