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
#include "AP_RangeFinder_LeddarVu.h"
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_LeddarVu::AP_RangeFinder_LeddarVu(RangeFinder &_ranger, uint8_t instance,
                                                               RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0));
    }
}

/*
   detect if a LeddarVu rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_LeddarVu::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0) != nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_LeddarVu::get_reading(uint16_t &reading_cm)
{
    uint8_t number_detections;
    LeddarVu_Status leddarvu_status;

    if (uart == nullptr) {
        return false;
    }

    switch (modbus_status) {
    case LEDDARVU_MODBUS_STATE_INIT: {
        uint8_t index = 0;
        // clear read buffer
        uint32_t nbytes = uart->available();
        while (nbytes-- > 0) {
            uart->read();
            if (++index > LEDDARVU_SERIAL_PORT_MAX) {
                // LEDDARVU_STATE_ERR_SERIAL_PORT
                return false;
            }
        }
        // clear buffer and buffer_len
        memset(read_buffer, 0, sizeof(read_buffer));
        read_len = 0;
        modbus_status = LEDDARVU_MODBUS_STATE_PRE_SEND_REQUEST;
        }
        // no break to fall through to next state LEDDARVU_MODBUS_STATE_PRE_SEND_REQUEST immediately

    case LEDDARVU_MODBUS_STATE_PRE_SEND_REQUEST:
        // send a request message for Modbus function 4
        uart->write(send_request_buffer, sizeof(send_request_buffer));
        modbus_status = LEDDARVU_MODBUS_STATE_SENT_REQUEST;
        last_sending_request_ms = AP_HAL::millis();
        // no break

    case LEDDARVU_MODBUS_STATE_SENT_REQUEST:
        if (uart->available()) {
            // change mod_bus status to read available buffer
            modbus_status = LEDDARVU_MODBUS_STATE_AVAILABLE;
            last_available_ms = AP_HAL::millis();
        } else {
            if (AP_HAL::millis() - last_sending_request_ms > 200) {
                // reset mod_bus status to read new buffer
                // if read_len is zero, send request without initialize
                modbus_status = (read_len == 0) ? LEDDARVU_MODBUS_STATE_PRE_SEND_REQUEST : LEDDARVU_MODBUS_STATE_INIT;
            }
        }
        break;

    case LEDDARVU_MODBUS_STATE_AVAILABLE:
        // parse a response message, set number_detections, detections and sum_distance
        leddarvu_status = parse_response(number_detections);

        if (leddarvu_status == LEDDARVU_STATE_OK) {
            reading_cm = sum_distance / number_detections;

            // reset mod_bus status to read new buffer
            modbus_status = LEDDARVU_MODBUS_STATE_INIT;

            return true;
        }
        // if status is not reading buffer, reset mod_bus status to read new buffer
        else if (leddarvu_status != LEDDARVU_STATE_READING_BUFFER || AP_HAL::millis() - last_available_ms > 200) {
            // if read_len is zero, send request without initialize
            modbus_status = (read_len == 0) ? LEDDARVU_MODBUS_STATE_PRE_SEND_REQUEST : LEDDARVU_MODBUS_STATE_INIT;
        }
        break;
    }

    return false;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_LeddarVu::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

/*
   CRC16
   CRC-16-IBM(x16+x15+x2+1)
*/
bool AP_RangeFinder_LeddarVu::CRC16(uint8_t *aBuffer, uint8_t aLength, bool aCheck)
{
    uint16_t crc = 0xFFFF;

    for (uint32_t i=0; i<aLength; i++) {
        crc ^= aBuffer[i];
        for (uint32_t j=0; j<8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    uint8_t lCRCLo = LOWBYTE(crc);
    uint8_t lCRCHi = HIGHBYTE(crc);

    if (aCheck) {
        return (aBuffer[aLength] == lCRCLo) && (aBuffer[aLength+1] == lCRCHi);
    } else {
        aBuffer[aLength] = lCRCLo;
        aBuffer[aLength+1] = lCRCHi;
        return true;
    }
}

 /*
    parse a response message from Modbus
    -----------------------------------------------
    [ read buffer packet ]
    -----------------------------------------------
      0: slave address (LEDDARVU_DEFAULT_ADDRESS)
      1: functions code
      2: ?
3-4-5-6: timestamp
    7-8: internal temperature
      9: ?
     10: number of detections
  11-12: first distance
  13-14: first amplitude
  15-16: second distance
  17-18: second amplitude
  19-20: third distances
  21-22: third amplitude
     23: CRC Low
     24: CRC High
    -----------------------------------------------
  */
AP_RangeFinder_LeddarVu::LeddarVu_Status AP_RangeFinder_LeddarVu::parse_response(uint8_t &number_detections)
{
    uint8_t index;
    uint8_t index_offset = LEDDARVU_DETECTION_DATA_INDEX_OFFSET;

    // read serial
    uint32_t nbytes = uart->available();

    if (nbytes != 0)  {
        for (index=read_len; index<nbytes+read_len; index++) {
            if (index >= LEDDARVU_READ_BUFFER_SIZE) {
                return LEDDARVU_STATE_ERR_BAD_RESPONSE;
            }
            read_buffer[index] = uart->read();
        }

        read_len += nbytes;

        if (read_len < LEDDARVU_READ_BUFFER_SIZE) {
            return LEDDARVU_STATE_READING_BUFFER;
        }
    }

    // lead_len is not 25 byte or function code is not 0x04
    if (read_len != LEDDARVU_READ_BUFFER_SIZE || read_buffer[1] != LEDDARVU_MODOBUS_FUNCTION_CODE) {
        return LEDDARVU_STATE_ERR_BAD_RESPONSE;
    }

    // CRC16
    if (!CRC16(read_buffer, read_len-2, true)) {
        return LEDDARVU_STATE_ERR_BAD_CRC;
    }

    // number of detections (index:10)
    number_detections = read_buffer[LEDDARVU_DETECTION_DATA_NUMBER_INDEX];

    // if the number of detection is over or zero , it is false
    if (number_detections > LEDDARVU_DETECTIONS_MAX || number_detections == 0) {
        return LEDDARVU_STATE_ERR_NUMBER_DETECTIONS;
    }

    memset(detections, 0, sizeof(detections));
    sum_distance = 0;
    for (index=0; index<number_detections; index++) {
        // construct data word from two bytes and convert mm to cm
        detections[index] =  (static_cast<uint16_t>(read_buffer[index_offset])*256 + read_buffer[index_offset+1]) / 10;
        sum_distance += detections[index];

        // add index offset (4) to read next detection data
        index_offset += LEDDARVU_DETECTION_DATA_OFFSET;
    }

    return LEDDARVU_STATE_OK;
}
