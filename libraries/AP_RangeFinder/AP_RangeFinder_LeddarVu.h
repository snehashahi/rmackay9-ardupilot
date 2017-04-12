#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

// defines
#define LEDDARVU_DEFAULT_ADDRESS 0x01
#define LEDDARVU_MODOBUS_FUNCTION_CODE 0x04
#define LEDDARVU_MODOBUS_FUNCTION_REGISTER_ADDRESS 20
#define LEDDARVU_MODOBUS_FUNCTION_READ_NUMBER 10

#define LEDDARVU_SERIAL_PORT_MAX 250
#define LEDDARVU_READ_BUFFER_SIZE 25

#define LEDDARVU_DETECTIONS_MAX 3
#define LEDDARVU_DETECTION_DATA_NUMBER_INDEX 10
#define LEDDARVU_DETECTION_DATA_INDEX_OFFSET 11
#define LEDDARVU_DETECTION_DATA_OFFSET 4

class AP_RangeFinder_LeddarVu : public AP_RangeFinder_Backend
{

public:

    // status
    enum LeddarVu_Status {
        LEDDARVU_STATE_OK = 0,
        LEDDARVU_STATE_READING_BUFFER = 1,
        LEDDARVU_STATE_ERR_BAD_CRC = -1,
        LEDDARVU_STATE_ERR_NO_RESPONSES = -2,
        LEDDARVU_STATE_ERR_BAD_RESPONSE = -3,
        LEDDARVU_STATE_ERR_SHORT_RESPONSE = -4,
        LEDDARVU_STATE_ERR_SERIAL_PORT = -5,
        LEDDARVU_STATE_ERR_NUMBER_DETECTIONS = -6
    };

    // LeddarVu Modbus status
    enum LeddarVu_ModbusStatus {
        LEDDARVU_MODBUS_STATE_INIT = 0,
        LEDDARVU_MODBUS_STATE_PRE_SEND_REQUEST,
        LEDDARVU_MODBUS_STATE_SENT_REQUEST,
        LEDDARVU_MODBUS_STATE_AVAILABLE
    };

    // constructor
    AP_RangeFinder_LeddarVu(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance, AP_SerialManager &serial_manager);

    // update state
    void update(void);

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);

    // CRC16
    bool CRC16(uint8_t *aBuffer, uint8_t aLength, bool aCheck);

    // parse a response message from ModBus
    LeddarVu_Status parse_response(uint8_t &number_detections);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms;
    uint32_t last_sending_request_ms;
    uint32_t last_available_ms;

    uint16_t detections[LEDDARVU_DETECTIONS_MAX];
    uint32_t sum_distance;

    LeddarVu_ModbusStatus modbus_status = LEDDARVU_MODBUS_STATE_INIT;
    uint8_t read_buffer[LEDDARVU_READ_BUFFER_SIZE];
    uint32_t read_len;

    // Modbus send request buffer
    // read input register (function code 0x04)
    const uint8_t send_request_buffer[8] = {
        LEDDARVU_DEFAULT_ADDRESS,
        LEDDARVU_MODOBUS_FUNCTION_CODE,
        0,
        LEDDARVU_MODOBUS_FUNCTION_REGISTER_ADDRESS,   // 20: Address of first register to read
        0,
        LEDDARVU_MODOBUS_FUNCTION_READ_NUMBER,        // 10: The number of consecutive registers to read
        0x30,   // CRC Lo
        0x09    // CRC Hi
    };
};
