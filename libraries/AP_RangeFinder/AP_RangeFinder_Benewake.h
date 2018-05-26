#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_Benewake : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_Benewake(RangeFinder::RangeFinder_State &_state,
                            AP_SerialManager &serial_manager,
                            uint8_t serial_instance);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);

    // update state
    void update(void);

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // get a reading
    // distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
    bool get_reading(uint16_t &reading_cm, bool &signal_ok);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;
    char linebuf[10];
    uint8_t linebuf_len = 0;
};
