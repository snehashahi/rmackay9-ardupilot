#pragma once

#include "AP_ZED_Backend.h"

class AP_ZED_MAV : public AP_ZED_Backend
{

public:
    // constructor
    AP_ZED_MAV(AP_ZED &frontend);

    // consume ZED data from MAVLink messages
    void handle_msg(mavlink_message_t *msg);
};
