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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_ZED_Backend;

#define AP_ZED_MAX_SENSORS 1
#define AP_ZED_TIMEOUT_MS 300

class AP_ZED
{
public:
    friend class AP_ZED_Backend;

    AP_ZED();

    // external position backend types (used by _TYPE parameter)
    enum AP_ZEDType {
        AP_ZEDType_None   = 0,
        AP_ZEDType_MAV    = 1
    };

    // The AP_ZEDState structure is filled in by the backend driver
    struct ZEDState {
        Vector3f angle_delta;       // attitude delta (in radians) of most recent update
        Vector3f position_delta;    // position delta (in meters) of most recent update
        uint64_t time_delta_usec;   // time delta (in usec) between previous and most recent update
        float confidence;           // confidence expressed as a value from 0 (no confidence) to 100 (very confident)
        uint32_t last_update_ms;    // system time (in milliseconds) of last update from sensor
    };

    // return true if ZED is enabled
    bool enabled() const;

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() const;

    // return latest attitude delta
    void get_deltas(Vector3f &angle_delta, Vector3f &position_delta, uint64_t &time_delta_usec, float &confidence) const;

    // return system time of last update
    uint32_t last_update_ms() const { return zed_state.last_update_ms; }

    // consume ZED data from MAVLink messages
    void handle_msg(mavlink_message_t *msg);

    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Int8 _type;

    // reference to backends
    AP_ZED_Backend *_driver;

    // state of backend
    ZEDState zed_state;
};
