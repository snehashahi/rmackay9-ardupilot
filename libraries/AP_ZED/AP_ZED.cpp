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

#include "AP_ZED.h"
#include "AP_ZED_Backend.h"
#include "AP_ZED_MAV.h"

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_ZED::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: ZED camera connection type
    // @Description: ZED camera connection type
    // @Values: 0:None,1:MAV
    // @User: Advanced
    AP_GROUPINFO("_TYPE",    0, AP_ZED, _type, 0),

    // @Param: _POS_X
    // @DisplayName: ZED camera X position offset
    // @Description: X position of the camera in body frame. Positive X is forward of the origin.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Y
    // @DisplayName: ZED camera Y position offset
    // @Description: Y position of the camera in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Z
    // @DisplayName: ZED camera Z position offset
    // @Description: Z position of the camera in body frame. Positive Z is down from the origin.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_POS", 1, AP_ZED, _pos_offset, 0.0f),

    AP_GROUPEND
};

AP_ZED::AP_ZED()
{
    AP_Param::setup_object_defaults(this, var_info);

    // create backend
    if (_type == AP_ZEDType_MAV) {
        _driver = new AP_ZED_MAV(*this);
    }
}

// return true if ZED is enabled
bool AP_ZED::enabled() const
{
    return ((_type != AP_ZEDType_None) && (_driver != nullptr));
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_ZED::healthy() const
{
    if (!enabled()) {
        return false;
    }

    // healthy if we have received sensor messages within the past 300ms
    return ((AP_HAL::millis() - zed_state.last_update_ms) < AP_ZED_TIMEOUT_MS);
}

// consume ZED data from MAVLink messages
void AP_ZED::handle_msg(mavlink_message_t *msg)
{
    // exit immediatley if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        _driver->handle_msg(msg);
    }
}

