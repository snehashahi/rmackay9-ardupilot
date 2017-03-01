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

// return latest attitude delta
void AP_ZED::get_deltas(Vector3f &angle_delta, Vector3f &position_delta, uint64_t &time_delta_usec, float &confidence) const
{
    angle_delta = zed_state.angle_delta;
    position_delta = zed_state.position_delta;
    time_delta_usec = zed_state.time_delta_usec;
    confidence = zed_state.confidence;
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

