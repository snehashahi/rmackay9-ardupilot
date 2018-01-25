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
#include <GCS_MAVLink/GCS.h>
#include "AP_Follow.h"
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define AP_FOLLOW_TIMEOUT_MS    1000    // position estimate timeout after 1 second

AP_Follow* AP_Follow::_static_follow_ptr = nullptr;

// table of user settable parameters
const AP_Param::GroupInfo AP_Follow::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Follow enable/disable
    // @Description: Enabled/disable following a target
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_Follow, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _TYPE
    // @DisplayName: Follow type
    // @Description: Follow type
    // @Values: 0:Keep Initial Offset, 1:Maintain Margin
    // @User: Standard
    AP_GROUPINFO("_TYPE", 2, AP_Follow, _type, 0),

    // @Param: _SYSID
    // @DisplayName: Follow target's mavlink system id
    // @Description: Follow target's mavlink system id
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_SYSID", 3, AP_Follow, _target_sysid, 2),

    // @Param: _MARGIN
    // @DisplayName: Follow minimum distance margin
    // @Description: Follow minimum distance margin
    // @Units: m
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("_MARGIN", 4, AP_Follow, _margin, 5),

    AP_GROUPEND
};

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Follow::AP_Follow()
{
}

// initialise follow subsystem
void AP_Follow::init(GCS &gcs)
{
    // store pointer to this object
    _static_follow_ptr = this;

    for (uint8_t i = 1; i < gcs.num_gcs(); i++) {
        gcs.chan(i).set_snoop(mavlink_snoop);
    }
}

// update the state of the sensor
void AP_Follow::update(void)
{
    // exit immediately if not healthy
    if (!_enabled) {
        _healthy = false;
    }

    // check for timeout and set health status
    if (_last_update_ms == 0 || (AP_HAL::millis() - _last_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        _healthy = false;
    } else {
        _healthy = true;
    }
}

// get target's estimated location
bool AP_Follow::get_target_location(Location &target_loc) const
{
    // exit immediately if not healthy
    if (!_enabled || !_healthy || !_have_location) {
        return false;
    }

    /*
    // calculate time since last actual position update
    float dt = (AP_HAL::micros() - vehicle.last_update_us) * 1.0e-6f;

    // if less than 5 seconds since last position update estimate the position
    if (dt < TRACKING_TIMEOUT_SEC) {
        // project the vehicle position to take account of lost radio packets
        vehicle.location_estimate = vehicle.location;
        float north_offset = vehicle.vel.x * dt;
        float east_offset = vehicle.vel.y * dt;
        location_offset(vehicle.location_estimate, north_offset, east_offset);
        vehicle.location_estimate.alt += vehicle.vel.z * 100.0f * dt;
        // set valid_location flag
        vehicle.location_valid = true;
    } else {
        // vehicle has been lost, set lost flag
        vehicle.location_valid = false;
    }*/

    // return latest position estimate
    return false;
}

// get target's heading in degrees (0 = north, 90 = east)
bool AP_Follow::get_target_heading(float &heading) const
{
    // exit immediately if not enabled, healthy or don't have the heading
    if (!_enabled || !_healthy || !_have_heading) {
        return false;
    }

    // return latest heading estimate
    return false;
}

// handle mavlink DISTANCE_SENSOR messages
void AP_Follow::handle_msg(const mavlink_message_t *msg)
{
    // exit immediately if not enabled
    if (!_enabled) {
        return;
    }

    // skip message if not from our target
    if (msg->sysid != _target_sysid) {
        return;
    }

    // decode global-position-int message
    if (msg->msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        // decode message
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(msg, &packet);
        _target_location.lat = packet.lat;
        _target_location.lng = packet.lon;
        _target_location.alt = packet.alt/10;       // convert millimeters to cm
        _have_location = true;
        _target_velocity_ned.x = packet.vx/100.0f;  // velocity north
        _target_velocity_ned.y = packet.vy/100.0f;  // velocity east
        _target_velocity_ned.z = packet.vz/100.0f;  // velocity down
        _have_velocity = true;
        if (packet.hdg <= 36000) {
            _target_heading = packet.hdg / 100.0f;      // heading (UINT16_MAX if unknown)
            _have_heading = true;
        }
        _last_update_ms = AP_HAL::millis();
    }
}

// mavlink snoop to listen for messages from other vehicles
void AP_Follow::mavlink_snoop(const mavlink_message_t* msg)
{
    // pass message to the last follow object created
    if (_static_follow_ptr != nullptr) {
        _static_follow_ptr->handle_msg(msg);
    }
}
