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

    // @Param: _DIST_MAX
    // @DisplayName: Follow distance maximum
    // @Description: Follow distance maximum.  targets further than this will be ignored
    // @Units: m
    // @Range: 1 1000
    // @User: Standard
    AP_GROUPINFO("_DIST_MAX", 5, AP_Follow, _dist_max, 100),

    AP_GROUPEND
};

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Follow::AP_Follow(const AP_AHRS &ahrs) :
        _ahrs(ahrs)
{
}

// initialise follow subsystem
void AP_Follow::init()
{
    // nothing to do
}

// update the state of the sensor
void AP_Follow::update(void)
{
    // exit immediately if not healthy
    if (!_enabled) {
        _healthy = false;
    }

    // check for timeout and set health status
    if (_last_location_update_ms == 0 || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        _healthy = false;
    } else {
        _healthy = true;
    }
}

// true if we have a valid target location estimate
bool AP_Follow::have_target() const
{
    // exit immediately if not healthy
    if (!_enabled || !_healthy) {
        return false;
    }

    // check for timeout
    if ((_last_location_update_ms == 0) || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    // we must have a valid target estimate
    return true;
}

// get target's estimated location
bool AP_Follow::get_target_location(Location &target_loc) const
{
    // exit immediately if not healthy
    if (!_enabled || !_healthy) {
        return false;
    }

    // check for timeout
    if ((_last_location_update_ms == 0) || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    // calculate time since last actual position update
    float dt = (AP_HAL::millis() - _last_location_update_ms) / 1000.0f;

    // project the vehicle position
    Location loc = _target_location;
    location_offset(loc, _target_velocity_ned.x * dt, _target_velocity_ned.y * dt);
    loc.alt -= _target_velocity_ned.z * 10.0f * dt; // convert m/s to cm/s, multiply by dt.  minus because NED

    // return latest position estimate
    target_loc = loc;
    return true;
}

// get distance vector to target in meters in neu frame
bool AP_Follow::get_distance_to_target_ned(Vector3f &dist_to_target) const
{
    // get our location
    Location current_loc;
    if (!_ahrs.get_position(current_loc)) {
        return false;
    }

    // get target location
    Location target_loc;
    if (!get_target_location(target_loc)) {
        return false;
    }

    // calculate difference
    dist_to_target = location_3d_diff_NED(current_loc, target_loc);
    return true;
}

// get target's heading in degrees (0 = north, 90 = east)
bool AP_Follow::get_target_heading(float &heading) const
{
    // exit immediately if not enabled, healthy or don't have the heading
    if (!_enabled || !_healthy) {
        return false;
    }

    // check for timeout
    if ((_last_heading_update_ms == 0) || (AP_HAL::millis() - _last_heading_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    // return latest heading estimate
    heading = _target_heading;
    return true;
}

// handle mavlink DISTANCE_SENSOR messages
void AP_Follow::handle_msg(const mavlink_message_t &msg)
{
    // exit immediately if not enabled
    if (!_enabled) {
        return;
    }

    // skip message if not from our target
    if (msg.sysid != _target_sysid) {
        return;
    }

    // decode global-position-int message
    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        uint32_t now = AP_HAL::millis();
        // decode message
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);
        _target_location.lat = packet.lat;
        _target_location.lng = packet.lon;
        _target_location.alt = packet.alt/10;       // convert millimeters to cm
        _target_velocity_ned.x = packet.vx/100.0f;  // velocity north
        _target_velocity_ned.y = packet.vy/100.0f;  // velocity east
        _target_velocity_ned.z = packet.vz/100.0f;  // velocity down
        _last_location_update_ms = now;
        if (packet.hdg <= 36000) {                  // heading (UINT16_MAX if unknown)
            _target_heading = packet.hdg / 100.0f;  // convert centi-degrees to degrees
            _last_heading_update_ms = now;
        }
    }
}
