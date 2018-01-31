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
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

class AP_Follow
{

public:

    // constructor
    AP_Follow(const AP_AHRS &ahrs);

    // initialise follow subsystem
    void init();

    // update follow subsystem
    void update();

    // true if follow subsystem is healthy (i.e. we are receiving messages)
    bool healthy() const { return _healthy; }

    // true if we have a valid target location estimate
    bool have_target() const;

    // get target's estimated location
    bool get_target_location(Location &target_loc) const;

    // get distance vector to target in meters in neu frame
    bool get_distance_to_target_ned(Vector3f &dist_to_target) const;

    // get target's heading in degrees (0 = north, 90 = east)
    bool get_target_heading(float &heading) const;

    // parse mavlink messages which may hold target's position, velocity and attitude
    void handle_msg(const mavlink_message_t &msg);

    // parameter list
    static const struct AP_Param::GroupInfo var_info[];

private:

    // get offsets in NED frame
    bool get_offsets_ned(Vector3f& offsets) const;

    // references
    const AP_AHRS &_ahrs;

    // parameters
    AP_Int8     _enabled;           // 1 if this subsystem is enabled
    AP_Int8     _type;              // follow type (keep initial offset, maintain margin)
    AP_Int16    _target_sysid;      // target's mavlink system id
    AP_Float    _margin;            // minimum target distance to target
    AP_Float    _dist_max;          // maximum distance to target.  targets further than this will be ignored
    AP_Int8     _offset_type;       // offset frame type (0:North-East-Down, 1:RelativeToLeadVehicleHeading)
    AP_Vector3f _offset;            // offset from lead vehicle in meters

    // local variables
    bool _healthy;                  // true if we are receiving mavlink messages (regardless of whether they have target position info within them)
    uint32_t _last_location_update_ms;  // system time of last position update
    Location _target_location;      // last known location of target
    Vector3f _target_velocity_ned;  // last known velocity of target in NED frame in m/s
    uint32_t _last_heading_update_ms;   // system time of last heading update
    float _target_heading;          // heading in degrees
};
