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
#include <GCS_MAVLink/GCS.h>

class AP_Follow
{

public:

    // constructor
    AP_Follow();

    // initialise follow subsystem
    void init(GCS &gcs);

    // update follow subsystem
    void update();

    // true if follow subsystem is healthy (i.e. we are receiving messages)
    bool healthy() const { return _healthy; }

    // true if we have a valid target location estimate
    bool have_target() const { return _have_location; }

    // get target's estimated location
    bool get_target_location(Location &target_loc) const;

    // get target's heading in degrees (0 = north, 90 = east)
    bool get_target_heading(float &heading) const;

    // parse mavlink messages which may hold target's position, velocity and attitude
    void handle_msg(const mavlink_message_t &msg);

    // parameter list
    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Int8     _enabled;            // 1 if this subsystem is enabled
    AP_Int8     _type;              // follow type (keep initial offset, maintain margin)
    AP_Int16    _target_sysid;      // target's mavlink system id
    AP_Float    _margin;            // minimum target distance to target

    // local variables
    bool _healthy;                  // true if we are receiving mavlink messages (regardless of whether they have target position info within them)
    bool _have_location;            // true if we have the target's location
    bool _have_velocity;            // true if we have the target's velocity
    bool _have_heading;             // true if we have the target's heading
    uint32_t _last_update_ms;       // system time of last position update
    Location _target_location;      // last known location of target
    Vector3f _target_velocity_ned;  // last known velocity of target in NED frame in m/s
    float _target_heading;          // heading in degrees
};
