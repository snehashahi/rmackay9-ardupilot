#include "Rover.h"

/*
To Do List
 - Improve tacking in light winds and bearing away in strong wings
 - consider drag vs lift sailing differences, ie upwind sail is like wing, dead down wind sail is like parachute
 - max speed paramiter and contoller, for maping you may not want to go too fast
 - mavlink sailing messages
 - motor sailing, some boats may also have motor, we need to decide at what point we would be better of just motoring in low wind, or for a tight loiter, or to hit waypoint exactly, or if stuck head to wind, or to reverse...
 - smart decision making, ie tack on windshifts, what to do if stuck head to wind
 - some sailing codes track waves to try and 'surf' and to allow tacking on a flat bit, not sure if there is much gain to be had here
 - add some sort of pitch monitoring to prevent nose diving in heavy weather
 - pitch PID for hydrofoils
 - more advanced sail control, ie twist
 - independent sheeting for main and jib
 - wing type sails with 'elevator' control
 - tack on depth sounder info to stop sailing into shallow water on indirect sailing routes
 - add option to do proper tacks, ie tacking on flat spot in the waves, or only try once at a certain speed, or some better method than just changing the desired heading suddenly
*/

// update mainsail's desired angle based on wind speed and direction and desired speed (in m/s)
void Rover::sailboat_update_mainsail(float desired_speed)
{
    if (!g2.motors.has_sail()) {
        return;
    }

    // relax sail if desired speed is zero
    if (!is_positive(desired_speed)) {
        g2.motors.set_mainsail(100.0f);
        return;
    }

    // + is wind over starboard side, - is wind over port side, but as the sails are sheeted the same on each side it makes no difference so take abs
    float wind_dir_apparent = fabsf(g2.windvane.get_apparent_wind_direction_rad());
    wind_dir_apparent = degrees(wind_dir_apparent);

    // set the main sail to the ideal angle to the wind
    float mainsail_angle = wind_dir_apparent - g2.sail_angle_ideal;

    // make sure between allowable range
    mainsail_angle = constrain_float(mainsail_angle, g2.sail_angle_min, g2.sail_angle_max);

    // linear interpolate mainsail value (0 to 100) from wind angle mainsail_angle
    float mainsail = linear_interpolate(0.0f, 100.0f, mainsail_angle, g2.sail_angle_min, g2.sail_angle_max);

    // use PID controller to sheet out
    const float pid_offset =  g2.attitude_control.get_sail_out_from_heel(radians(g2.sail_heel_angle_max), G_Dt) * 100.0f;

    mainsail = constrain_float((mainsail+pid_offset), 0.0f ,100.0f);
    g2.motors.set_mainsail(mainsail);
}

// returns true if sailboat should take a indirect navigation route, either to go upwind or in the future for speed
// desired_heading should be in centi-degrees
bool Rover::sailboat_use_indirect_route(float desired_heading_cd)
{
    if (!g2.motors.has_sail()) {
        return false;
    }

    const float desired_heading_rad = radians(desired_heading_cd * 0.01f);

    // check if desired heading is in the no go zone, if it is we can't go direct
    // add 10 deg padding to try and avoid constant switching between methods, maybe add a 'dead zone'?
    if (fabsf(wrap_PI((g2.windvane.get_absolute_wind_direction_rad() - desired_heading_rad))) <= radians(g2.sail_no_go + 10.0f)) {
        _sailboat_indirect_route = true;
    } else {
        _sailboat_indirect_route = false;
    }

    return _sailboat_indirect_route;
}

// If we can't sail on the desired heading then we should pick the best heading that we can sail on
float Rover::sailboat_calc_heading(float desired_heading_cd)
{
    if (!g2.motors.has_sail()) {
        return desired_heading_cd;
    }

    /*
        Until we get more fancy logic for best possible speed just assume we can sail upwind at the no go angle
        Just set off on which ever of the no go angles is on the current tack, once the end destination is within a single tack it will switch back to direct route method
        This should result in a long leg with a single tack to get to the destination.
        Tack can be triggered by geo fence, aux switch, rudder input and max cross track error

        Need to add some logic to stop it from tacking back towards fence once it has been bounced off, possibly a minimum distance and time between tacks or something
    */

    // left and right no go headings looking upwind
    const float left_no_go_heading_rad = wrap_2PI(g2.windvane.get_absolute_wind_direction_rad() + radians(g2.sail_no_go));
    const float right_no_go_heading_rad = wrap_2PI(g2.windvane.get_absolute_wind_direction_rad() - radians(g2.sail_no_go));

    // calculate what tack we are on if it has been too long since we knew
    if (_sailboat_current_tack == Tack_Unknown || (AP_HAL::millis() - _sailboat_last_calc_heading_ms) > 1000) {
        if (is_negative(g2.windvane.get_apparent_wind_direction_rad())) {
            _sailboat_current_tack = Tack_Port;
        } else {
            _sailboat_current_tack = Tack_STBD;
        }
    }
    _sailboat_last_calc_heading_ms = AP_HAL::millis();

    // maximum cross track error before tack, this effectively defines a 'corridor' of width 2*waypoint_overshoot that the boat will stay within, disable if tacking or in hold mode
    if ((fabsf(rover.nav_controller->crosstrack_error()) >= g.waypoint_overshoot) && !is_zero(g.waypoint_overshoot) && !sailboat_tacking()) {
        // make sure the new tack will reduce the cross track error
        // if were on starboard tack we are traveling towards the left hand boundary
        if (is_positive(rover.nav_controller->crosstrack_error()) && (_sailboat_current_tack == Tack_STBD)) {
            _sailboat_tack = true;
        }
        // if were on port tack we are traveling towards the right hand boundary
        if (is_negative(rover.nav_controller->crosstrack_error()) && (_sailboat_current_tack == Tack_Port)) {
            _sailboat_tack = true;
        }
    }

    // are we due to tack?
    // TODO: double check that we did tack if we meant to
    // TODO: do_tack routine that takes the current and target heading and gives target rates depending on conditions, ie in light wind tack slowly to conserve momentum
    if (_sailboat_tack) {
        gcs().send_text(MAV_SEVERITY_INFO, "Sailboat - Tacking");

        // Pick a heading for the new tack
        switch (_sailboat_current_tack) {
            case Tack_Unknown:
                // should never happen
                break;
            case Tack_Port:
                _sailboat_new_tack_heading_cd = degrees(right_no_go_heading_rad) * 100.0f;
                _sailboat_current_tack = Tack_STBD;
                break;
            case Tack_STBD:
                _sailboat_new_tack_heading_cd = degrees(left_no_go_heading_rad) * 100.0f;
                _sailboat_current_tack = Tack_Port;
                break;
        }

        _sailboat_tack = false;
        _sailboat_tacking = true;
        _sailboat_tack_start_ms = AP_HAL::millis();
    }

    // if were in the process of a tack we should not change the target heading, not sure if this is a good idea or not, the target shouldn't change too much while tacking, except if the vane provides poor readings as we are tacking
    if (_sailboat_tacking) {
        // Check if we have tacked round enough or if we have timed out
        // not sure if the time out is necessary
        if (AP_HAL::millis() - _sailboat_tack_start_ms > 50000.0f || fabsf(wrap_180_cd(_sailboat_new_tack_heading_cd - ahrs.yaw_sensor)) < (10.0f * 100.0f)){
            _sailboat_tacking = false;
            // If we timed out and did not reached the desired heading so we cannot be sure what tack we are on
            if (AP_HAL::millis() - _sailboat_tack_start_ms > 50000.0f) {
                _sailboat_current_tack = Tack_Unknown;
            }
        }
        desired_heading_cd = _sailboat_new_tack_heading_cd;
    } else {
        // set new heading
        switch (_sailboat_current_tack) {
            case Tack_Unknown:
                // should never happen
                break;
            case Tack_Port:
                desired_heading_cd = degrees(left_no_go_heading_rad) * 100.0f;
                break;
            case Tack_STBD:
                desired_heading_cd = degrees(right_no_go_heading_rad) * 100.0f;
                break;
        }
    }

    return desired_heading_cd;
}

// returns true if boat is currently tacking
bool Rover::sailboat_tacking() const
{
    return (_sailboat_tack || _sailboat_tacking);
}

// user initiated tack
void Rover::sailboat_trigger_tack()
{
    if (!control_mode->allows_tacking_from_transmitter()) {
        return;
    }

    if (!sailboat_tacking() && rover._sailboat_indirect_route) {
        _sailboat_tack = true;
    }
}

// check if user triggers tack by holding steering to > 90%
void Rover::sailboat_check_steering_triggered_tack()
{
    // exit immediately if not sailboat
    if (!g2.motors.has_sail()) {
        return;
    }

    // rudder threshold to trigger tack in auto heading modes
    const float steer_threshold = 0.9f;

    // allow force tack from rudder input
    const float steering_in = rover.channel_steer->norm_input();

    if ((fabsf(steering_in) > steer_threshold) && !sailboat_tacking()) {
        switch (_sailboat_current_tack) {
            case Tack_Unknown:
                // sailboat_calc_heading has not been called so ignore
                break;
            case Tack_Port:
                if (steering_in < -steer_threshold) { // if we are on port a left hand steering input would be a tack
                    _sailboat_tack = true;
                }
                break;
            case Tack_STBD:
                if (steering_in > steer_threshold) { // if on stbd right hand turn is a tack
                    _sailboat_tack = true;
                }
                break;
        }
    }
}

// return heading in radians when tacking in acro
float Rover::sailboat_acro_tack_heading_rad()
{
    // initiate tack
    if (_sailboat_tack) {
        // match the current angle to the true wind on the new tack
        _sailboat_acro_tack_heading_rad = wrap_2PI(ahrs.yaw + 2.0f * wrap_PI((g2.windvane.get_absolute_wind_direction_rad() - ahrs.yaw)));
        _sailboat_tack = false;
        _sailboat_tacking = true;
        _sailboat_tack_start_ms = AP_HAL::millis();
    }

    // wait until tack is completed
    // check if we have tacked round enough or if we have timed out
    // time out needed for acro as the pilot is not in control while tacking
    if (_sailboat_tacking ) {
        if (((AP_HAL::millis() - _sailboat_tack_start_ms) > 5000.0f) || fabsf(wrap_PI(_sailboat_acro_tack_heading_rad - ahrs.yaw)) < radians(5.0f)){
            _sailboat_tacking = false;
        }
    }

    return _sailboat_acro_tack_heading_rad;
}

// return sailboat's maximum turn rate in deg/sec
float Rover::sailboat_get_rate_max(float rate_max_degs) const
{
    if (!g2.motors.has_sail()) {
        return rate_max_degs;
    }

    // if were traveling in a 'straight line' on a single tack reduce the maximum allowed rate to smooth out heading response to wind changes, use normal max rate for tacking
    if (!_sailboat_tack && !_sailboat_tacking && is_positive(g2.sail_straight_rate_max)) {
        return g2.sail_straight_rate_max;
    }

    return rate_max_degs;
}

// Velocity Made Good, this is the speed we are traveling towards the desired destination
// only for logging at this stage
// https://en.wikipedia.org/wiki/Velocity_made_good
float Rover::sailboat_get_VMG() const
{
    // return 0 if not heading towards waypoint
    if (!control_mode->is_autopilot_mode()) {
        return 0.0f;
    }

    float speed;
    if (!g2.attitude_control.get_forward_speed(speed)) {
        return 0.0f;
    }
    return (speed * cosf(wrap_PI(radians(nav_controller->target_bearing_cd()) - ahrs.yaw)));
}
