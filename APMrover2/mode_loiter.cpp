#include "mode.h"
#include "Rover.h"

bool ModeLoiter::_enter()
{
    // only available for boats
    //if (!rover.is_boat()) {
    //    return false;
    //}

    // set _destination to reasonable stopping point
    calc_stopping_location(_destination);

    // initialise desired speed to current speed
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;
    _yaw_error_cd = 0.0f;

    // set reversed based on speed
    //rover.set_reverse(is_negative(_desired_speed));

    ::printf("dspd:%4.2f desy:%4.2f\n",
        (double)_desired_speed,
        (double)_desired_yaw_cd
        );
    return true;
}

void ModeLoiter::update()
{
    // get current speed
    float speed;
    if (!attitude_control.get_forward_speed(speed)) {
        speed = 0.0f;
    }

    // get distance (in meters) to destination
    _distance_to_destination = get_distance(rover.current_loc, _destination);

    // if within waypoint radius slew desired speed towards zero and use existing desired heading
    if (_distance_to_destination <= g.waypoint_radius) {
        if (is_negative(_desired_speed)) {
            _desired_speed = MIN(_desired_speed + attitude_control.get_accel_max(), 0.0f);
        } else {
            _desired_speed = MAX(_desired_speed - attitude_control.get_accel_max(), 0.0f);
        }
        // let heading drift
        _desired_yaw_cd = ahrs.yaw_sensor;
        _yaw_error_cd = 0.0f;
    } else {
        // P controller with hard-coded gain to convert distance to desired speed
        // To-Do: add constraint to ensure _desired_speed is not higher than vehicle can slow in _distance_to_destination
        _desired_speed = MIN((_distance_to_destination - g.waypoint_radius) * 0.5f, g.speed_cruise);

        bool fwd = true;
        // calculate bearing to destination
        _desired_yaw_cd = get_bearing_cd(rover.current_loc, _destination);
        _yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
        // if destination is behind vehicle, reverse towards it
        if (fabsf(_yaw_error_cd) > 9000) {
            _desired_yaw_cd = wrap_180_cd(_desired_yaw_cd + 18000);
            _yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
           // _yaw_error_cd = wrap_180_cd(_yaw_error_cd + 18000);
            _desired_speed = -_desired_speed;
            fwd = false;
            ::printf("rev fwd:%d ye:%4.2f\n",
                (int)fwd,
                (double)_yaw_error_cd);
        }

        // reduce desired speed if yaw_error is large
        float yaw_error_ratio = 1.0f - constrain_float(fabsf(_yaw_error_cd / 9000.0f), 0.0f, 1.0f) * ((100.0f - g.speed_turn_gain) * 0.01f);
        _desired_speed *= yaw_error_ratio;

        // debug
        static uint8_t counter = 0;
        counter++;
        if (counter > 10 || !fwd) {
            counter = 0;
            if (!is_zero(_desired_speed) || !fwd) {
                ::printf("fwd:%d dist:%4.2f ye:%4.2f dy:%4.2f dspd:%6.4f spd:%6.4f yer:%6.4f\n",
                    (int)fwd,
                    (double)_distance_to_destination,
                    (double)_yaw_error_cd,
                    (double)_desired_yaw_cd,
                    (double)_desired_speed,
                    (double)speed,
                    (double)yaw_error_ratio
                    );
            }
        }
    }

    // set reversed flag
    bool reversed = is_negative(g2.motors.get_throttle());

    // run steering and throttle controllers
    //calc_steering_to_heading(_desired_yaw_cd, _desired_speed < 0);
    calc_steering_to_heading(_desired_yaw_cd, reversed);
    //calc_steering_to_heading(0, reversed);
    calc_throttle(_desired_speed, false, true);

    // mark us as in_reverse when using a negative throttle
    // To-Do: only in reverse if vehicle is actually travelling backwards?
    //rover.set_reverse(_desired_speed < 0);
    rover.set_reverse(reversed);
}
