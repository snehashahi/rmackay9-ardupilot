#include "mode.h"
#include "Rover.h"

void ModeSimple::update()
{
    // get speed forward
    float speed;
    if (!attitude_control.get_forward_speed(speed)) {
        // no valid speed so stop
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    // convert pilot stick input into desired heading and throttle
    float desired_heading, desired_throttle;
    if (!get_pilot_desired_heading_and_throttle(desired_heading, desired_throttle)) {
        // stop vehicle on failure to decode pilot input
        stop_vehicle();
        return;
    }

    // convert pilot throttle input to desired speed
    float target_speed = desired_throttle * 0.01f * calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);

    // calculate angle error (in radians)
    const float angle_err_rad = wrap_PI(radians(desired_heading) - ahrs.yaw);

    // we never reverse
    rover.set_reverse(false);

    // apply object avoidance to desired speed using half vehicle's maximum acceleration/deceleration
    rover.g2.avoid.adjust_speed(0.0f, 0.5f * attitude_control.get_accel_max(), ahrs.yaw, target_speed, rover.G_Dt);

    // run heading to turn rate controller and throttle controller
    const float steering_out = attitude_control.get_steering_out_angle_error(angle_err_rad, g2.motors.have_skid_steering(), g2.motors.limit.steer_left, g2.motors.limit.steer_right, false);
    g2.motors.set_steering(steering_out * 4500.0f);
    calc_throttle(target_speed, false);
}
