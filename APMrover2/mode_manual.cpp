#include "mode.h"
#include "Rover.h"

void ModeManual::update()
{
    float desired_steering, desired_throttle;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);

    // update speed max estimate (used for motor rate control)
    // ToDo: this should be called intermittantly in case cruise-throttle or cruise-speed has been updated
    send_speed_max_estimate_to_motors();

    // copy RC scaled inputs to outputs
    g2.motors.set_throttle(desired_throttle);
    g2.motors.set_steering(desired_steering);

    // mark us as in_reverse when using a negative throttle to stop AHRS getting off
    rover.set_reverse(is_negative(g2.motors.get_throttle()));
}
