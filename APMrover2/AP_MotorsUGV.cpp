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
#include "SRV_Channel/SRV_Channel.h"
#include "AP_MotorsUGV.h"
#include "Rover.h"

extern const AP_HAL::HAL& hal;

// parameters for the motor class
const AP_Param::GroupInfo AP_MotorsUGV::var_info[] = {
    // @Param: PWM_TYPE
    // @DisplayName: Motor Output PWM type
    // @Description: This selects the output PWM type as regular PWM, OneShot, Brushed motor support using PWM (duty cycle) with separated direction signal, Brushed motor support with separate throttle and direction PWM (duty cyle)
    // @Values: 0:Normal,1:OneShot,2:OneShot125,3:BrushedWithRelay,4:BrushedBiPolar
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PWM_TYPE", 1, AP_MotorsUGV, _pwm_type, PWM_TYPE_NORMAL),

    // @Param: PWM_FREQ
    // @DisplayName: Motor Output PWM freq for brushed motors
    // @Description: Motor Output PWM freq for brushed motors
    // @Units: kHz
    // @Range: 1 20
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PWM_FREQ", 2, AP_MotorsUGV, _pwm_freq, 16),

    // @Param: SAFE_DISARM
    // @DisplayName: Motor PWM output disabled when disarmed
    // @Description: Disables motor PWM output when disarmed
    // @Values: 0:PWM enabled while disarmed, 1:PWM disabled while disarmed
    // @User: Advanced
    AP_GROUPINFO("SAFE_DISARM", 3, AP_MotorsUGV, _disarm_disable_pwm, 0),

    // @Param: THR_MIN
    // @DisplayName: Throttle minimum
    // @Description: Throttle minimum percentage the autopilot will apply. This is mostly useful for rovers with internal combustion motors, to prevent the motor from cutting out in auto mode.
    // @Units: %
    // @Range: 0 20
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MIN", 5, AP_MotorsUGV, _throttle_min, 0),

    // @Param: THR_MAX
    // @DisplayName: Throttle maximum
    // @Description: Throttle maximum percentage the autopilot will apply. This can be used to prevent overheating an ESC or motor on an electric rover
    // @Units: %
    // @Range: 30 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MAX", 6, AP_MotorsUGV, _throttle_max, 100),

    // @Param: SLEWRATE
    // @DisplayName: Throttle slew rate
    // @Description: Throttle slew rate as a percentage of total range per second. A value of 100 allows the motor to change over its full range in one second.  A value of zero disables the limit.  Note some NiMH powered rovers require a lower setting of 40 to reduce current demand to avoid brownouts.
    // @Units: %/s
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SLEWRATE", 8, AP_MotorsUGV, _slew_rate, 100),

    // @Param: THST_EXPO
    // @DisplayName: Thrust Curve Expo
    // @Description: Thrust curve exponent (-1 to +1 with 0 being linear)
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("THST_EXPO", 9, AP_MotorsUGV, _thrust_curve_expo, 0.0f),

    // @Param: _WL_P
    // @DisplayName: Wheel rate controller (Left) P gain
    // @Description: Wheel rate controller (Left) P gain.  Converts wheel rate error (as a percentage of full range) to wheel output (in the range -1 to +1)
    // @Range: 0.100 2.000
    // @User: Standard

    // @Param: _WL_I
    // @DisplayName: Wheel rate controller (Left) I gain
    // @Description: Wheel rate controller (Left) I gain.  Corrects long term error between the desired rate (in rad/s) and actual
    // @Range: 0.000 2.000
    // @User: Standard

    // @Param: _WL_IMAX
    // @DisplayName: Wheel rate controller (Left) I gain maximum
    // @Description: Wheel rate controller (Left) I gain maximum.  Constrains the output (range -1 to +1) that the I term will generate
    // @Range: 0.000 1.000
    // @User: Standard

    // @Param: _WL_D
    // @DisplayName: Wheel rate controller (Left) D gain
    // @Description: Wheel rate controller (Left) D gain.  Compensates for short-term change in desired rate vs actual
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _WL_FILT
    // @DisplayName: Wheel rate controller (Left) filter frequency
    // @Description: Wheel rate controller (Left) input filter.  Lower values reduce noise but add delay.
    // @Range: 0.000 100.000
    // @Units: Hz
    // @User: Standard

    // @Param: _WL_FF
    // @DisplayName: Wheel rate controller (Left) feed-forward gain
    // @Description: Wheel rate controller (Left) feed-forward gain.  Converts wheel rate error (as a percentage of full range) to wheel output (in the range -1 to +1)
    // @Range: 0.0 2.000
    // @User: Standard
    AP_SUBGROUPINFO(_wheel_control_state[0].pid, "WL_", 10, AP_MotorsUGV, AC_PID),

    // @Param: _WR_P
    // @DisplayName: Wheel rate controller (Right) P gain
    // @Description: Wheel rate controller (Right) P gain.  Converts wheel rate error (as a percentage of full range) to wheel output (in the range -1 to +1)
    // @Range: 0.100 2.000
    // @User: Standard

    // @Param: _WR_I
    // @DisplayName: Wheel rate controller (Right) I gain
    // @Description: Wheel rate controller (Right) I gain.  Corrects long term error between the desired rate (in rad/s) and actual
    // @Range: 0.000 2.000
    // @User: Standard

    // @Param: _WR_IMAX
    // @DisplayName: Wheel rate controller (Right) I gain maximum
    // @Description: Wheel rate controller (Right) I gain maximum.  Constrains the output (range -1 to +1) that the I term will generate
    // @Range: 0.000 1.000
    // @User: Standard

    // @Param: _WR_D
    // @DisplayName: Wheel rate controller (Right) D gain
    // @Description: Wheel rate controller (Right) D gain.  Compensates for short-term change in desired rate vs actual
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _WR_FILT
    // @DisplayName: Wheel rate controller (Right) filter frequency
    // @Description: Wheel rate controller (Right) input filter.  Lower values reduce noise but add delay.
    // @Range: 0.000 100.000
    // @Units: Hz
    // @User: Standard

    // @Param: _WR_FF
    // @DisplayName: Wheel rate controller (Right) feed-forward gain
    // @Description: Wheel rate controller (Right) feed-forward gain.  Converts wheel rate error (as a percentage of full range) to wheel output (in the range -1 to +1)
    // @Range: 0.0 2.000
    // @User: Standard
    AP_SUBGROUPINFO(_wheel_control_state[1].pid, "WR_", 11, AP_MotorsUGV, AC_PID),

    AP_GROUPEND
};

AP_MotorsUGV::AP_MotorsUGV(AP_ServoRelayEvents &relayEvents, AP_WheelEncoder &wenc) :
        _relayEvents(relayEvents),
        _wenc(wenc)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_MotorsUGV::init()
{
    // setup servo ouput
    setup_servo_output();

    // setup pwm type
    setup_pwm_type();

    // set safety output
    setup_safety_output();
}

// setup output in case of main CPU failure
void AP_MotorsUGV::setup_safety_output()
{
    if (_pwm_type == PWM_TYPE_BRUSHED_WITH_RELAY) {
        // set trim to min to set duty cycle range (0 - 100%) to servo range
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttle);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleLeft);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleRight);
    }

    if (_disarm_disable_pwm) {
        // throttle channels output zero pwm (i.e. no signal)
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
    } else {
        // throttle channels output trim values (because rovers will go backwards if set to MIN)
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
        SRV_Channels::set_safety_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
    }

    // stop sending pwm if main CPU fails
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
}

// setup servo output ranges
void AP_MotorsUGV::setup_servo_output()
{
    // k_steering are limited to -45;45 degree
    SRV_Channels::set_angle(SRV_Channel::k_steering, SERVO_MAX);

    // k_throttle are in power percent so -100 ... 100
    SRV_Channels::set_angle(SRV_Channel::k_throttle, 100);

    // skid steering left/right throttle as -1000 to 1000 values
    SRV_Channels::set_angle(SRV_Channel::k_throttleLeft,  1000);
    SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 1000);
}

// set steering as a value from -4500 to +4500
void AP_MotorsUGV::set_steering(float steering)
{
    _steering = constrain_float(steering, -4500.0f, 4500.0f);
}

// set throttle as a value from -100 to 100
void AP_MotorsUGV::set_throttle(float throttle)
{
    // sanity check throttle min and max
    _throttle_min = constrain_int16(_throttle_min, 0, 20);
    _throttle_max = constrain_int16(_throttle_max, 30, 100);

    // check throttle is between -_throttle_max ~ +_throttle_max but outside -throttle_min ~ +throttle_min
    _throttle = constrain_float(throttle, -_throttle_max, _throttle_max);
    if ((_throttle_min > 0) && (fabsf(_throttle) < _throttle_min)) {
        if (is_negative(_throttle)) {
            _throttle = -_throttle_min;
        } else {
            _throttle = _throttle_min;
        }
    }
}

// inform motor library of maximum vehicle speed
// used for estimate wheel's maximum turn rate which is required for wheel rate control
void AP_MotorsUGV::set_speed_max(float speed_max)
{
    _vehicle_speed_max = speed_max;

    // calculate left and right wheel's maximum rotation rate (vehicle's maximum speed / circumference of wheel)
    for (uint8_t i = 0; i <= 1; i++) {
        if (is_positive(speed_max) && _wenc.enabled(i) && is_positive(_wenc.get_wheel_radius(i))) {
            _wheel_control_state[i].rate_max = speed_max / (2.0f * M_PI * _wenc.get_wheel_radius(i));
        } else {
            _wheel_control_state[i].rate_max = 0.0f;
        }
    }
}

/*
  work out if skid steering is available
 */
bool AP_MotorsUGV::have_skid_steering() const
{
    if (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) &&
        SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
        return true;
    }
    return false;
}

void AP_MotorsUGV::output(bool armed, float dt)
{
    // soft-armed overrides passed in armed status
    if (!hal.util->get_soft_armed()) {
        armed = false;
    }

    // clear and set limits based on input (limit flags may be set again by output_regular or output_skid_steering methods)
    set_limits_from_input(armed, _steering, _throttle);

    // slew limit throttle
    slew_limit_throttle(dt);

    if (have_skid_steering()) {
        // output for skid steering style frames
        output_skid_steering(armed, _steering, _throttle);
    } else {
        // output for regular steering/throttle style frames
        output_regular(armed, _steering, _throttle);
    }

    // send values to the PWM timers for output
    SRV_Channels::calc_pwm();
    SRV_Channels::cork();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
}

// test steering or throttle output as a percentage of the total (range -100 to +100)
// used in response to DO_MOTOR_TEST mavlink command
bool AP_MotorsUGV::output_test_pct(motor_test_order motor_seq, float pct)
{
    // check if the motor_seq is valid
    if (motor_seq > MOTOR_TEST_THROTTLE_RIGHT) {
        return false;
    }
    pct = constrain_float(pct, -100.0f, 100.0f);

    switch (motor_seq) {
        case MOTOR_TEST_THROTTLE: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
                return false;
            }
            output_throttle(SRV_Channel::k_throttle, pct);
            break;
        }
        case MOTOR_TEST_STEERING: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
                return false;
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_steering, pct * 45.0f);
            break;
        }
        case MOTOR_TEST_THROTTLE_LEFT: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft)) {
                return false;
            }
            output_throttle(SRV_Channel::k_throttleLeft, pct);
            break;
        }
        case MOTOR_TEST_THROTTLE_RIGHT: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
                return false;
            }
            output_throttle(SRV_Channel::k_throttleRight, pct);
            break;
        }
        default:
            return false;
    }
    SRV_Channels::calc_pwm();
    SRV_Channels::cork();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
    return true;
}

// test steering or throttle output using a pwm value
bool AP_MotorsUGV::output_test_pwm(motor_test_order motor_seq, float pwm)
{
    // check if the motor_seq is valid
    if (motor_seq > MOTOR_TEST_THROTTLE_RIGHT) {
        return false;
    }
    switch (motor_seq) {
        case MOTOR_TEST_THROTTLE: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
                return false;
            }
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttle, pwm);
            break;
        }
        case MOTOR_TEST_STEERING: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
                return false;
            }
            SRV_Channels::set_output_pwm(SRV_Channel::k_steering, pwm);
            break;
        }
        case MOTOR_TEST_THROTTLE_LEFT: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft)) {
                return false;
            }
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            break;
        }
        case MOTOR_TEST_THROTTLE_RIGHT: {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
                return false;
            }
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            break;
        }
        default:
            return false;
    }
    SRV_Channels::calc_pwm();
    SRV_Channels::cork();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
    return true;
}

//  returns true if checks pass, false if they fail.  report should be true to send text messages to GCS
bool AP_MotorsUGV::pre_arm_check(bool report) const
{
    // check if both regular and skid steering functions have been defined
    if (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) &&
        SRV_Channels::function_assigned(SRV_Channel::k_throttleRight) &&
        SRV_Channels::function_assigned(SRV_Channel::k_throttle) &&
        SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: regular AND skid steering configured");
        }
        return false;
    }
    // check if only one of skid-steering output has been configured
    if (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) != SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: check skid steering config");
        }
        return false;
    }
    // check if only one of throttle or steering outputs has been configured
    if (SRV_Channels::function_assigned(SRV_Channel::k_throttle) != SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
        if (report) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: check steering and throttle config");
        }
        return false;
    }
    return true;
}

// setup pwm output type
void AP_MotorsUGV::setup_pwm_type()
{
    switch (_pwm_type) {
    case PWM_TYPE_ONESHOT:
    case PWM_TYPE_ONESHOT125:
        // tell HAL to do immediate output
        hal.rcout->set_output_mode(AP_HAL::RCOutput::MODE_PWM_ONESHOT);
        break;
    case PWM_TYPE_BRUSHED_WITH_RELAY:
    case PWM_TYPE_BRUSHED_BIPOLAR:
        hal.rcout->set_output_mode(AP_HAL::RCOutput::MODE_PWM_BRUSHED);
        /*
         * Group 0: channels 0 1
         * Group 1: channels 4 5 6 7
         * Group 2: channels 2 3
         */
        // TODO : See if we can seperate frequency between groups
        hal.rcout->set_freq((1UL << 0), static_cast<uint16_t>(_pwm_freq * 1000));  // Steering group
        hal.rcout->set_freq((1UL << 2), static_cast<uint16_t>(_pwm_freq * 1000));  // Throttle group
        break;
    default:
        // do nothing
        break;
    }
}

// output to regular steering and throttle channels
void AP_MotorsUGV::output_regular(bool armed, float steering, float throttle)
{
    // always allow steering to move
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering);

    // output to throttle channels
    if (armed) {
        // handle armed case
        output_throttle(SRV_Channel::k_throttle, throttle);
    } else {
        // handle disarmed case
        if (_disarm_disable_pwm) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
        } else {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
        }
    }
}

// output to skid steering channels
void AP_MotorsUGV::output_skid_steering(bool armed, float steering, float throttle)
{
    // handle simpler disarmed case
    if (!armed) {
        if (_disarm_disable_pwm) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
        } else {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
        }
        return;
    }

    // skid steering mixer
    float steering_scaled = steering / 4500.0f; // steering scaled -1 to +1
    float throttle_scaled = throttle / 100.0f;  // throttle scaled -1 to +1

    // apply constraints
    steering_scaled = constrain_float(steering_scaled, -1.0f, 1.0f);
    throttle_scaled = constrain_float(throttle_scaled, -1.0f, 1.0f);

    // check for saturation and scale back throttle and steering proportionally
    const float saturation_value = fabsf(steering_scaled) + fabsf(throttle_scaled);
    if (saturation_value > 1.0f) {
        steering_scaled = steering_scaled / saturation_value;
        throttle_scaled = throttle_scaled / saturation_value;
    }

    // reverse steering direction if throttle is negative to mimic regular rovers
    const float steering_dir = is_negative(throttle_scaled) ? -1.0f : 1.0f;

    // add in throttle and steering
    const float motor_left = throttle_scaled + (steering_dir * steering_scaled);
    const float motor_right = throttle_scaled - (steering_dir * steering_scaled);

    // send pwm value to each motor
    output_throttle(SRV_Channel::k_throttleLeft, 100.0f * motor_left);
    output_throttle(SRV_Channel::k_throttleRight, 100.0f * motor_right);
}

// output throttle value to main throttle channel, left throttle or right throttle.  throttle should be scaled from -100 to 100
void AP_MotorsUGV::output_throttle(SRV_Channel::Aux_servo_function_t function, float throttle)
{
    // sanity check servo function
    if (function != SRV_Channel::k_throttle && function != SRV_Channel::k_throttleLeft && function != SRV_Channel::k_throttleRight) {
        return;
    }

    // index number is "0" for main throttle and left wheel, "1" for right wheel.  Used to index into relay and wheel encoder objects
    uint8_t index_num = (function == SRV_Channel::k_throttleRight) ? 1 : 0;

    // run pid controller
    throttle = get_rate_controlled_throttle(throttle, index_num);

    // constrain and scale output
    throttle = get_scaled_throttle(throttle);

    // set relay if necessary
    if (_pwm_type == PWM_TYPE_BRUSHED_WITH_RELAY) {
        // find the output channel, if not found return
        const SRV_Channel *out_chan = SRV_Channels::get_channel_for(function);
        if (out_chan == nullptr) {
            return;
        }
        const int8_t reverse_multiplier = out_chan->get_reversed() ? -1 : 1;
        bool relay_high = is_negative(reverse_multiplier * throttle);
        _relayEvents.do_set_relay(index_num, relay_high);
        // invert the output to always have positive value calculated by calc_pwm
        throttle = reverse_multiplier * fabsf(throttle);
    }

    // output to servo channel
    switch (function) {
        case SRV_Channel::k_throttle:
            SRV_Channels::set_output_scaled(function,  throttle);
            break;
        case SRV_Channel::k_throttleLeft:
        case SRV_Channel::k_throttleRight:
            SRV_Channels::set_output_scaled(function,  throttle * 10.0f);
            break;
        default:
            // do nothing
            break;
    }
}

// slew limit throttle for one iteration
void AP_MotorsUGV::slew_limit_throttle(float dt)
{
    if (_slew_rate > 0) {
        // slew throttle
        const float throttle_change_max = MAX(1.0f, _slew_rate * dt * 0.01f * (_throttle_max - _throttle_min));
        if (_throttle > _throttle_prev + throttle_change_max) {
            _throttle = _throttle_prev + throttle_change_max;
            limit.throttle_upper = true;
        } else if (_throttle < _throttle_prev - throttle_change_max) {
            _throttle = _throttle_prev - throttle_change_max;
            limit.throttle_lower = true;
        }
    }
    _throttle_prev = _throttle;
}

// set limits based on steering and throttle input
void AP_MotorsUGV::set_limits_from_input(bool armed, float steering, float throttle)
{
    // set limits based on inputs
    limit.steer_left = !armed || (steering <= -4500.0f);
    limit.steer_right = !armed || (steering >= 4500.0f);
    limit.throttle_lower = !armed || (throttle <= -_throttle_max);
    limit.throttle_upper = !armed || (throttle >= _throttle_max);
}

// scale a throttle using the _thrust_curve_expo parameter.  throttle should be in the range -100 to +100
float AP_MotorsUGV::get_scaled_throttle(float throttle) const
{
    // return immediatley if no scaling
    if (is_zero(_thrust_curve_expo) || (_thrust_curve_expo > 1.0f) || (_thrust_curve_expo < -1.0f)) {
        return throttle;
    }

    // calculate scaler
    const float sign = (throttle < 0.0f) ? -1.0f : 1.0f;
    const float throttle_pct = constrain_float(throttle, -100.0f, 100.0f) / 100.0f;
    return 100.0f * sign * ((_thrust_curve_expo - 1.0f) + safe_sqrt((1.0f - _thrust_curve_expo) * (1.0f - _thrust_curve_expo) + 4.0f * _thrust_curve_expo * fabsf(throttle_pct))) / (2.0f * _thrust_curve_expo);
}

// run wheel rate control
// throttle should be in the range -100 to +100
float AP_MotorsUGV::get_rate_controlled_throttle(float throttle, uint8_t wenc_idx)
{
    // return unmodified throttle if no wheel encoder enabled
    if (!_wenc.enabled(wenc_idx) || is_zero(_wheel_control_state[wenc_idx].rate_max)) {
        return throttle;
    }

    // return immediately if input throttle is zero
    // this is safer because the motors should always stop even if the wheel encoders are setup incorrectly
    if (is_zero(throttle)) {
        return throttle;
    }

    // calculate dt since last iteration
    uint32_t now = AP_HAL::millis();
    float dt = (now - _wheel_control_state[wenc_idx].last_update_ms) / 1000.0f;
    if (dt > 1.0f) {
        dt = 0.0f;
        // initialise limits and PID I term and input filter
        _wheel_control_state[wenc_idx].limit_lower = false;
        _wheel_control_state[wenc_idx].limit_upper = false;
        _wheel_control_state[wenc_idx].pid.reset_I();
        _wheel_control_state[wenc_idx].pid.reset_filter();
    }
    _wheel_control_state[wenc_idx].last_update_ms = now;

    // calculate wheel rotation rate since last update
    float dist = _wenc.get_distance(wenc_idx);
    if (is_positive(dt)) {
        _wheel_control_state[wenc_idx].rate = (dist - _wheel_control_state[wenc_idx].distance_prev) / dt;
        // update pid controller with latest dt value
        _wheel_control_state[wenc_idx].pid.set_dt(dt);
    } else {
        _wheel_control_state[wenc_idx].rate = 0.0f;
    }
    _wheel_control_state[wenc_idx].distance_prev = dist;

    // calculate desired rate as a percentage of maximum rotation rate
    float rate_desired_pct = (throttle * 0.01f);
    float rate_desired = rate_desired_pct * _wheel_control_state[wenc_idx].rate_max;

    // calculate rate error (as a percentage of maximum rotation rate) and pass to pid controller
    float rate_error_pct = (rate_desired - _wheel_control_state[wenc_idx].rate) / _wheel_control_state[wenc_idx].rate_max;
    _wheel_control_state[wenc_idx].pid.set_input_filter_all(rate_error_pct);

    // record desired and actual rate for dataflash logging
    _wheel_control_state[wenc_idx].pid.set_desired_rate(rate_desired);
    _wheel_control_state[wenc_idx].pid.set_actual_rate(_wheel_control_state[wenc_idx].rate);

    // get feed forward
    float ff = _wheel_control_state[wenc_idx].pid.get_ff(rate_desired_pct);

    // get p
    float p = _wheel_control_state[wenc_idx].pid.get_p();

    // get i unless winch hit limit on last iteration
    float i = _wheel_control_state[wenc_idx].pid.get_integrator();
    if (((is_negative(rate_desired_pct) && !_wheel_control_state[wenc_idx].limit_lower) || (is_positive(rate_desired_pct) && !_wheel_control_state[wenc_idx].limit_upper))) {
        i = _wheel_control_state[wenc_idx].pid.get_i();
    }

    // get d
    float d = _wheel_control_state[wenc_idx].pid.get_d();

    // final output uses input throttle as base and adds correction to achieve desired rate
    // multiplied by 100 to convert from -1 to +1 range to -100 to +100 range
    float output_total = throttle + ((ff + p + i + d) * 100.0f);

    // constrain and set limit flags
    _wheel_control_state[wenc_idx].limit_lower = (output_total <= -100.0f);
    _wheel_control_state[wenc_idx].limit_upper = (output_total >= 100.0f);
    output_total = constrain_float(output_total, -100.0f, 100.0f);

    // return controlled throttle
    return output_total;
}
