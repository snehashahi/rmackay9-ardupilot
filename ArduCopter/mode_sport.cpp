#include "Copter.h"

/*
 * Init and run calls for sport flight mode
 */

// sport_init - initialise sport controller
bool Copter::ModeSport::init(bool ignore_checks)
{
    // initialize vertical speed and acceleration
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    return true;
}

// sport_run - runs the sport controller
// should be called at 100hz or more
void Copter::ModeSport::run()
{
    SportModeState sport_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform
    update_simple_mode();

    // get pilot's desired roll and pitch rates

    // calculate rate requests
    float target_roll_rate = channel_roll->get_control_in() * g.acro_rp_p;
    float target_pitch_rate = channel_pitch->get_control_in() * g.acro_rp_p;

    // get attitude targets
    const Vector3f att_target = attitude_control->get_att_target_euler_cd();

    // Calculate trainer mode earth frame rate command for roll
    int32_t roll_angle = wrap_180_cd(att_target.x);
    target_roll_rate -= constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_roll;

    // Calculate trainer mode earth frame rate command for pitch
    int32_t pitch_angle = wrap_180_cd(att_target.y);
    target_pitch_rate -= constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_pitch;

    AP_Vehicle::MultiCopter &aparm = copter.aparm;
    if (roll_angle > aparm.angle_max){
        target_roll_rate -=  g.acro_rp_p*(roll_angle-aparm.angle_max);
    }else if (roll_angle < -aparm.angle_max) {
        target_roll_rate -=  g.acro_rp_p*(roll_angle+aparm.angle_max);
    }

    if (pitch_angle > aparm.angle_max){
        target_pitch_rate -=  g.acro_rp_p*(pitch_angle-aparm.angle_max);
    }else if (pitch_angle < -aparm.angle_max) {
        target_pitch_rate -=  g.acro_rp_p*(pitch_angle+aparm.angle_max);
    }

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // State Machine Determination
    if (motors->get_spool_mode() == AP_Motors::SHUT_DOWN) {
        sport_state = Sport_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered(target_climb_rate)) {
        sport_state = Sport_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        sport_state = Sport_Landed;
    } else {
        sport_state = Sport_Flying;
    }

    // State Machine
    switch (sport_state) {

    case Sport_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#if FRAME_CONFIG == HELI_FRAME
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
#else
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        break;

    case Sport_Takeoff:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        break;

    case Sport_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
#if FRAME_CONFIG == HELI_FRAME
        if ((target_climb_rate < 0.0f) && !motors->get_interlock()) {
#else
        if (target_climb_rate < 0.0f) {
#endif
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        break;

    case Sport_Flying:
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // adjust climb rate using rangefinder
        if (copter.rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_rate_roll_pitch_yaw(target_roll_rate, target_pitch_rate, target_yaw_rate);

    // call z-axis position controller
    pos_control->update_z_controller();
}
