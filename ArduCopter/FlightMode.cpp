#include "Copter.h"
#include "FlightMode.h"

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Copter::FlightMode::get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max)
{
    // sanity check angle max parameter
    _copter.aparm.angle_max = constrain_int16(_copter.aparm.angle_max,1000,8000);

    // limit max lean angle
    angle_max = constrain_float(angle_max, 1000, _copter.aparm.angle_max);

    // scale roll_in, pitch_in to ANGLE_MAX parameter range
    float scaler = _copter.aparm.angle_max/(float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_in *= scaler;
    pitch_in *= scaler;

    // do circular limit
    float total_in = norm(pitch_in, roll_in);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_in = (18000/M_PI) * atanf(cosf(pitch_in*(M_PI/18000))*tanf(roll_in*(M_PI/18000)));

    // return
    roll_out = roll_in;
    pitch_out = pitch_in;
}
