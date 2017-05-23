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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

/*
  base class constructor.
  This incorporates initialisation as well.
*/
AP_BattMonitor_Backend::AP_BattMonitor_Backend(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state) :
        _mon(mon),
        _state(mon_state)
{
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor_Backend::capacity_remaining_pct() const
{
    float mah_remaining = _mon._pack_capacity[_state.instance] - _state.current_total_mah;
    if ( _mon._pack_capacity[_state.instance] > 10 ) { // a very very small battery
        return (100 * (mah_remaining) / _mon._pack_capacity[_state.instance]);
    } else {
        return 0;
    }
}

/// get capacity for this instance
int32_t AP_BattMonitor_Backend::get_capacity() const
{
    return _mon.pack_capacity_mah(_state.instance);
}

// update battery resistance estimate.  throttle_above_threshold should be true if throttle is high enough to calculate the estimate
void AP_BattMonitor_Backend::update_resistance_estimate(bool resting, bool throttle_above_threshold)
{
    // get current system time
    uint32_t now = AP_HAL::millis();

    // if disarmed reset resting voltage and current
    if (resting) {
        _state.voltage_resting = _state.voltage;
        _state.current_resting = _state.current_amps;
        _state.resistance_timer_ms = now;
        return;
    }

    if (_state.voltage_resting > _state.voltage && _state.current_resting < _state.current_amps) {
        uint32_t time_diff_ms = now - _state.resistance_timer_ms;
        // update battery resistance when throttle is over hover throttle
        float batt_resistance = (_state.voltage_resting-_state.voltage)/(_state.current_amps - _state.current_resting);
        if ((time_diff_ms < 1000) && ((_state.current_resting*2.0f) < _state.current_amps)) {
            if (throttle_above_threshold) {
                // filter reaches 90% in 1/4 the test time
                _state.resistance += 0.05f*(batt_resistance - _state.resistance);
            } else {
                // initialize battery resistance to prevent change in resting voltage estimate
                _state.resistance_timer_ms = now;
                _state.resistance = batt_resistance;
            }
        }
        // make sure battery resistance value doesn't result in the predicted battery voltage exceeding the resting voltage
        if (batt_resistance < _state.resistance && throttle_above_threshold){
            _state.resistance = batt_resistance;
        }
    }
}
