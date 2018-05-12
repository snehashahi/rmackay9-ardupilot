#include "AP_RTC.h"

#include "AP_RTC_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const char *AP_RTC::_clock_source_types[] = {
    "GPS",
    "SYSTEM_TIME",
    "HW",
};

uint64_t AP_RTC::get()
{
    update_rtc_shift();

    const uint32_t now = AP_HAL::millis();

    const AP_RTC_Backend *best_backend = nullptr;
    int64_t bb_shift = 0;

    // preferentially return GPS-based RTC if one is available:
    for (uint8_t i=0; i<_max_backends; i++) {
        const AP_RTC_Backend *backend = _backends[i];
        if (backend == nullptr) {
            continue;
        }
        int64_t shift;
        if (!backend->get_rtc_shift(shift)) {
            continue;
        }
        if (best_backend != nullptr) { // we always want to return something; bypass checks for first set source
            const int64_t reasonable_rtc_shift = 1500000000000000;
            if (rtc_shift < reasonable_rtc_shift) {
                // e.g. GPS with no satellites
                continue;
            }
            if (now - backend->last_reading_ms > rtc_source_timeout_us) {
                // no reading in too long.  Something is wrong...
                continue;
            }
            // if the existing best-backend has time way, way in the
            // past then we will use this new time source
            if (bb_shift > 0 && bb_shift > reasonable_rtc_shift) {
                if (backend->type >= best_backend->type) {
                    // this source is same as / inferior to current return value
                    continue;
                }
            }
        }
        best_backend = backend;
        bb_shift = shift;
    }

    uint64_t ret = AP_HAL::micros64();
    if (best_backend != nullptr) {
        if (last_rtc_shift_update == 0) {
            rtc_shift = bb_shift;
        } else {
            const int64_t delta = bb_shift - rtc_shift;
            if (delta > 1000000000) {
                // if the jump is huge-and-positive then we assume
                // someone's moved from a non-epoch-time to an
                // epoch-time and we step-jump to match them
                rtc_shift = bb_shift;
            } else if (delta < -1000000000) {
                // ignore any "best source" far in the past
            } else {
                // drift the master clock towards this "best source"
                const uint64_t micros_since_last_drift_update = now - last_rtc_shift_update;
                // we should consider scaling adjustment here
                int32_t max_adjustment = micros_since_last_drift_update;
                if (max_adjustment > 5000) {
                    max_adjustment = 5000;
                }
                const int64_t adjustment = constrain_int64(delta,
                                                           -max_adjustment,
                                                           max_adjustment);
                rtc_shift += adjustment;
            }
        }
        last_rtc_shift_update = now;
    }

    ret += rtc_shift;

    // update the hardware clock:
    hal.util->set_hw_rtc(ret);

    // update signing timestamp
    GCS_MAVLINK::update_signing_timestamp(ret);

    return ret;
}

// possibly send an update to the GCS
void AP_RTC::check_gcs_update()
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_report_time_ms < gcs_update_interval_ms) {
        return;
    }
    for (uint8_t i=0; i<_max_backends; i++) {
        const AP_RTC_Backend *backend = _backends[i];
        if (backend == nullptr) {
            continue;
        }
        backend->gcs_update(i);
    }
    last_report_time_ms = now;
}

void AP_RTC::update_rtc_shift()
{
    const uint64_t now = AP_HAL::micros64();
    if (last_drift_update == 0) {
        last_drift_update = now;
        return;
    }
    const uint64_t micros_since_last_drift_update = now - last_drift_update;
    last_drift_update = now;

    // make microsecond adjustments to rtc shifts based on average
    // received shifts
    for (uint8_t i=0; i<_max_backends; i++) {
        AP_RTC_Backend *backend = _backends[i];
        if (backend == nullptr) {
            continue;
        }
        backend->update_rtc_shift(micros_since_last_drift_update);
    }
#if AP_RTC_DEBUG
    check_gcs_update();
#endif
}

void AP_RTC::get_system_clock_utc(int32_t &hour, int32_t &min, int32_t &sec, int32_t &ms)
{
     // get time of day in ms
    const uint64_t time_ms = get() / 1000U;

    // separate time into ms, sec, min, hour and days but all expressed in milliseconds
    ms = time_ms % 1000;
    uint32_t sec_ms = (time_ms % (60 * 1000)) - ms;
    uint32_t min_ms = (time_ms % (60 * 60 * 1000)) - sec_ms - ms;
    uint32_t hour_ms = (time_ms % (24 * 60 * 60 * 1000)) - min_ms - sec_ms - ms;

    // convert times as milliseconds into appropriate units
    sec = sec_ms / 1000;
    min = min_ms / (60 * 1000);
    hour = hour_ms / (60 * 60 * 1000);
}

// get milliseconds from now to a target time of day expressed as hour, min, sec, ms
// match starts from first value that is not -1. I.e. specifying hour=-1, minutes=10 will ignore the hour and return time until 10 minutes past 12am (utc)
uint32_t AP_RTC::get_time_utc(int32_t hour, int32_t min, int32_t sec, int32_t ms)
{
    // determine highest value specified (0=none, 1=ms, 2=sec, 3=min, 4=hour)
    int8_t largest_element = 0;
    if (hour != -1) {
        largest_element = 4;
    } else if (min != -1) {
        largest_element = 3;
    } else if (sec != -1) {
        largest_element = 2;
    } else if (ms != -1) {
        largest_element = 1;
    } else {
        // exit immediately if no time specified
        return 0;
    }

    // get start_time_ms as h, m, s, ms
    int32_t curr_hour, curr_min, curr_sec, curr_ms;
    get_system_clock_utc(curr_hour, curr_min, curr_sec, curr_ms);
    int32_t total_delay_ms = 0;

    // calculate ms to target
    if (largest_element >= 1) {
        total_delay_ms += ms - curr_ms;
    }
    if (largest_element == 1 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms += 1000);
    }

    // calculate sec to target
    if (largest_element >= 2) {
        total_delay_ms += (sec - curr_sec)*1000;
    }
    if (largest_element == 2 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms += (60*1000));
    }

    // calculate min to target
    if (largest_element >= 3) {
        total_delay_ms += (min - curr_min)*60*1000;
    }
    if (largest_element == 3 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms += (60*60*1000));
    }

    // calculate hours to target
    if (largest_element >= 4) {
        total_delay_ms += (hour - curr_hour)*60*60*1000;
    }
    if (largest_element == 4 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms += (24*60*60*1000));
    }

    // total delay in milliseconds
    return static_cast<uint32_t>(total_delay_ms);
}

/*
  convert a source time to our flight controller system time
 */
bool AP_RTC::convert_to_system_time(source_type type, uint64_t source_time_usec, uint64_t &system_time_usec)
{
    // get time in order to update state
    get();

    // loop through backends looking for mavlink source
    for (uint8_t i=0; i<_max_backends; i++) {
        const AP_RTC_Backend *backend = _backends[i];
        if (backend == nullptr) {
            continue;
        }
        if (backend->type == type) {
            // get this backend's shift
            int64_t shift;
            if (!backend->get_rtc_shift(shift)) {
                continue;
            }
            // sanity check, source
            const int64_t reasonable_rtc_shift = 1500000000000000;
            if (rtc_shift < reasonable_rtc_shift) {
                // e.g. GPS with no satellites
                continue;
            }
            if ((AP_HAL::millis() - backend->last_reading_ms) > rtc_source_timeout_us) {
                // no reading in too long.  Something is wrong...
                continue;
            }
            // convert time to system time by removing shift
            system_time_usec = source_time_usec - shift;
            return true;
        }
    }

    return false;
}

AP_RTC_Backend *AP_RTC::register_rtc_source(AP_RTC::source_type type)
{
    uint8_t i;
    bool have_slot = false;
    for (i=0; i<_max_backends; i++) {
        if (_backends[i] == nullptr) {
            have_slot = true;
            break;
        }
    }
    if (!have_slot) {
        gcs().send_text(MAV_SEVERITY_WARNING, "RTC: too many clock sources");
        return nullptr;
    }

#if AP_RTC_DEBUG
    gcs().send_text(MAV_SEVERITY_INFO, "RTC: new %s clock source",
        _clock_source_types[type]);
#endif

    _backends[i] = new AP_RTC_Backend(type); // subclass when required
    if (_backends[i] == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "RTC: new failed");
        return nullptr;
    }
    return _backends[i];
}

// singleton instance
AP_RTC *AP_RTC::_singleton;

namespace AP {

AP_RTC &rtc()
{
    return *AP_RTC::get_singleton();
}

}
