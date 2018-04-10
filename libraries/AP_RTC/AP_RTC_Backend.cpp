#include "AP_RTC_Backend.h"

#include "AP_RTC.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Scheduler/AP_Scheduler.h>

bool AP_RTC_Backend::get_rtc_shift(int64_t &ret) const
{
    if (!is_set) {
        return false;
    }
    ret = rtc_shift;
    return true;
}

// static int64_t first_delta;

// static int64_t last_delta;
// static uint64_t last_micros64;
// static int64_t last_time_utc_usec;
void AP_RTC_Backend::set(const uint64_t time_utc_usec)
{
    const uint64_t now = AP_HAL::micros64();
    const int64_t measured_rtc_shift = time_utc_usec - now;

    bool do_set = false;
    if (!is_set) {
        do_set = true;
    }
    int64_t rtc_shift_error = measured_rtc_shift - rtc_shift;
    if (rtc_shift_error > 100000000) {
        // this is a step-change in the time input.  This probably
        // means a GPS has seen a satellite.
        do_set = true;
    }
    if (do_set) {
        rtc_shift = measured_rtc_shift;
        measured_rtc_shift_average = measured_rtc_shift;
        rtc_shift_error = 0;
        is_set = true;
    } else {
        // we smooth the readings here to reduce the impact of noise
        measured_rtc_shift_average += rtc_shift_error/8;
    }

    // spread adjustment over an interval equal to
    const uint64_t meas_interval_us = (AP_HAL::millis() - last_reading_ms)*1000;
    const uint32_t loops_to_fix_things = meas_interval_us/AP::scheduler().get_loop_period_us();
    adjustment_interval_loopcount = 0;
    per_loop_adjustment_us = 0;
    if (rtc_shift_error == 0) {
        // even a stopped clock is right....
    } else if (abs(rtc_shift_error) > loops_to_fix_things) {
        per_loop_adjustment_us = -rtc_shift_error/loops_to_fix_things;
    } else {
        adjustment_interval_loopcount = loops_to_fix_things / abs(rtc_shift_error);
    }
#if AP_RTC_DEBUG > 2
    if (type == AP_RTC::SOURCE_MAVLINK_SYSTEM_TIME) {
        gcs().send_text(MAV_SEVERITY_INFO, "micros64=%llu", now);
        gcs().send_text(MAV_SEVERITY_INFO, "rtc_shift=%lld", rtc_shift);
        gcs().send_text(MAV_SEVERITY_INFO, "measured_rtc_shift=%lld", measured_rtc_shift);
        gcs().send_text(MAV_SEVERITY_INFO, "meas_interval_us=%llu", meas_interval_us);
        gcs().send_text(MAV_SEVERITY_INFO, "rtc_shift_error=%lld", rtc_shift_error);
        gcs().send_text(MAV_SEVERITY_INFO, "loops_ti_fix_things=%u", loops_to_fix_things);
        gcs().send_text(MAV_SEVERITY_INFO, "adjustment_interval_loopcount=%u", adjustment_interval_loopcount);
        gcs().send_text(MAV_SEVERITY_INFO, "per_loop_adjustment_us=%d\n", per_loop_adjustment_us);
        gcs().send_text(MAV_SEVERITY_INFO, "total_delta_microseconds time=%llu", now-total_delta_microseconds_since);
        gcs().send_text(MAV_SEVERITY_INFO, "total_delta_microseconds=%lld", total_delta_microseconds);
        if (total_delta_microseconds != 0) {
            // const uint32_t tmp = now-total_delta_microseconds_since;
            // const int32_t tmp2 = total_delta_microseconds;
            const int64_t average = ((int64_t)((now-total_delta_microseconds_since)))/total_delta_microseconds;
            gcs().send_text(MAV_SEVERITY_INFO, "average error=%lld\n", average);
        }
    }
#endif

    last_reading_ms = AP_HAL::millis();

    DataFlash_Class::instance()->Log_Write("RTCB",
                                           "TimeUS,Meas,Pred",
                                           "QQQ",
                                           now,
                                           time_utc_usec,
                                           rtc_shift);
}

void AP_RTC_Backend::update_rtc_shift(const uint64_t micros_since_last_drift_update)
{
    if (!is_set) {
        return;
    }
    // don't allow time to go backwards:
    int64_t adjustment = -per_loop_adjustment_us;
    if (adjustment_interval_loopcount) {
        adjustment_interval_loopcount_count++;
        if (adjustment_interval_loopcount_count == adjustment_interval_loopcount) {
            const int64_t delta = measured_rtc_shift_average - rtc_shift;
            adjustment += (delta > 0) ? 1 : -1;
            adjustment_interval_loopcount_count = 0;
        }
    }

    if (total_delta_microseconds_since == 0) {
        total_delta_microseconds_since = AP_HAL::micros();
    }
    adjustment = constrain_int64(adjustment, -micros_since_last_drift_update, micros_since_last_drift_update);
    total_delta_microseconds += adjustment;
    rtc_shift += adjustment;
}

#if AP_RTC_DEBUG
void AP_RTC_Backend::gcs_update(uint8_t offset) const
{
    gcs().send_text(MAV_SEVERITY_INFO, "RTC: %u %llu skew=%lld (%s)",
                    offset,
                    rtc_shift + AP_HAL::micros64(),
                    total_delta_microseconds,
                    AP_RTC::_clock_source_types[(uint8_t)type]);
}
#endif
