#pragma once

#include "AP_RTC.h"

class AP_RTC_Backend {

public:

    AP_RTC_Backend(AP_RTC::source_type _type) :
        type(_type) {}

    /*
      set clock in UTC microseconds
    */
    void set(uint64_t time_utc_usec);
    bool get_rtc_shift(int64_t &time_utc_usec) const;

    // call update_rtc_shift at the main loop rate:
    void update_rtc_shift(uint64_t micros_since_last_drift_update);
    const AP_RTC::source_type type;
    uint32_t last_reading_ms; // boot-time since last reading received

    void gcs_update(uint8_t offset) const;

    int32_t per_loop_adjustment_us;
    uint16_t adjustment_interval_loopcount;
    uint16_t adjustment_interval_loopcount_count;


private:

    bool is_set;        // by default we consider system_time was not set
    int64_t rtc_shift; // delta between epoch and board boot time
    int64_t measured_rtc_shift_average; // smoothed readings supplied by e.g. GPS or SYSTEM_TIME message
    int64_t total_delta_microseconds; // total number of microseconds added or removed
    uint64_t total_delta_microseconds_since;

};
