#pragma once

#include <stdint.h>

#define AP_RTC_DEBUG 0

class AP_RTC {

public:

    AP_RTC() {
        if (_singleton != nullptr) {
            // it's an error to get here.  But I don't want to include
            // AP_HAL here
            return;
        }
        _singleton = this;
    }

    // ordering is important in source_type; lower-numbered is
    // considered a better time source.
    enum source_type : uint8_t {
        SOURCE_GPS,
        SOURCE_MAVLINK_SYSTEM_TIME,
        SOURCE_HW,
    };

    /*
      get clock in UTC microseconds

      non-const as it updates the RTC shift as part of retrieving the value
     */
    uint64_t get();

    /*
      get time in UTC hours, minutes, seconds and milliseconds
     */
    void get_system_clock_utc(int32_t &hour, int32_t &min, int32_t &sec, int32_t &ms);

    uint32_t get_time_utc(int32_t hour, int32_t min, int32_t sec, int32_t ms);

    // get singleton instance
    static AP_RTC *get_singleton() {
        return _singleton;
    }

    static const uint8_t _max_backends = 6; // at least one for each mavlink channel
    class AP_RTC_Backend *_backends[_max_backends];
    class AP_RTC_Backend *register_rtc_source(source_type type);

    static const char *_clock_source_types[];

private:

    static AP_RTC *_singleton;

    uint32_t last_report_time_ms;

    uint64_t last_drift_update;
    void update_rtc_shift();

    // GPS should give us time every 100ms, so 250ms seems a reasonable timeout
    const uint32_t rtc_source_timeout_us = 250000;

    // possibly send an update to the GCS
    const uint16_t gcs_update_interval_ms = 10000;
    void check_gcs_update();

    int64_t rtc_shift; // master clock RTC shift.  We adjust this
                       // based on our best time source.

    // microseconds-since-boot since we last drifted the clock.  Used
    // to ensure time doesn't go backwards:
    uint64_t last_rtc_shift_update;
};

namespace AP {
    AP_RTC &rtc();
};
