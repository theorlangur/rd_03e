#ifndef C4001_TASK_HPP_
#define C4001_TASK_HPP_
#include "lib/lib_dfr_c4001.h"

namespace c4001
{
    enum class err_t
    {
        Ok,
        Range,
        RangeTrig,
        Delay,
        Sensitivity,
        InhibitDuration
    };
    using err_callback_t = void(*)(err_t);
    dfr::C4001* setup(err_callback_t err);

    void set_range(float from, float to);
    void set_range_trig(float trig);
    void set_detect_delay(float v);
    void set_clear_delay(float v);
    void set_detect_clear_delay(float detect, float clear);
    void set_detect_sensitivity(uint8_t s);
    void set_hold_sensitivity(uint8_t s);
    void set_sensitivity(uint8_t detect, uint8_t hold);
    void set_inhibit_duration(float dur);
}

#endif
