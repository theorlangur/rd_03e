#ifndef ZB_C4001_CLUSTER_DESC_HPP_
#define ZB_C4001_CLUSTER_DESC_HPP_

#include <nrfzbcpp/zb_main.hpp>

extern "C"
{
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zb_nrf_platform.h>
}

namespace zb
{
    static constexpr uint16_t kZB_ZCL_CLUSTER_ID_C4001 = 0xfc80;
    struct zb_zcl_c4001_t
    {
        float range_min = 0.6f;
        float range_max = 25;
        float range_trig = 8;
    };

    template<> struct zcl_description_t<zb_zcl_c4001_t> {
        static constexpr auto get()
        {
            using T = zb_zcl_c4001_t;
            return cluster_t<
                cluster_info_t{.id = kZB_ZCL_CLUSTER_ID_C4001},
                attributes_t<
                    attribute_t{.m = &T::range_min, .id = 0x0000, .a=Access::RW},
                    attribute_t{.m = &T::range_max, .id = 0x0001, .a=Access::RW},
                    attribute_t{.m = &T::range_trig,.id = 0x0002, .a=Access::RW}
                >{}
            >{};
        }
    };
}
#endif
