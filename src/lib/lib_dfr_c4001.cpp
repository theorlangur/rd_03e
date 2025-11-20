#define FORCE_FMT
#define PRINTF_FUNC(...) printk(__VA_ARGS__)

#include <algorithm>
#include <cstring>
#include "lib_dfr_c4001.h"
#include "lib_misc_helpers.hpp"

#define DBG_UART Channel::DbgNow _dbg_uart{this}; 
#define DBG_ME DbgNow _dbg_me{this}; 

namespace dfr
{
    C4001::C4001(const struct device *pUART):
        uart::Channel(pUART)
    {
    }

    C4001::ExpectedResult C4001::Init()
    {
        SetDefaultWait(kDefaultWait);
        TRY_UART_COMM(Configure(), "Init");
        TRY_UART_COMM(Open(), "Init");
        return ReloadConfig();
    }

    C4001::ExpectedResult C4001::ReloadConfig()
    {
        TRY_UART_COMM(StopSensor(), "ReloadConfig.StopSensor");
        RxBlock rx(*this);
        TRY_UART_COMM(UpdateHWVersion(), "ReloadConfig.UpdateHWVersion");
        TRY_UART_COMM(UpdateSWVersion(), "ReloadConfig.UpdateSWVersion");
        TRY_UART_COMM(StartSensor(), "ReloadConfig.StartSensor");
        return std::ref(*this);
    }

    C4001::ExpectedResult C4001::Restart()
    {
        using namespace uart::primitives;
        return std::ref(*this);
    }

    C4001::ExpectedResult C4001::FactoryReset()
    {
        using namespace uart::primitives;
        SetDefaultWait(duration_ms_t(1000));
        return ReloadConfig();
    }

    C4001::ExpectedResult C4001::StopSensor()
    {
        TRY_UART_COMM(SendCmdNoResp(kCmdSensorStop), "");
        return std::ref(*this);
    }

    C4001::ExpectedResult C4001::StartSensor()
    {
        m_Dbg = true;
        //ChangeWait longerWait(*this, 300);
        TRY_UART_COMM(SendCmd(kCmdSensorStart), "");
        m_Dbg = false;
        return std::ref(*this);
    }

    C4001::ExpectedResult C4001::UpdateHWVersion()
    {
        using namespace uart::primitives;
        std::fill(std::begin(m_HWVersion.m_Version), std::end(m_HWVersion.m_Version), 0);
        TRY_UART_COMM(SendCmdWithParams(to_send(kCmdGetHWVersion), to_recv(
                        find_str_t{"HardwareVersion:"},
                        read_until_t{m_HWVersion.m_Version, '\r'}
                        )), "");
        *(std::end(m_HWVersion.m_Version) - 1) = 0;
        return std::ref(*this);
    }

    C4001::ExpectedResult C4001::UpdateSWVersion()
    {
        using namespace uart::primitives;
        std::fill(std::begin(m_SWVersion.m_Version), std::end(m_SWVersion.m_Version), 0);
        TRY_UART_COMM(SendCmdWithParams(to_send(kCmdGetSWVersion), to_recv(
                        find_str_t{"SoftwareVersion:"},
                        read_until_t{m_SWVersion.m_Version, '\r'}
                        )), "");
        *(std::end(m_SWVersion.m_Version) - 1) = 0;
        return std::ref(*this);
    }
}
